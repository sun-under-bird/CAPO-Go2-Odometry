// capo_evaluator_node.cpp
// CAPO 里程计 vs MuJoCo Ground Truth 定量误差评估节点（阶段十七新增，仅仿真验证用）。
//
// 输入：
//   /SMX/Odom_2D     CAPO 2D 里程计（nav_msgs/Odometry）
//   /ground_truth/odom  仿真真值（ground_truth_node 发布）
//
// 输出：
//   周期性（默认 5 s）在日志中打印当前误差与累计统计；
//   节点退出（Ctrl-C）时打印最终评估报告；
//   同时把逐帧配对数据写入 CSV（默认 /tmp/capo_eval.csv）供离线分析。
//
// 第一版指标（与计划一致）：
//   x/y/z/yaw 瞬时误差、终点位置误差、累计行程、相对位置误差百分比、
//   位置 RMSE（2D/3D，即未做轨迹对齐的 ATE 近似）、yaw RMSE。

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace
{

// 从四元数提取 yaw（ZYX 欧拉角）
inline double yaw_from_quaternion(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// 误差角归一化到 [-pi, pi]
inline double wrap_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

}  // namespace

class CapoEvaluatorNode : public rclcpp::Node
{
public:
    explicit CapoEvaluatorNode(const rclcpp::NodeOptions & options)
    : Node("capo_evaluator_node", options)
    {
        this->get_parameter_or("capo_topic", capo_topic_, std::string("SMX/Odom_2D"));
        this->get_parameter_or("gt_topic", gt_topic_, std::string("ground_truth/odom"));
        this->get_parameter_or("report_period_s", report_period_s_, 5.0);
        this->get_parameter_or("csv_path", csv_path_, std::string("/tmp/capo_eval.csv"));

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        capo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            capo_topic_, qos,
            std::bind(&CapoEvaluatorNode::capo_callback, this, std::placeholders::_1));
        gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            gt_topic_, qos,
            std::bind(&CapoEvaluatorNode::gt_callback, this, std::placeholders::_1));

        report_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(report_period_s_),
            std::bind(&CapoEvaluatorNode::report, this));

        // CSV 表头：时间戳、CAPO x/y/z/yaw、GT x/y/z/yaw
        if (!csv_path_.empty()) {
            csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
            if (csv_file_.is_open()) {
                csv_file_ << "t,capo_x,capo_y,capo_z,capo_yaw,gt_x,gt_y,gt_z,gt_yaw\n";
            } else {
                RCLCPP_WARN(this->get_logger(), "无法打开 CSV 文件：%s", csv_path_.c_str());
            }
        }

        RCLCPP_INFO(
            this->get_logger(), "评估节点已启动：CAPO[%s] vs GT[%s]",
            capo_topic_.c_str(), gt_topic_.c_str());
    }

    ~CapoEvaluatorNode() override
    {
        // 退出时输出最终报告
        if (sample_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), "%s", final_report().c_str());
        }
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    std::string capo_topic_;
    std::string gt_topic_;
    double report_period_s_ = 5.0;
    std::string csv_path_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr capo_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    std::ofstream csv_file_;
    std::mutex data_mutex_;

    nav_msgs::msg::Odometry::SharedPtr latest_gt_;

    // 统计量
    size_t sample_count_ = 0;
    bool gt_path_init_ = false;
    double gt_path_length_ = 0.0;       // GT 累计行程
    double last_gt_x_ = 0.0, last_gt_y_ = 0.0;
    double sum_sq_pos_ = 0.0;           // 3D 位置误差平方和
    double sum_sq_pos2d_ = 0.0;         // 2D 位置误差平方和
    double sum_sq_yaw_ = 0.0;           // yaw 误差平方和
    double last_err_x_ = 0.0, last_err_y_ = 0.0, last_err_z_ = 0.0, last_err_yaw_ = 0.0;
    double start_gt_x_ = 0.0, start_gt_y_ = 0.0;   // GT 起点（用于终点位移）
    double last_capo_x_ = 0.0, last_capo_y_ = 0.0;
    double capo_path_length_ = 0.0;     // CAPO 累计行程
    bool capo_path_init_ = false;

    void gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 累计 GT 行程（欧式距离）
        if (gt_path_init_) {
            gt_path_length_ += std::hypot(
                msg->pose.pose.position.x - last_gt_x_,
                msg->pose.pose.position.y - last_gt_y_);
        } else {
            start_gt_x_ = msg->pose.pose.position.x;
            start_gt_y_ = msg->pose.pose.position.y;
            gt_path_init_ = true;
        }
        last_gt_x_ = msg->pose.pose.position.x;
        last_gt_y_ = msg->pose.pose.position.y;
        latest_gt_ = msg;
    }

    // 每帧 CAPO 里程计与最近一帧 GT 配对并累计误差
    void capo_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_gt_) return;

        const auto & cp = msg->pose.pose.position;
        const auto & gq = latest_gt_->pose.pose.orientation;
        const auto & gp = latest_gt_->pose.pose.position;
        const auto & cq = msg->pose.pose.orientation;

        const double capo_yaw = yaw_from_quaternion(cq.w, cq.x, cq.y, cq.z);
        const double gt_yaw = yaw_from_quaternion(gq.w, gq.x, gq.y, gq.z);
        const double ex = cp.x - gp.x;
        const double ey = cp.y - gp.y;
        const double ez = cp.z - gp.z;
        const double eyaw = wrap_angle(capo_yaw - gt_yaw);

        last_err_x_ = ex; last_err_y_ = ey; last_err_z_ = ez; last_err_yaw_ = eyaw;
        sum_sq_pos_ += ex * ex + ey * ey + ez * ez;
        sum_sq_pos2d_ += ex * ex + ey * ey;
        sum_sq_yaw_ += eyaw * eyaw;
        ++sample_count_;

        // CAPO 累计行程
        if (capo_path_init_) {
            capo_path_length_ += std::hypot(cp.x - last_capo_x_, cp.y - last_capo_y_);
        } else {
            capo_path_init_ = true;
        }
        last_capo_x_ = cp.x;
        last_capo_y_ = cp.y;

        if (csv_file_.is_open()) {
            csv_file_ << this->now().seconds() << ","
                      << cp.x << "," << cp.y << "," << cp.z << "," << capo_yaw << ","
                      << gp.x << "," << gp.y << "," << gp.z << "," << gt_yaw << "\n";
        }
    }

    // 周期性中间报告
    void report()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (sample_count_ == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 10000,
                "尚未同时收到 CAPO 与 Ground Truth 数据");
            return;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "[中期] 样本 %zu | 当前误差 x=%.3f y=%.3f z=%.3f yaw=%.3f rad | "
            "RMSE 2D=%.3f 3D=%.3f yaw=%.3f | 行程 GT=%.2f m CAPO=%.2f m",
            sample_count_, last_err_x_, last_err_y_, last_err_z_, last_err_yaw_,
            std::sqrt(sum_sq_pos2d_ / sample_count_),
            std::sqrt(sum_sq_pos_ / sample_count_),
            std::sqrt(sum_sq_yaw_ / sample_count_),
            gt_path_length_, capo_path_length_);
    }

    // 最终评估报告（节点退出时输出）
    std::string final_report()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const double rmse2d = std::sqrt(sum_sq_pos2d_ / sample_count_);
        const double rmse3d = std::sqrt(sum_sq_pos_ / sample_count_);
        const double rmse_yaw = std::sqrt(sum_sq_yaw_ / sample_count_);
        const double endpoint_err = std::sqrt(
            last_err_x_ * last_err_x_ + last_err_y_ * last_err_y_ + last_err_z_ * last_err_z_);
        const double rel_err = (gt_path_length_ > 1e-6)
            ? 100.0 * endpoint_err / gt_path_length_ : 0.0;

        char buf[1024];
        std::snprintf(
            buf, sizeof(buf),
            "\n========== CAPO vs Ground Truth 最终评估 ==========\n"
            "样本数:              %zu\n"
            "终点误差 x/y/z:      %.3f / %.3f / %.3f m\n"
            "终点 yaw 误差:       %.3f rad (%.1f°)\n"
            "终点位置误差(3D):    %.3f m\n"
            "累计行程 GT/CAPO:    %.2f / %.2f m\n"
            "相对位置误差:        %.2f %%（终点误差/GT 行程）\n"
            "位置 RMSE (2D):      %.3f m\n"
            "位置 RMSE (3D):      %.3f m（未对齐 ATE 近似）\n"
            "yaw RMSE:            %.3f rad\n"
            "CSV 数据:            %s\n"
            "===================================================",
            sample_count_, last_err_x_, last_err_y_, last_err_z_,
            last_err_yaw_, last_err_yaw_ * 180.0 / M_PI,
            endpoint_err, gt_path_length_, capo_path_length_, rel_err,
            rmse2d, rmse3d, rmse_yaw,
            csv_path_.empty() ? "未启用" : csv_path_.c_str());
        return std::string(buf);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CapoEvaluatorNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
