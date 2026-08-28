#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <unitree_go/msg/low_state.hpp>

class Go2LowStateAdapterNode : public rclcpp::Node
{
public:
    // 初始化 LowState 订阅、CAPO 输入发布者和固定频率转发定时器。
    explicit Go2LowStateAdapterNode(const rclcpp::NodeOptions &options)
    : Node("go2_lowstate_adapter_node", options)
    {
        this->get_parameter_or("sub_lowstate_topic", sub_lowstate_topic_, std::string("/lowstate"));
        this->get_parameter_or("pub_imu_topic", pub_imu_topic_, std::string("SMX/Go2IMU"));
        this->get_parameter_or("pub_joint_topic", pub_joint_topic_, std::string("SMX/Go2Joint"));
        this->get_parameter_or("imu_frame", imu_frame_, std::string("base_imu"));
        this->get_parameter_or("publish_rate_hz", publish_rate_hz_, 200.0);
        this->get_parameter_or("use_estimated_foot_force", use_estimated_foot_force_, false);

        if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
            throw std::invalid_argument("publish_rate_hz 必须是正数");
        }

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_imu_topic_, 10);
        joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_joint_topic_, 10);

        const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
            sub_lowstate_topic_, lowstate_qos,
            std::bind(&Go2LowStateAdapterNode::lowstate_callback, this, std::placeholders::_1));

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_));
        publish_timer_ = this->create_wall_timer(
            period, std::bind(&Go2LowStateAdapterNode::publish_latest, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Go2 适配节点已启动：%s -> %s / %s，发布频率 %.1f Hz",
            sub_lowstate_topic_.c_str(), pub_imu_topic_.c_str(), pub_joint_topic_.c_str(),
            publish_rate_hz_);
    }

private:
    // CAPO 腿序为 FL、FR、RL、RR；Unitree 电机序为 FR、FL、RR、RL。
    static constexpr std::array<int, 4> kUnitreeMotorBaseByCapoLeg = {3, 0, 9, 6};
    static constexpr std::array<int, 4> kUnitreeFootByCapoLeg = {1, 0, 3, 2};

    std::string sub_lowstate_topic_;
    std::string pub_imu_topic_;
    std::string pub_joint_topic_;
    std::string imu_frame_;
    double publish_rate_hz_ = 200.0;
    bool use_estimated_foot_force_ = false;
    int missing_lowstate_cycles_ = 0;

    unitree_go::msg::LowState::SharedPtr latest_lowstate_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // 缓存最新 LowState，由 200 Hz 定时器统一发布，避免直接转发 500 Hz 原始数据。
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        latest_lowstate_ = msg;
        missing_lowstate_cycles_ = 0;
    }

    // 将缓存的 LowState 同步转换为 IMU 和 48 元素关节数组并发布。
    void publish_latest()
    {
        if (!latest_lowstate_) {
            ++missing_lowstate_cycles_;
            if (missing_lowstate_cycles_ >= static_cast<int>(publish_rate_hz_ * 2.0)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "尚未收到 %s，请检查 Go2 DDS 和 ROS_DOMAIN_ID", sub_lowstate_topic_.c_str());
            }
            return;
        }

        const auto stamp = this->get_clock()->now();
        const auto &lowstate = *latest_lowstate_;

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = imu_frame_;

        // Unitree 四元数顺序为 w、x、y、z，ROS 消息字段顺序需显式映射。
        const auto &unitree_quaternion = lowstate.imu_state.quaternion;
        const double quaternion_norm = std::sqrt(
            unitree_quaternion[0] * unitree_quaternion[0] +
            unitree_quaternion[1] * unitree_quaternion[1] +
            unitree_quaternion[2] * unitree_quaternion[2] +
            unitree_quaternion[3] * unitree_quaternion[3]);
        if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-6) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "LowState IMU 四元数无效，本周期不发布 CAPO 输入");
            return;
        }

        imu_msg.orientation.w = unitree_quaternion[0] / quaternion_norm;
        imu_msg.orientation.x = unitree_quaternion[1] / quaternion_norm;
        imu_msg.orientation.y = unitree_quaternion[2] / quaternion_norm;
        imu_msg.orientation.z = unitree_quaternion[3] / quaternion_norm;
        imu_msg.angular_velocity.x = lowstate.imu_state.gyroscope[0];
        imu_msg.angular_velocity.y = lowstate.imu_state.gyroscope[1];
        imu_msg.angular_velocity.z = lowstate.imu_state.gyroscope[2];
        imu_msg.linear_acceleration.x = lowstate.imu_state.accelerometer[0];
        imu_msg.linear_acceleration.y = lowstate.imu_state.accelerometer[1];
        imu_msg.linear_acceleration.z = lowstate.imu_state.accelerometer[2];

        std_msgs::msg::Float64MultiArray joint_msg;
        joint_msg.data.assign(48, 0.0);

        for (int capo_leg = 0; capo_leg < 4; ++capo_leg) {
            const int source_motor_base = kUnitreeMotorBaseByCapoLeg[capo_leg];
            const int destination_motor_base = capo_leg * 4;

            for (int joint = 0; joint < 3; ++joint) {
                const auto &motor = lowstate.motor_state[source_motor_base + joint];
                const int destination = destination_motor_base + joint;
                joint_msg.data[destination] = motor.q;
                joint_msg.data[16 + destination] = motor.dq;
            }

            // CAPO 的接触传感器通道位于每条腿 calf 的 tau 槽，其余 tau/wheel 槽保持为零。
            const int source_foot = kUnitreeFootByCapoLeg[capo_leg];
            const double foot_force = use_estimated_foot_force_
                ? static_cast<double>(lowstate.foot_force_est[source_foot])
                : static_cast<double>(lowstate.foot_force[source_foot]);
            joint_msg.data[32 + destination_motor_base + 2] = foot_force;
        }

        imu_publisher_->publish(imu_msg);
        joint_publisher_->publish(joint_msg);
    }
};

// 启动适配节点，并默认加载当前 Go2 上的 CAPO 参数文件。
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
        .arguments({
            "--ros-args",
            "--params-file", "/root/CAPO-LeggedRobotOdometry/config.yaml"
        });

    auto node = std::make_shared<Go2LowStateAdapterNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
