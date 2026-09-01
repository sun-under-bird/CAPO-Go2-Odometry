// ground_truth_node.cpp
// 仿真 Ground Truth 发布节点（阶段十六新增，仅用于 MuJoCo 仿真验证，不参与真机链路）。
//
// 数据来源：unitree_mujoco 仿真桥接发布的 rt/sportmodestate（ROS2 侧为 /sportmodestate，
// unitree_go/msg/SportModeState）。其中：
//   position[3]   <- MuJoCo frame_pos 传感器（base/imu site 的世界系位置）
//   velocity[3]   <- MuJoCo frame_vel 传感器（世界系线速度）
//   imu_state.quaternion[4] <- MuJoCo imu_quat（仿真无噪声，可作为真值姿态）
//   foot_force[4] <- 仿真桥接填充的足端法向接触力（CAPO 适配新增）
//
// 输出：/ground_truth/odom（nav_msgs/msg/Odometry），frame_id=world，child_frame_id=base_footprint，
// 供 evaluator 与 CAPO 的 /SMX/Odom_2D 做定量误差对比。

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

class GroundTruthNode : public rclcpp::Node
{
public:
    explicit GroundTruthNode(const rclcpp::NodeOptions & options)
    : Node("ground_truth_node", options)
    {
        // 话题与坐标系均可通过参数覆盖
        this->get_parameter_or("sub_sportstate_topic", sub_topic_, std::string("sportmodestate"));
        this->get_parameter_or("pub_odom_topic", pub_topic_, std::string("ground_truth/odom"));
        this->get_parameter_or("world_frame", world_frame_, std::string("world"));
        this->get_parameter_or("base_frame", base_frame_, std::string("base_footprint"));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_topic_, 10);

        // 仿真桥接以 best-effort 语义高速发布，订阅侧同样使用 best_effort 避免 QoS 不兼容
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
            sub_topic_, qos,
            std::bind(&GroundTruthNode::sportstate_callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(), "Ground Truth 节点已启动：%s -> %s",
            sub_topic_.c_str(), pub_topic_.c_str());
    }

private:
    std::string sub_topic_;
    std::string pub_topic_;
    std::string world_frame_;
    std::string base_frame_;

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    // 每收到一帧 sportmodestate 就转发一帧 Ground Truth 里程计
    void sportstate_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = world_frame_;
        odom.child_frame_id = base_frame_;

        // 世界系位置真值
        odom.pose.pose.position.x = msg->position[0];
        odom.pose.pose.position.y = msg->position[1];
        odom.pose.pose.position.z = msg->position[2];

        // 姿态真值：Unitree 四元数顺序为 w,x,y,z
        const auto & q = msg->imu_state.quaternion;
        const double norm = std::sqrt(
            q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (norm > 1e-6) {
            odom.pose.pose.orientation.w = q[0] / norm;
            odom.pose.pose.orientation.x = q[1] / norm;
            odom.pose.pose.orientation.y = q[2] / norm;
            odom.pose.pose.orientation.z = q[3] / norm;
        } else {
            odom.pose.pose.orientation.w = 1.0;
        }

        // 世界系线速度真值（注意：该速度在世界系下，非机体系 twist）
        odom.twist.twist.linear.x = msg->velocity[0];
        odom.twist.twist.linear.y = msg->velocity[1];
        odom.twist.twist.linear.z = msg->velocity[2];
        odom.twist.twist.angular.z = msg->yaw_speed;

        publisher_->publish(odom);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
