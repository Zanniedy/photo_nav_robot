#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

namespace sim_imu_bridge {

class SimImuBridgeNode : public rclcpp::Node {
public:
    explicit SimImuBridgeNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{})
        : Node("sim_imu_bridge_node", opts)
    {
        yaw_pub_       = create_publisher<std_msgs::msg::Float32>("/robot/yaw",       10);
        yaw_speed_pub_ = create_publisher<std_msgs::msg::Float32>("/robot/yaw_speed", 10);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::QoS(10).best_effort(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                double now = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

                // angular_velocity.z: rad/s, 顺时针为正与硬件一致（Ignition坐标系z向上）
                double wz_rads = msg->angular_velocity.z;
                float  wz_degs = static_cast<float>(wz_rads * 180.0 / M_PI);

                if (last_stamp_ > 0.0) {
                    double dt = now - last_stamp_;
                    if (dt > 0.0 && dt < 0.5) {
                        yaw_ += wz_rads * dt;
                    }
                }
                last_stamp_ = now;

                std_msgs::msg::Float32 yaw_msg, speed_msg;
                yaw_msg.data   = static_cast<float>(yaw_ * 180.0 / M_PI);
                speed_msg.data = wz_degs;

                yaw_pub_->publish(yaw_msg);
                yaw_speed_pub_->publish(speed_msg);
            });
    }

private:
    double last_stamp_ = -1.0;
    double yaw_        = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     yaw_speed_pub_;
};

} // namespace sim_imu_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_imu_bridge::SimImuBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
