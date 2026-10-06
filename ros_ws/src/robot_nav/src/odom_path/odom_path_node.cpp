#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace odom_path {

class OdomPathNode : public rclcpp::Node {
public:
    explicit OdomPathNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
    : Node("odom_path_node", opts)
    {
        declare_parameter("map_frame", "map");
        declare_parameter("max_poses", 2000);

        map_frame_ = get_parameter("map_frame").as_string();
        max_poses_ = static_cast<size_t>(get_parameter("max_poses").as_int());

        lio_path_.header.frame_id = map_frame_;
        ekf_path_.header.frame_id = map_frame_;

        lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/lio_odom", rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                append(lio_path_, *msg);
                lio_pub_->publish(lio_path_);
            });

        ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/lio_ekf_odom", rclcpp::QoS(10).reliable(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                append(ekf_path_, *msg);
                ekf_pub_->publish(ekf_path_);
            });

        lio_pub_ = create_publisher<nav_msgs::msg::Path>("/lio_odom_path", 10);
        ekf_pub_ = create_publisher<nav_msgs::msg::Path>("/lio_ekf_path",  10);
    }

private:
    void append(nav_msgs::msg::Path & path, const nav_msgs::msg::Odometry & odom)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header          = odom.header;
        ps.header.frame_id = map_frame_;
        ps.pose            = odom.pose.pose;
        path.header.stamp  = odom.header.stamp;
        path.poses.push_back(ps);
        if (path.poses.size() > max_poses_) {
            path.poses.erase(path.poses.begin());
        }
    }

    std::string map_frame_;
    size_t      max_poses_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr        lio_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr        ekf_pub_;

    nav_msgs::msg::Path lio_path_;
    nav_msgs::msg::Path ekf_path_;
};

} // namespace odom_path

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odom_path::OdomPathNode>());
    rclcpp::shutdown();
    return 0;
}
