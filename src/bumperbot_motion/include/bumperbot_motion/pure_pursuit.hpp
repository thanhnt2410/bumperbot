#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace bumperbot_motion
{

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::TimerBase::SharedPtr control_loop_;

    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    nav_msgs::msg::Path global_plan_;

    void controlLoop();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    bool transformPlan(const std::string & frame);
    geometry_msgs::msg::PoseStamped getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose);
    
    double getCurvature(const geometry_msgs::msg::Pose & carrot_pose);


};

}