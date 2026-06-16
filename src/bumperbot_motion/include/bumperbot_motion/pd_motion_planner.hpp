#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace bumperbot_motion
{

class PDMotionPlanner : public rclcpp::Node
{
public:
    PDMotionPlanner();

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::TimerBase::SharedPtr control_loop_;

    double kp_;
    double kd_;
    double step_size_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    nav_msgs::msg::Path global_plan_;

    double prev_angular_error_, prev_linear_error_;
    rclcpp::Time last_cycle_time_;

    void controlLoop();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    bool transformPlan(const std::string & frame);
    geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);


};

}