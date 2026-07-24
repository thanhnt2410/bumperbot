#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

class TwistRelay : public rclcpp::Node
{
public:
    TwistRelay() : Node("twist_relay")
    {
        declare_parameter<double>("safety_slowdown_factor", 0.35);
        safety_slowdown_factor_ =
            get_parameter("safety_slowdown_factor").as_double();
        controller_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "bumperbot_controller/cmd_vel_unstamped", 10,
            std::bind(&TwistRelay::controller_twist_callback, this, std::placeholders::_1)
        );
        controller_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "bumperbot_controller/cmd_vel", 10
        );
        safety_slowdown_sub_ = create_subscription<std_msgs::msg::Bool>(
            "safety_slowdown", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                safety_slowdown_active_ = msg->data;
            }
        );
        joy_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/input_joy/cmd_vel_stamped", 10,
            std::bind(&TwistRelay::joy_twist_callback, this, std::placeholders::_1)
        );
        joy_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/input_joy/cmd_vel", 10
        );
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_slowdown_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
    double safety_slowdown_factor_;
    bool safety_slowdown_active_{false};


    void controller_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped twist_stamped;
        twist_stamped.header.stamp = get_clock()->now();
        twist_stamped.twist = *msg;
        if (safety_slowdown_active_) {
            twist_stamped.twist.linear.x *= safety_slowdown_factor_;
            twist_stamped.twist.angular.z *= safety_slowdown_factor_;
        }
        controller_pub_->publish(twist_stamped);
    }
    void joy_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;
        twist = msg->twist;
        joy_pub_->publish(twist);
    }
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
