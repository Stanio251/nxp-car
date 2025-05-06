#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AnglePIDNode : public rclcpp::Node {
public:
  AnglePIDNode() : Node("angle_pid_node") {
    RCLCPP_INFO(this->get_logger(), "Angle PID Node started.");

    // Subscriber to /angle_error_value
    error_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/angle_error_value", 10,
      std::bind(&AnglePIDNode::error_callback, this, std::placeholders::_1)
    );

    // Publisher to /cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void error_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    int error = msg->data;
    float Kp = 0.01;  // Proportional gain — tune this!

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.1;  // Constant forward speed
    cmd_msg.angular.z = -Kp * error;  // Correcting rotation

    RCLCPP_INFO(this->get_logger(), "Error: %d -> angular.z: %.2f", error, cmd_msg.angular.z);
    cmd_pub_->publish(cmd_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr error_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnglePIDNode>());
    rclcpp::shutdown();
    return 0;
  }
  