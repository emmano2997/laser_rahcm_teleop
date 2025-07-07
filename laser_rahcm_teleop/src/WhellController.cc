#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

class WheelController : public rclcpp::Node {
public:
  WheelController() : Node("wheel_controller") {
    // Parameters
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 3);
    this->declare_parameter<double>("scale_linear", 1.0);
    this->declare_parameter<double>("scale_angular", 1.0);

    axis_linear_ = this->get_parameter("axis_linear").as_int();
    axis_angular_ = this->get_parameter("axis_angular").as_int();
    scale_linear_ = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();

    // Subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&WheelController::joy_callback, this, std::placeholders::_1));

    sub_mode_ = this->create_subscription<std_msgs::msg::Bool>(
      "mode_vehicle", 10, std::bind(&WheelController::mode_callback, this, std::placeholders::_1));

    // Publishers
    left_front_pub_ = this->create_publisher<std_msgs::msg::Float32>("/wheel/left_front_wheel_cmd", 10);
    left_rear_pub_ = this->create_publisher<std_msgs::msg::Float32>("/wheel/left_rear_wheel_cmd", 10);
    right_front_pub_ = this->create_publisher<std_msgs::msg::Float32>("/wheel/right_front_wheel_cmd", 10);
    right_rear_pub_ = this->create_publisher<std_msgs::msg::Float32>("/wheel/right_rear_wheel_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Wheel controller node started");
  }

private:
  void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    ground_mode_ = msg->data;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!ground_mode_) return;  

    float linear = msg->axes[axis_linear_] * scale_linear_;
    float angular = msg->axes[axis_angular_] * scale_angular_;
    
    // Create messages
    auto left_front = std_msgs::msg::Float32();
    auto left_rear = std_msgs::msg::Float32();
    auto right_front = std_msgs::msg::Float32();
    auto right_rear = std_msgs::msg::Float32();

    // Calculate wheel commands
    left_front.data = linear - angular;
    left_rear.data = linear - angular;
    right_front.data = linear + angular;
    right_rear.data = linear + angular;

    // Publish commands
    left_front_pub_->publish(left_front);
    left_rear_pub_->publish(left_rear);
    right_front_pub_->publish(right_front);
    right_rear_pub_->publish(right_rear);
  }

  // Member variables
  bool ground_mode_ = false;
  int axis_linear_;
  int axis_angular_;
  double scale_linear_;
  double scale_angular_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mode_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_rear_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_rear_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}