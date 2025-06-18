#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WheelController : public rclcpp::Node
{
public:
  WheelController() : Node("wheel_controller")
  {
    // Parâmetros configuráveis
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 3);
    this->declare_parameter<int>("enable_button", 4); // Botão LB
    this->declare_parameter<double>("scale_linear", 0.5);
    this->declare_parameter<double>("scale_angular", 1.0);

    axis_linear_ = this->get_parameter("axis_linear").as_int();
    axis_angular_ = this->get_parameter("axis_angular").as_int();
    enable_button_ = this->get_parameter("enable_button").as_int();
    scale_linear_ = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&WheelController::joy_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_controller/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Wheel controller node started");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto cmd_vel = geometry_msgs::msg::Twist();

    if (msg->buttons[enable_button_] == 1) {
      cmd_vel.linear.x = msg->axes[1] * scale_linear_;     // Frente/Trás
      cmd_vel.angular.z = msg->axes[3] * scale_angular_;   // Giro em Z (horário / anti)
    } else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }
     // Freio de emergência (botão B - índice 1)
    if (msg->buttons[1] == 1) {
      RCLCPP_WARN(this->get_logger(), "Freio de emergência acionado!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel);
      return;  
    }

    RCLCPP_DEBUG(this->get_logger(), "Linear: %.2f, Angular: %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
  }

  int axis_linear_, axis_angular_, enable_button_;
  double scale_linear_, scale_angular_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}