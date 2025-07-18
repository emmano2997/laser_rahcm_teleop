#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

class ThrusterController : public rclcpp::Node
{
public:
  ThrusterController() : Node("ThrusterController"), thruster_enabled_(true)
  {
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 3);
    this->declare_parameter<double>("scale_linear", 1.0);
    this->declare_parameter<double>("scale_angular", 1.0);

    axis_linear_ = this->get_parameter("axis_linear").as_int();
    axis_angular_ = this->get_parameter("axis_angular").as_int();
    scale_linear_ = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ThrusterController::joy_callback, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/mode_vehicle", 10, std::bind(&ThrusterController::mode_callback, this, std::placeholders::_1));

    left_front_pub_ = this->create_publisher<std_msgs::msg::Float32>("/thruster/left_front_wheel_cmd", 10);
    left_rear_pub_ = this->create_publisher<std_msgs::msg::Float32>("/thruster/left_rear_wheel_cmd", 10);
    right_front_pub_ = this->create_publisher<std_msgs::msg::Float32>("/thruster/right_front_wheel_cmd", 10);
    right_rear_pub_ = this->create_publisher<std_msgs::msg::Float32>("/thruster/right_rear_wheel_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "ThrusterController iniciado. Aguardando comandos...");
  }

private:
  void mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    thruster_enabled_ = !msg->data;  // true (terra) → desliga thrusters
    RCLCPP_INFO(this->get_logger(), "Thruster %s",
      thruster_enabled_ ? "ATIVADO (modo AQUÁTICO)" : "DESATIVADO (modo TERRA)");
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!thruster_enabled_) {
      return; 
    }

    float linear = msg->axes[axis_linear_] * scale_linear_;
    float angular = msg->axes[axis_angular_] * scale_angular_;

    auto left_front = std_msgs::msg::Float32();
    auto left_rear = std_msgs::msg::Float32();
    auto right_front = std_msgs::msg::Float32();
    auto right_rear = std_msgs::msg::Float32();

    left_front.data = linear - angular;
    left_rear.data = linear - angular;
    right_front.data = linear + angular;
    right_rear.data = linear + angular;

    left_front_pub_->publish(left_front);
    left_rear_pub_->publish(left_rear);
    right_front_pub_->publish(right_front);
    right_rear_pub_->publish(right_rear);

    // Botão B = emergência
    if (msg->buttons[1] == 1) {
      RCLCPP_WARN(this->get_logger(), "Parada de emergência ativada!");

      auto stop_msg = std_msgs::msg::Float32();
      stop_msg.data = 0.0;

      left_front_pub_->publish(stop_msg);
      left_rear_pub_->publish(stop_msg);
      right_front_pub_->publish(stop_msg);
      right_rear_pub_->publish(stop_msg);
      return;
    }
  }

  int axis_linear_, axis_angular_;
  double scale_linear_, scale_angular_;
  bool thruster_enabled_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_rear_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_rear_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterController>());
  rclcpp::shutdown();
  return 0;
}
