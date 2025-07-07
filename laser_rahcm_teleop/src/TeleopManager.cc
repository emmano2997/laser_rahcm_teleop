#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

class TeleopManager : public rclcpp::Node {
public:
  TeleopManager() : Node("TeleopManager") {
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&TeleopManager::joy_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Bool>("/mode_vehicle", 10);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    static bool last = false;
    if (msg->buttons[3] == 1 && !last) {
      Ground_mode_ = !Ground_mode_;
      std_msgs::msg::Bool mode_msg;
      mode_msg.data = Ground_mode_;
      pub_->publish(mode_msg);
      RCLCPP_INFO(this->get_logger(), "Mode changed: %s", Ground_mode_ ? "GROUND" : "AQUATIC");
    }
    last = msg->buttons[3]; // Y Button
  }

  bool Ground_mode_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopManager>());
  rclcpp::shutdown();
  return 0;
}