#include "rclcpp/rclcpp.hpp"              
#include "sensor_msgs/msg/joy.hpp"        
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"   
#include "std_msgs/msg/float64.hpp"             
#include "std_msgs/msg/float32.hpp"             
#include "std_msgs/msg/int32.hpp"                
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  
#include <string>                                

class RcTeleopNode : public rclcpp::Node
{
public:
  RcTeleopNode() : Node("rc_teleop")
  {
    // Declarar e obter parâmetro para nome do tópico de entrada
    this->declare_parameter<std::string>("joy_topic", "joy_raw");
    std::string joy_topic = this->get_parameter("joy_topic").as_string();

    // Assinatura no tópico do joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, 10,
      std::bind(&RcTeleopNode::joy_callback, this, std::placeholders::_1)
    );

    // Publicador do tópico de saída
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

    RCLCPP_INFO(this->get_logger(), "rc_teleop node started. Subscribing to '%s'", joy_topic.c_str());
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joy_pub_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcTeleopNode>());
  rclcpp::shutdown();
  return 0;
}