#ifndef GAZEBO_WHEEL_PLUGIN_HH
#define GAZEBO_WHEEL_PLUGIN_HH

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class WheelPlugin;

  class Wheel
  {
    public: explicit Wheel(WheelPlugin *_parent);
    public: void OnWheelCmd(const std_msgs::msg::Float32 & _msg);

    public: double maxCmd;
    public: double maxTorque;
    public: double wheelRadius;
    public: physics::LinkPtr link;
    public: std::string cmdTopic;
    public: rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmdSub;
    public: double currCmd;
    public: common::Time lastCmdTime;
    public: physics::JointPtr wheelJoint;
    protected: WheelPlugin *plugin;
  };

  class WheelPlugin : public ModelPlugin
  {
    public: WheelPlugin() = default;
    public: ~WheelPlugin() override = default;
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    protected: void Update();
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                  const std::string &_paramName,
                                  const double _defaultVal) const;
    private: double ScaleWheelCmd(const double _cmd,
                                 const double _max_cmd,
                                 const double _max_torque) const;
    public: std::mutex mutex;
    private: rclcpp::Node::SharedPtr node;
    public: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: double cmdTimeout;
    private: double downwardForce;
    private: double frictionMultiplier;
    private: std::vector<Wheel> wheels;
    private: event::ConnectionPtr updateConnection;
    private: rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
    private: sensor_msgs::msg::JointState jointStateMsg;
    private: double publisherRate = 100.0;
    private: common::Time prevUpdateTime;
  };
} // namespace gazebo

#endif