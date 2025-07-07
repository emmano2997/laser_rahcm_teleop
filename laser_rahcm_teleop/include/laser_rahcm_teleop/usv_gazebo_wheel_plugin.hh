/*
 * Copyright (C) 2017  Brian Bingham
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef WHEEL_GAZEBO_PLUGINS_WHEEL_HH
#define WHEEL_GAZEBO_PLUGINS_WHEEL_HH

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
    public: virtual ~WheelPlugin() = default;
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    protected: virtual void Update();
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
    private: std::vector<Wheel> wheels;
    private: event::ConnectionPtr updateConnection;
    private: rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
    private: sensor_msgs::msg::JointState jointStateMsg;
    private: double publisherRate = 100.0;
    private: common::Time prevUpdateTime;
  };
}

#endif