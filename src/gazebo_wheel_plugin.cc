#include "laser_rahcm_teleop/gazebo_wheel_plugin.hh"


using namespace gazebo;

Wheel::Wheel(WheelPlugin *_parent)
{
  this->plugin = _parent;
  this->currCmd = 0.0;
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastCmdTime = this->plugin->world->SimTime();
  #else
    this->lastCmdTime = this->plugin->world->GetSimTime();
  #endif
}

void Wheel::OnWheelCmd(const std_msgs::msg::Float32 & _msg)
{
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastCmdTime = this->plugin->world->SimTime();
  #else
    this->lastCmdTime = this->plugin->world->GetSimTime();
  #endif
  this->currCmd = _msg.data;
}

double WheelPlugin::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
    return _defaultVal;
  return _sdfPtr->Get<double>(_paramName);
}

double WheelPlugin::ScaleWheelCmd(const double _cmd,
                                 const double _max_cmd,
                                 const double _max_torque) const
{
  return (_cmd / _max_cmd) * _max_torque;
}

void WheelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("wheel_plugin"), "Loading wheel plugin...");
  this->model = _parent;
  this->world = this->model->GetWorld();

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);
  this->publisherRate = this->SdfParamDouble(_sdf, "publisherRate", 100.0);
  this->downwardForce = this->SdfParamDouble(_sdf, "downwardForce", 100.0);
  this->frictionMultiplier = this->SdfParamDouble(_sdf, "frictionMultiplier", 1.0);

  int wheelCounter = 0;
  if (_sdf->HasElement("wheel"))
  {
    sdf::ElementPtr wheelSDF = _sdf->GetElement("wheel");
    while (wheelSDF)
    {
      Wheel wheel(this);
      
      RCLCPP_INFO_STREAM(rclcpp::get_logger("wheel_plugin"), "Wheel " << wheelCounter);
      
      if (wheelSDF->HasElement("linkName"))
      {
        std::string linkName = wheelSDF->Get<std::string>("linkName");
        wheel.link = this->model->GetLink(linkName);
        if (!wheel.link)
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("wheel_plugin"), "Link not found: " << linkName);
        else
          RCLCPP_INFO_STREAM(rclcpp::get_logger("wheel_plugin"), "Wheel added to link: " << linkName);
      }
      else
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("wheel_plugin"), "No linkName specified for wheel " << wheelCounter);
      }

      if (wheelSDF->HasElement("wheelJointName"))
      {
        std::string jointName = wheelSDF->Get<std::string>("wheelJointName");
        wheel.wheelJoint = this->model->GetJoint(jointName);
        if (!wheel.wheelJoint)
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("wheel_plugin"), "Joint not found: " << jointName);
        else
          RCLCPP_INFO_STREAM(rclcpp::get_logger("wheel_plugin"), "Wheel joint: " << jointName);
      }
      else
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("wheel_plugin"), "No wheelJointName specified for wheel " << wheelCounter);
      }

      if (wheelSDF->HasElement("cmdTopic"))
      {
        wheel.cmdTopic = wheelSDF->Get<std::string>("cmdTopic");
      }
      else
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("wheel_plugin"), "No cmdTopic specified for wheel " << wheelCounter);
      }

      wheel.maxCmd = this->SdfParamDouble(wheelSDF, "maxCmd", 2.0);
      wheel.maxTorque = this->SdfParamDouble(wheelSDF, "maxTorque", 50.0);
      wheel.wheelRadius = this->SdfParamDouble(wheelSDF, "wheelRadius", 0.1);

      this->wheels.push_back(wheel);
      wheelSDF = wheelSDF->GetNextElement("wheel");
      wheelCounter++;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("wheel_plugin"), "No 'wheel' tags in description");
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("wheel_plugin"), "Found " << wheelCounter << " wheels");

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  this->node = std::make_shared<rclcpp::Node>("wheel_plugin"); 

  #if GAZEBO_MAJOR_VERSION >= 8
    this->prevUpdateTime = this->world->SimTime();
  #else
    this->prevUpdateTime = this->world->GetSimTime();
  #endif

  this->jointStatePub = this->node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  this->jointStateMsg.name.resize(wheels.size());
  this->jointStateMsg.position.resize(wheels.size());
  this->jointStateMsg.velocity.resize(wheels.size());
  this->jointStateMsg.effort.resize(wheels.size());

  for (size_t i = 0; i < this->wheels.size(); ++i)
  {
    this->jointStateMsg.name[i] = this->wheels[i].wheelJoint->GetName();
    this->wheels[i].cmdSub = this->node->create_subscription<std_msgs::msg::Float32>(
      this->wheels[i].cmdTopic, 1, std::bind(&Wheel::OnWheelCmd, &this->wheels[i], std::placeholders::_1));
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&WheelPlugin::Update, this));
}

void WheelPlugin::Update()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    common::Time now = this->world->SimTime();
  #else
    common::Time now = this->world->GetSimTime();
  #endif
  
  rclcpp::spin_some(this->node);

  for (size_t i = 0; i < this->wheels.size(); ++i)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      double dtc = (now - this->wheels[i].lastCmdTime).Double();
      if (dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
        this->wheels[i].currCmd = 0.0;

      double wheel_speed = this->wheels[i].wheelJoint->GetVelocity(0);
      
      double torque = this->ScaleWheelCmd(this->wheels[i].currCmd,
                                        this->wheels[i].maxCmd,
                                        this->wheels[i].maxTorque);
      
      // Modelo de atrito aprimorado
      double rolling_friction = -0.05 * wheel_speed * this->frictionMultiplier;
      double static_friction = (fabs(wheel_speed) < 0.01) ? 
                            -0.05 * torque * this->frictionMultiplier : 0.0;
      
      this->wheels[i].wheelJoint->SetForce(0, 
        torque + rolling_friction + static_friction);
      
      // Força descendente para melhor tração
      this->wheels[i].link->AddForce(ignition::math::Vector3d(0, 0, -this->downwardForce));

      #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Angle position = this->wheels[i].wheelJoint->Position(0);
      #else
        gazebo::math::Angle position = this->wheels[i].wheelJoint->GetAngle(0);
      #endif
      position.Normalize();
      this->jointStateMsg.position[i] = position.Radian();
      this->jointStateMsg.velocity[i] = wheel_speed;
      this->jointStateMsg.effort[i] = torque;
    }
  }

  if (now - this->prevUpdateTime >= (1 / this->publisherRate))
  {
    rclcpp::Clock clock(RCL_ROS_TIME);
    this->jointStateMsg.header.stamp = clock.now();
    this->jointStatePub->publish(this->jointStateMsg);
    this->prevUpdateTime = now;
  }
}

GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)