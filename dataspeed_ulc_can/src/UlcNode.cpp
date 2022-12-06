/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "UlcNode.h"
#include <dataspeed_ulc_can/dispatch.h>

namespace dataspeed_ulc_can
{

template <typename T>
static void getParamWithSaturation(ros::NodeHandle &nh, const std::string& key, T& value, T min, T max)
{
  if (nh.getParam(key, value)) {
    if (value < min) {
      value = min;
      nh.setParam(key, value);
    } else if (value > max) {
      value = max;
      nh.setParam(key, value);
    }
  }
}

template <class T>
static T overflowSaturation(double input, T limit_min, T limit_max, double scale_factor, const std::string& input_name, const std::string& units)
{
  if (input < (limit_min * scale_factor)) {
    ROS_WARN("%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input, units.c_str(), limit_min * scale_factor, units.c_str());
    return limit_min;
  } else if (input > (limit_max * scale_factor)) {
    ROS_WARN("%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input, units.c_str(), limit_max * scale_factor, units.c_str());
    return limit_max;
  } else {
    return input / scale_factor;
  }
}

static inline bool validInputs(const dataspeed_ulc_msgs::UlcCmd& cmd)
{
  bool valid = true;
  if (std::isnan(cmd.linear_velocity) && cmd.pedals_mode == dataspeed_ulc_msgs::UlcCmd::SPEED_MODE) {
    ROS_WARN("NaN input detected on speed input");
    valid = false;
  }
  if (std::isnan(cmd.accel_cmd) && cmd.pedals_mode == dataspeed_ulc_msgs::UlcCmd::ACCEL_MODE) {
    ROS_WARN("NaN input detected on speed input");
    valid = false;
  }
    if (cmd.pedals_mode != dataspeed_ulc_msgs::UlcCmd::SPEED_MODE && cmd.pedals_mode != dataspeed_ulc_msgs::UlcCmd::ACCEL_MODE) {
    ROS_WARN("Invalid pedals mode in command message");
    valid = false;
  }
  if (std::isnan(cmd.yaw_command)) {
    ROS_WARN("NaN input detected on yaw command input");
    valid = false;
  }
  if (std::isnan(cmd.linear_accel)) {
    ROS_WARN("NaN input detected on linear accel input");
    valid = false;
  }
  if (std::isnan(cmd.linear_decel)) {
    ROS_WARN("NaN input detected on linear decel input");
    valid = false;
  }
  if (std::isnan(cmd.lateral_accel)) {
    ROS_WARN("NaN input detected on lateral accel input");
    valid = false;
  }
  if (std::isnan(cmd.angular_accel)) {
    ROS_WARN("NaN input detected on angular accel input");
    valid = false;
  }
  if (std::isnan(cmd.jerk_limit_throttle)) {
    ROS_WARN("NaN input detected on throttle jerk input");
    valid = false;
  }
  if (std::isnan(cmd.jerk_limit_brake)) {
    ROS_WARN("NaN input detected on brake jerk input");
    valid = false;
  }
  return valid;
}

// Last firmware versions before new ULC interface.
PlatformMap OLD_ULC_FIRMWARE({
  {PlatformVersion(P_FCA_RU,      M_STEER, ModuleVersion(1,5,2))},
  {PlatformVersion(P_FCA_WK2,     M_STEER, ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_C1,     M_STEER, ModuleVersion(1,2,2))},
  {PlatformVersion(P_FORD_CD4,    M_STEER, ModuleVersion(2,5,2))},
  {PlatformVersion(P_FORD_CD5,    M_STEER, ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_GE1,    M_STEER, ModuleVersion(0,1,0))},
  {PlatformVersion(P_FORD_P5,     M_STEER, ModuleVersion(1,4,2))},
  {PlatformVersion(P_FORD_T6,     M_STEER, ModuleVersion(0,2,2))},
  {PlatformVersion(P_FORD_U6,     M_STEER, ModuleVersion(1,0,2))},
  {PlatformVersion(P_POLARIS_GEM, M_STEER, ModuleVersion(1,1,1))},
  {PlatformVersion(P_POLARIS_RZR, M_STEER, ModuleVersion(0,3,1))},
});

UlcNode::UlcNode(ros::NodeHandle &n, ros::NodeHandle &pn) :
  enable_(false), accel_mode_supported_(true)
{
  // Setup publishers
  pub_report_ = n.advertise<dataspeed_ulc_msgs::UlcReport>("ulc_report", 2);
  pub_can_ = n.advertise<can_msgs::Frame>("can_tx", 100);

  // Setup CARMA publishers
  pub_robot_status_ = n.advertise<cav_msgs::RobotEnabled>("robot_status", 10);
  pub_discovery_ = n.advertise<cav_msgs::DriverStatus>("discovery", 10);

  // Setup subscribers
  const ros::TransportHints NODELAY = ros::TransportHints().tcpNoDelay();
  sub_can_ = n.subscribe<can_msgs::Frame>("can_rx", 100, &UlcNode::recvCan, this, NODELAY);
  sub_cmd_ = n.subscribe<dataspeed_ulc_msgs::UlcCmd>("ulc_cmd", 2, &UlcNode::recvUlcCmd, this, NODELAY);
  sub_twist_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 2, &UlcNode::recvTwist, this, NODELAY);
  sub_twist_stamped_ = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel_stamped", 2, &UlcNode::recvTwistStamped, this, NODELAY);
  sub_enable_ = n.subscribe<std_msgs::Bool>("dbw_enabled", 2, &UlcNode::recvEnable, this, NODELAY);

  // Setup CARMA subscribers
  sub_vehicle_cmd_ = n.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 2, &UlcNode::recvVehicleCmd, this, NODELAY);

  // Setup timer for config message retransmission
  double freq = 5.0;
  getParamWithSaturation(pn, "config_frequency", freq, 5.0, 50.0);
  config_timer_ = n.createTimer(ros::Duration(1.0 / freq), &UlcNode::configTimerCb, this);

  // Setup timer for driver discovery
  last_discovery_pub_ = ros::Time::now();
  last_status_pub_ = ros::Time::now();

  // Setup driver discovery message
  driver_status_.name = "/hardware_interface/controller";
  driver_status_.status = cav_msgs::DriverStatus::OPERATIONAL;
  driver_status_.controller = true;
}

void UlcNode::recvEnable(const std_msgs::BoolConstPtr& msg)
{
  enable_ = msg->data;
}

void UlcNode::recvUlcCmd(const dataspeed_ulc_msgs::UlcCmdConstPtr& msg)
{
  // Check for differences in acceleration and jerk limits
  bool diff = (msg->linear_accel  != ulc_cmd_.linear_accel)
           || (msg->linear_decel  != ulc_cmd_.linear_decel)
           || (msg->lateral_accel != ulc_cmd_.lateral_accel)
           || (msg->angular_accel != ulc_cmd_.angular_accel)
           || (msg->jerk_limit_throttle != ulc_cmd_.jerk_limit_throttle)
           || (msg->jerk_limit_brake != ulc_cmd_.jerk_limit_brake);
  ulc_cmd_ = *msg;

  // Publish command message
  sendCmdMsg(true);

  // Publish config message on change
  if (diff) {
    sendCfgMsg();
  }
}

void UlcNode::recvTwistCmd(const geometry_msgs::Twist& msg)
{
  // Populate command fields
  ulc_cmd_.linear_velocity = msg.linear.x;
  ulc_cmd_.pedals_mode = dataspeed_ulc_msgs::UlcCmd::SPEED_MODE;
  ulc_cmd_.coast_decel = false;
  ulc_cmd_.yaw_command = msg.angular.z;
  ulc_cmd_.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;

  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_pedals = true;
  ulc_cmd_.enable_shifting = true;
  ulc_cmd_.enable_steering = true;
  ulc_cmd_.shift_from_park = true;
  ulc_cmd_.linear_accel = 0;
  ulc_cmd_.linear_decel = 0;
  ulc_cmd_.angular_accel = 0;
  ulc_cmd_.lateral_accel = 0;
  ulc_cmd_.jerk_limit_throttle = 0;
  ulc_cmd_.jerk_limit_brake = 0;

  // Publish command message
  sendCmdMsg(false);
}

void UlcNode::recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  recvTwistCmd(*msg);
}

void UlcNode::recvTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg)
{
  recvTwistCmd(msg->twist);
}

void UlcNode::recvAutowareTwistStamped(const geometry_msgs::TwistStamped& msg)
{
  recvTwistCmd(msg.twist);
}

void UlcNode::recvControlCmd(const autoware_msgs::ControlCommand& msg)
{
  double acker_wheelbase_ = 2.8498; // 112.2 inches
  double steering_ratio_ = 14.8;
  
  // Populate command fields
  ulc_cmd_.linear_velocity = msg.linear_velocity;
  ulc_cmd_.yaw_command = current_speed_ * tan(msg.steering_angle / steering_ratio_) / acker_wheelbase_;
  ulc_cmd_.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;

  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_pedals = true;
  ulc_cmd_.enable_shifting = true;
  ulc_cmd_.enable_steering = true;
  ulc_cmd_.shift_from_park = true;
  ulc_cmd_.linear_accel = 0;
  ulc_cmd_.linear_decel = 0;
  ulc_cmd_.angular_accel = 0;
  ulc_cmd_.lateral_accel = 0;

  // Publish command message
  sendCmdMsg(false);
}

void UlcNode::recvVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  //recvControlCmd(msg->ctrl_cmd);
  recvAutowareTwistStamped(msg->twist_cmd);
}

void UlcNode::recvCan(const can_msgs::FrameConstPtr& msg)
{
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_ULC_REPORT:
        if (msg->dlc >= sizeof(MsgUlcReport)) {
          const MsgUlcReport *ptr = (const MsgUlcReport *)msg->data.elems;
          dataspeed_ulc_msgs::UlcReport report;
          report.header.stamp = msg->header.stamp;
          report.speed_ref = (float)ptr->speed_ref * 0.02f;
          report.timeout = ptr->timeout;
          report.pedals_enabled = ptr->pedals_enabled;
          report.pedals_mode = ptr->pedals_mode;
          report.speed_meas = (float)ptr->speed_meas * 0.02f;
          report.override_latched = ptr->override;
          report.steering_enabled = ptr->steering_enabled;
          report.steering_mode = ptr->steering_mode;
          report.accel_ref = (float)ptr->accel_ref * 0.05f;
          current_speed_ = report.speed_meas;
          report.accel_meas = (float)ptr->accel_meas * 0.05f;
          report.max_steering_angle = (float)ptr->max_steering_angle * 5.0f;
          report.coasting = ptr->coasting;
          report.max_steering_vel = (float)ptr->max_steering_vel * 8.0f;
          report.steering_preempted = ptr->steering_preempted;
          report.speed_preempted = ptr->speed_preempted;
          pub_report_.publish(report);
          if (report.pedals_enabled && report.steering_enabled && !report.override_latched) {
            active_ = true;
          } else {
            active_ = false;
          }
        }
        break;
      case ID_VERSION:
        if (msg->dlc >= sizeof(MsgVersion)) {
          const MsgVersion *ptr = (const MsgVersion*)msg->data.elems;
          if ((Module)ptr->module == M_STEER) {
            const PlatformVersion version((Platform)ptr->platform, (Module)ptr->module, ptr->major, ptr->minor, ptr->build);
            const ModuleVersion old_ulc_version = OLD_ULC_FIRMWARE.findModule(version);
            const char * str_p = platformToString(version.p);
            const char * str_m = moduleToString(version.m);
            if (firmware_.findModule(version) != version.v) {
              firmware_.insert(version);
              if (version <= old_ulc_version) {
                accel_mode_supported_ = false;
                ROS_WARN("Firmware %s %s  version %u.%u.%u does not support ULC acceleration interface mode.", str_p, str_m,
                        version.v.major(), version.v.minor(), version.v.build());
              }
            }
          }
        }
        break;
    }
  }

  // Publish RobotEnabled
  robot_enabled_.robot_active = active_;
  robot_enabled_.robot_enabled = enable_;
  if (last_status_pub_ == ros::Time(0) || (ros::Time::now() - last_status_pub_).toSec() > 0.098) {
    pub_robot_status_.publish(robot_enabled_);
    last_status_pub_ = ros::Time::now();
  }

  // Publish DriverStatus
  if (last_discovery_pub_ == ros::Time(0) || (ros::Time::now() - last_discovery_pub_).toSec() > 0.8) {
    pub_discovery_.publish(driver_status_);
    last_discovery_pub_ = ros::Time::now();
  }
}

void UlcNode::sendCmdMsg(bool cfg)
{
  // Validate input fields
  if (validInputs(ulc_cmd_)) {
    if (cfg) {
      cmd_stamp_ = ros::Time::now();
    }
  } else {
    cmd_stamp_ = ros::Time(0);
    driver_status_.status = cav_msgs::DriverStatus::FAULT;
    return;
  }

  // Build CAN message
  can_msgs::Frame msg;
  msg.id = ID_ULC_CMD;
  msg.is_extended = false;
  msg.dlc = sizeof(MsgUlcCmd);
  MsgUlcCmd *ptr = (MsgUlcCmd *)msg.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));

  // Populate enable bits
  if (enable_) {
    ptr->enable_pedals = ulc_cmd_.enable_pedals;
    ptr->enable_steering = ulc_cmd_.enable_steering;
    ptr->enable_shifting = ulc_cmd_.enable_shifting;
    ptr->shift_from_park = ulc_cmd_.shift_from_park;
  }

  // Populate command fields
  ptr->clear = ulc_cmd_.clear;
  ptr->pedals_mode = ulc_cmd_.pedals_mode;
  ptr->coast_decel = ulc_cmd_.coast_decel;
  ptr->steering_mode = ulc_cmd_.steering_mode;
  if (ulc_cmd_.pedals_mode == dataspeed_ulc_msgs::UlcCmd::SPEED_MODE) {
    ptr->lon_command =
        overflowSaturation(ulc_cmd_.linear_velocity, INT16_MIN, INT16_MAX, 0.0025, "ULC command speed", "m/s");
  } else if (ulc_cmd_.pedals_mode == dataspeed_ulc_msgs::UlcCmd::ACCEL_MODE) {
    if (!accel_mode_supported_) {
      ROS_WARN_THROTTLE(1.0, "Firmware does not support acceleration mode interface");
      return;
    }
    ptr->lon_command =
        overflowSaturation(ulc_cmd_.accel_cmd, INT16_MIN, INT16_MAX, 5e-4, "ULC command accel", "m/s^2");
  } else {
    ptr->lon_command = 0;
    ROS_WARN_THROTTLE(1.0, "Unsupported ULC pedal control mode [%d]", ulc_cmd_.pedals_mode);
    cmd_stamp_ = ros::Time(0);
    return;
  }
  if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE) {
    ptr->yaw_command = overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.00025, "ULC yaw rate command", "rad/s");
  } else if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE) {
    ptr->yaw_command = overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.0000061, "ULC curvature command", "1/m");
  } else {
    ptr->yaw_command = 0;
    ROS_WARN_THROTTLE(1.0, "Unsupported ULC steering control mode [%d]", ulc_cmd_.steering_mode);
    driver_status_.status = cav_msgs::DriverStatus::FAULT;
    cmd_stamp_ = ros::Time(0);
    return;
  }

  // Publish message
  pub_can_.publish(msg);
}

void UlcNode::sendCfgMsg()
{
  // Build CAN message
  can_msgs::Frame msg;
  msg.id = ID_ULC_CONFIG;
  msg.is_extended = false;
  msg.dlc = sizeof(MsgUlcCfg);
  MsgUlcCfg *ptr = (MsgUlcCfg *)msg.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));

  // Populate acceleration and jerk limits
  ptr->linear_accel  = overflowSaturation(ulc_cmd_.linear_accel,  0, UINT8_MAX, 0.025, "Linear accel limit",  "m/s^2");
  ptr->linear_decel  = overflowSaturation(ulc_cmd_.linear_decel,  0, UINT8_MAX, 0.025, "Linear decel limit",  "m/s^2");
  ptr->lateral_accel = overflowSaturation(ulc_cmd_.lateral_accel, 0, UINT8_MAX, 0.05, "Lateral accel limit", "m/s^2");
  ptr->angular_accel = overflowSaturation(ulc_cmd_.angular_accel, 0, UINT8_MAX, 0.02, "Angular accel limit", "rad/s^2");
  ptr->jerk_limit_throttle = overflowSaturation(ulc_cmd_.jerk_limit_throttle, 0, UINT8_MAX, 0.1, "Throttle jerk limit", "m/s^3");
  ptr->jerk_limit_brake = overflowSaturation(ulc_cmd_.jerk_limit_brake, 0, UINT8_MAX, 0.1, "Brake jerk limit", "m/s^3");

  // Publish message
  pub_can_.publish(msg);

  // Reset timer
  config_timer_.stop();
  config_timer_.start();
}

void UlcNode::configTimerCb(const ros::TimerEvent& event)
{
  // Retransmit config message while command is valid
  if (event.current_real - cmd_stamp_ < ros::Duration(0.1)) {
    sendCfgMsg();
  }
}

}
