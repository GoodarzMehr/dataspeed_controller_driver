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

#ifndef ULCNODE_H
#define ULCNODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <can_msgs/Frame.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <dataspeed_ulc_msgs/UlcReport.h>
#include <dataspeed_ulc_can/PlatformMap.h>
#include <autoware_msgs/VehicleCmd.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_msgs/DriverStatus.h>

namespace dataspeed_ulc_can
{

class UlcNode
{
public:
  UlcNode(ros::NodeHandle &n, ros::NodeHandle &pn);
private:

  void recvCan(const can_msgs::FrameConstPtr& msg);
  void recvUlcCmd(const dataspeed_ulc_msgs::UlcCmdConstPtr& msg);
  void recvTwistCmd(const geometry_msgs::Twist& msg);
  void recvTwist(const geometry_msgs::TwistConstPtr& msg);
  void recvTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg);
  void recvAutowareTwistStamped(const geometry_msgs::TwistStamped& msg);
  void recvControlCmd(const autoware_msgs::ControlCommand& msg);
  void recvVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg);
  void recvEnable(const std_msgs::BoolConstPtr& msg);
  void configTimerCb(const ros::TimerEvent& event);
  void sendCmdMsg(bool cfg);
  void sendCfgMsg();

  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_twist_stamped_;
  ros::Subscriber sub_can_;
  ros::Subscriber sub_enable_;
  ros::Subscriber sub_vehicle_cmd_;
  ros::Publisher pub_report_;
  ros::Publisher pub_can_;
  ros::Publisher pub_robot_status_;
  ros::Publisher pub_discovery_;
  ros::Timer config_timer_;

  dataspeed_ulc_msgs::UlcCmd ulc_cmd_;
  cav_msgs::RobotEnabled robot_enabled_;
  cav_msgs::DriverStatus driver_status_;
  ros::Time last_discovery_pub_;
  ros::Time last_status_pub_;
  ros::Time cmd_stamp_;
  double current_speed_;
  bool enable_;
  bool active_;
  bool accel_mode_supported_;

  // Firmware Versions
  PlatformMap firmware_;
};

}

#endif // ULCNODE_H
