/*!********************************************************************************
 * \brief     estimate_position_with_linear_speed implementation 
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef ESTIMATE_POSITION_WITH_LINEAR_SPEED_H
#define ESTIMATE_POSITION_WITH_LINEAR_SPEED_H

// System
#include <string>
#include <thread>
#include <tuple>
#include <iostream>
#include <fstream>
// ROS
#include "std_srvs/Empty.h"
#include <ros/ros.h>

#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <BehaviorExecutionManager.h>
#include <yaml-cpp/yaml.h>

class BehaviorEstimatePositionWithLinearSpeed : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorEstimatePositionWithLinearSpeed();
  ~BehaviorEstimatePositionWithLinearSpeed();
  int main(int argc, char** argv);

private:
  ros::NodeHandle nh;
  std::string nspace;

  ros::Subscriber linear_speed_sub;
  ros::Publisher position_pub;

  geometry_msgs::PointStamped position_msg;
  ros::Time time_since_started;

  ros::Time last_stamp;

private:
  // BehaviorExecutionManager
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

public: // Callbacks
  void linearSpeedCallBack(const geometry_msgs::TwistStamped &msg);
};

#endif
