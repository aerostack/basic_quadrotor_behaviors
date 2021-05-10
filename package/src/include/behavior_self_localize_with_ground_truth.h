/*!********************************************************************************
 * \brief     self_localize_with_ground_truth implementation 
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

#ifndef SELF_LOCALIZE_GROUND_TRUTH_H
#define SELF_LOCALIZE_GROUND_TRUTH_H

// System
#include <string>
#include <thread>
#include <tuple>
#include <iostream>
#include <fstream>
// ROS
#include "std_srvs/Empty.h"
#include <ros/ros.h>

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <BehaviorExecutionManager.h>

class BehaviorSelfLocalizeWithGroundTruth : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorSelfLocalizeWithGroundTruth();
  ~BehaviorSelfLocalizeWithGroundTruth();
  int main(int argc, char** argv);

private:
  ros::NodeHandle nh;
  std::string nspace;

  ros::Subscriber ground_speed_sub;
  ros::Subscriber ground_pose_sub;

  ros::Publisher self_localization_pose_pub;
  ros::Publisher self_localization_speed_pub;

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
  void groundPoseCallBack(const geometry_msgs::PoseStamped &message);
  void groundSpeedCallBack(const geometry_msgs::TwistStamped &message);
};

#endif
