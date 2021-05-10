/*!********************************************************************************
 * \brief     hover implementation
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

#ifndef HOVER_H
#define HOVER_H

// System
#include <string>
#include "math.h"
// ROS
#include "std_srvs/Empty.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/FlightActionCommand.h>
#include "aerostack_msgs/FlightState.h"
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>

// Aerostack libraries
#include <BehaviorExecutionManager.h>

class BehaviorHoverWithFlightActionControl : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorHoverWithFlightActionControl();
  ~BehaviorHoverWithFlightActionControl();
  int main(int argc, char** argv);

private:
  // Congfig variables
  std::string flight_action_str;
  std::string status_str;

  ros::NodeHandle node_handle;
  std::string nspace; 
 
  ros::Subscriber status_sub;
  ros::Publisher flight_action_pub;

  aerostack_msgs::FlightState status_msg;

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

  // Callbacks
  void statusCallBack(const aerostack_msgs::FlightState &msg);
};

#endif
