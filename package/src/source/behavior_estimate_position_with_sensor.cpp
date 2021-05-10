/*!********************************************************************************
 * \brief     estimate_position_with_sensor implementation 
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

#include "../include/behavior_estimate_position_with_sensor.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorEstimatePositionWithSensor behavior;
  behavior.start();
  return 0;
}

BehaviorEstimatePositionWithSensor::BehaviorEstimatePositionWithSensor() : BehaviorExecutionManager() 
{ 
  setName("estimate_position_with_sensor");
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING);
  sensor_position_sub = nh.subscribe("/" + nspace + "/sensor_measurement/position", 1, &BehaviorEstimatePositionWithSensor::sensorPositionCallBack, this);
}

BehaviorEstimatePositionWithSensor::~BehaviorEstimatePositionWithSensor() {}

void BehaviorEstimatePositionWithSensor::onConfigure()
{ 
  nh = getNodeHandle();
  nspace = getNamespace(); 
}

void BehaviorEstimatePositionWithSensor::onActivate()
{
  localization_position_pub = nh.advertise<geometry_msgs::PointStamped>("/" + nspace + "/self_localization/position", 1, true);
}

void BehaviorEstimatePositionWithSensor::onDeactivate()
{
  //sensor_position_sub.shutdown();
  localization_position_pub.shutdown();
}

void BehaviorEstimatePositionWithSensor::onExecute()
{
  
}

bool BehaviorEstimatePositionWithSensor::checkSituation()
{
  if(position_msg.header.stamp.toSec() != 0) return true;
  else return false;
}

void BehaviorEstimatePositionWithSensor::checkGoal()
{
}

void BehaviorEstimatePositionWithSensor::checkProgress() 
{ 
 
}

void BehaviorEstimatePositionWithSensor::checkProcesses() 
{ 
 
}

void BehaviorEstimatePositionWithSensor::sensorPositionCallBack(const geometry_msgs::PointStamped &msg)
{
  position_msg = msg;
  localization_position_pub.publish(msg);
}
