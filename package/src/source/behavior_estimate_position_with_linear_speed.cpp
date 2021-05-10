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

#include "../include/behavior_estimate_position_with_linear_speed.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorEstimatePositionWithLinearSpeed behavior;
  behavior.start();
  return 0;
}

BehaviorEstimatePositionWithLinearSpeed::BehaviorEstimatePositionWithLinearSpeed() : BehaviorExecutionManager() 
{ 
  setName("estimate_position_with_linear_speed");
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING);
}

BehaviorEstimatePositionWithLinearSpeed::~BehaviorEstimatePositionWithLinearSpeed() {}

void BehaviorEstimatePositionWithLinearSpeed::onConfigure()
{ 
  nh = getNodeHandle();
  nspace = getNamespace();

  last_stamp.sec = 0;
}

void BehaviorEstimatePositionWithLinearSpeed::onActivate()
{
  linear_speed_sub = nh.subscribe("/" + nspace + "/sensor_measurement/linear_speed", 1, &BehaviorEstimatePositionWithLinearSpeed::linearSpeedCallBack, this);
  position_pub = nh.advertise<geometry_msgs::PointStamped>("/" + nspace + "/self_localization/position", 1, true);

  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["initial_position"].IsDefined())
  {
    std::vector<double> point=config_file["initial_position"].as<std::vector<double>>();
    position_msg.point.x = point[0];
    position_msg.point.y = point[1];
    position_msg.point.z = point[2];
  }else{
    std::cout<<"BehaviorEstimatePositionWithLinearSpeed: Initial position (0,0,0)"<<std::endl;    
    position_msg.point.x = 0;
    position_msg.point.y = 0;
    position_msg.point.z = 0;
  }

  time_since_started = ros::Time::now();
}

void BehaviorEstimatePositionWithLinearSpeed::onDeactivate()
{
  linear_speed_sub.shutdown();
  position_pub.shutdown();
}

void BehaviorEstimatePositionWithLinearSpeed::onExecute()
{
  
}

bool BehaviorEstimatePositionWithLinearSpeed::checkSituation()
{
  return true;
}


void BehaviorEstimatePositionWithLinearSpeed::checkGoal()
{

}

// Estimating position with linear speed loses precision as time passes 
void BehaviorEstimatePositionWithLinearSpeed::checkProgress() 
{ 
  if (ros::Time::now().toSec() - time_since_started.toSec() >= 15*60){ //15 minutes
    //BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
}

void BehaviorEstimatePositionWithLinearSpeed::checkProcesses() 
{ 
 
}

void BehaviorEstimatePositionWithLinearSpeed::linearSpeedCallBack(const geometry_msgs::TwistStamped &msg)
{
  if(last_stamp.sec == 0){
    last_stamp = msg.header.stamp;
    return;
  }

  ros::Duration diff = msg.header.stamp - last_stamp;

  position_msg.header.stamp = msg.header.stamp;
  position_msg.point.x += msg.twist.linear.x * diff.toSec();
  position_msg.point.y += msg.twist.linear.y * diff.toSec();
  position_msg.point.z += msg.twist.linear.z * diff.toSec();

  last_stamp = msg.header.stamp;
  position_pub.publish(position_msg);
}
