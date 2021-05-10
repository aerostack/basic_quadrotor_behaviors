/*!********************************************************************************
 * \brief     Land behavior implementation 
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

#include "../include/behavior_land.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorLand behavior;
  behavior.start();
  return 0;
}

BehaviorLand::BehaviorLand() : BehaviorExecutionManager() { 
  setName("land"); 
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorLand::~BehaviorLand() {}

bool BehaviorLand::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.state != aerostack_msgs::FlightState::LANDED){
    return true;
  }else{
    setErrorMessage("Error: Drone is already landed");
    return false;
  }
}

void BehaviorLand::checkGoal()
{
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
}

void BehaviorLand::checkProgress() 
{  

}

void BehaviorLand::checkProcesses() 
{ 
 
}

void BehaviorLand::onConfigure()
{
  nspace = getNamespace();
  nh = getNodeHandle();

  ros::param::get("~flight_action_topic", flight_action_str);
  ros::param::get("~flight_state_topic", state_str);

  status_sub = nh.subscribe("/" + nspace + "/"+state_str, 1, &BehaviorLand::statusCallBack, this);
}

void BehaviorLand::onActivate()
{
  flight_action_pub = nh.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + flight_action_str, 1, true);

  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::LAND;
  flight_action_pub.publish(msg);
}

void BehaviorLand::onDeactivate()
{
  flight_action_pub.shutdown();
}


void BehaviorLand::onExecute()
{
  
}

void BehaviorLand::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}
