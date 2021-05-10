/*!********************************************************************************
 * \brief     Take off behavior implementation 
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

#include "../include/behavior_take_off.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorTakeOff behavior;
  behavior.start();
  return 0;
}

BehaviorTakeOff::BehaviorTakeOff() : BehaviorExecutionManager() 
{ 
  setName("take_off");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorTakeOff::~BehaviorTakeOff() {}

void BehaviorTakeOff::onConfigure()
{ 
  nh = getNodeHandle();
  nspace = getNamespace();

  ros::param::get("~battery_topic", battery_topic);
  ros::param::get("~flight_action_topic", flight_action_str);
  ros::param::get("~flight_state_topic", state_str);

  //battery and pose params
  status_sub = nh.subscribe("/" + nspace + "/"+state_str, 1, &BehaviorTakeOff::statusCallBack, this);
  battery_subscriber = nh.subscribe("/" + nspace + "/"+battery_topic, 1, &BehaviorTakeOff::batteryCallback, this);
  isLow=false;
}

void BehaviorTakeOff::onActivate()
{
  flight_action_pub = nh.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + flight_action_str, 1, true);

  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
  flight_action_pub.publish(msg);
}

void BehaviorTakeOff::onDeactivate()
{
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  flight_action_pub.publish(msg);
  flight_action_pub.shutdown();
}

void BehaviorTakeOff::onExecute()
{
  
}

bool BehaviorTakeOff::checkSituation(){
  auto start = std::chrono::system_clock::now();
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  if (status_msg.state != aerostack_msgs::FlightState::LANDED && status_msg.state != aerostack_msgs::FlightState::UNKNOWN){
    setErrorMessage("Error: Already flying");
    rsp.situation_occurs = false;
  }else if(isLow){
    setErrorMessage("Error: Battery low, unable to perform action");
    rsp.situation_occurs = false;
  }
  else{
    rsp.situation_occurs = true;
  }
  std::cout<<"AQUI: "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-start).count()<<std::endl;
  return rsp.situation_occurs;
}

void BehaviorTakeOff::batteryCallback(const sensor_msgs::BatteryState& battery) {
	if(battery.percentage * 100 < BATTERY_LOW_THRESHOLD) {
		isLow=true;
	}else{
    isLow=false;
  }
}

void BehaviorTakeOff::checkGoal()
{
  // Check achievement
  if (status_msg.state == aerostack_msgs::FlightState::FLYING || status_msg.state == aerostack_msgs::FlightState::HOVERING){
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
}

void BehaviorTakeOff::checkProgress() {}

void BehaviorTakeOff::checkProcesses() {}

void BehaviorTakeOff::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}
