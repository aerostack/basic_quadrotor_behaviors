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

#include "../include/behavior_hover_with_flight_action_control.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorHoverWithFlightActionControl behavior;
  behavior.start();
  return 0;
}

BehaviorHoverWithFlightActionControl::BehaviorHoverWithFlightActionControl() : BehaviorExecutionManager() { 
  setName("hover_with_flight_action_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorHoverWithFlightActionControl::~BehaviorHoverWithFlightActionControl() {}

void BehaviorHoverWithFlightActionControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace(); 
 
  ros::param::get("~flight_action_topic", flight_action_str);
  ros::param::get("~flight_state_topic", status_str);

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorHoverWithFlightActionControl::statusCallBack, this);
}

bool BehaviorHoverWithFlightActionControl::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.state != aerostack_msgs::FlightState::LANDED){
    return true;
  }
  else{
    setErrorMessage("Error: Drone is not flying");
    return false;
  }
}

void BehaviorHoverWithFlightActionControl::onActivate()
{
  flight_action_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+flight_action_str, 1, true);
  
  aerostack_msgs::FlightActionCommand flight_action_command;
  flight_action_command.header.stamp = ros::Time::now();
  flight_action_command.action = aerostack_msgs::FlightActionCommand::HOVER;
  flight_action_pub.publish(flight_action_command);  
}

void BehaviorHoverWithFlightActionControl::onDeactivate()
{
  flight_action_pub.shutdown();
}

void BehaviorHoverWithFlightActionControl::onExecute()
{ 
}

void BehaviorHoverWithFlightActionControl::checkGoal(){}


void BehaviorHoverWithFlightActionControl::checkProgress() {
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
}


void BehaviorHoverWithFlightActionControl::checkProcesses() 
{ 
 
}

void BehaviorHoverWithFlightActionControl::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}