/*!********************************************************************************
 * \brief     Wait behavior implementation 
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

#include "../include/behavior_wait.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorWait behavior;
  behavior.start();
  return 0;
}

BehaviorWait::BehaviorWait() : BehaviorExecutionManager() 
{ 
  setName("wait");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
 }

BehaviorWait::~BehaviorWait() {}

bool BehaviorWait::checkSituation()
{
  return true;
}

void BehaviorWait::checkGoal()
{
    if(timeout_end && timer_msg)
    {
      BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
    }
}

void BehaviorWait::checkProgress()
{ 

}

void BehaviorWait::checkProcesses() 
{ 
 
}

void BehaviorWait::onConfigure()
{
  nh = getNodeHandle();
  nspace = getNamespace();

  timer_msg=false;
}

void BehaviorWait::onActivate()
{
  timer_msg=false;
  timeout_end=false;
  /*behavior implementation*/
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["duration"].IsDefined())
  {
    timeout=config_file["duration"].as<float>();
    timeout_end=true;
    timer = nh.createTimer(ros::Duration(timeout), &BehaviorWait::timerCallback, this);
  }
}

void BehaviorWait::onDeactivate()
{
}

void BehaviorWait::onExecute()
{

}

void BehaviorWait::timerCallback(const ros::TimerEvent& event)
{
  timer_msg=true;
}
