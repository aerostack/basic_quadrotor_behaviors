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

#include "../include/behavior_self_localize_with_ground_truth.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorSelfLocalizeWithGroundTruth behavior;
  behavior.start();
  return 0;
}

BehaviorSelfLocalizeWithGroundTruth::BehaviorSelfLocalizeWithGroundTruth() : BehaviorExecutionManager() 
{ 
  setName("self_localize_with_ground_truth");
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING);
}

BehaviorSelfLocalizeWithGroundTruth::~BehaviorSelfLocalizeWithGroundTruth() {}

void BehaviorSelfLocalizeWithGroundTruth::onConfigure()
{ 
  nh = getNodeHandle();
  nspace = getNamespace();
}

void BehaviorSelfLocalizeWithGroundTruth::onActivate()
{
  ground_pose_sub = nh.subscribe("/" + nspace + "/ground_truth/pose", 1, &BehaviorSelfLocalizeWithGroundTruth::groundPoseCallBack, this);
  ground_speed_sub = nh.subscribe("/" + nspace + "/ground_truth/speed", 1, &BehaviorSelfLocalizeWithGroundTruth::groundSpeedCallBack, this);
  
  self_localization_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/self_localization/pose", 1, true);
  self_localization_speed_pub = nh.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/self_localization/speed", 1, true);
}
void BehaviorSelfLocalizeWithGroundTruth::onDeactivate()
{
  ground_pose_sub.shutdown();
  ground_speed_sub.shutdown();
  self_localization_pose_pub.shutdown();
  self_localization_speed_pub.shutdown();
}

void BehaviorSelfLocalizeWithGroundTruth::onExecute()
{
  
}

bool BehaviorSelfLocalizeWithGroundTruth::checkSituation()
{
  return true;
}

void BehaviorSelfLocalizeWithGroundTruth::checkGoal()
{

}

void BehaviorSelfLocalizeWithGroundTruth::checkProgress() 
{ 
 
}

void BehaviorSelfLocalizeWithGroundTruth::checkProcesses() 
{ 
 
}

// Custom topic Callbacks
void BehaviorSelfLocalizeWithGroundTruth::groundPoseCallBack(const geometry_msgs::PoseStamped &message)
{
  self_localization_pose_pub.publish(message);
}

void BehaviorSelfLocalizeWithGroundTruth::groundSpeedCallBack(const geometry_msgs::TwistStamped &message)
{
  self_localization_speed_pub.publish(message);
}
