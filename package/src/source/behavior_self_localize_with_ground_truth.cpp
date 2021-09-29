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
  ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic"   , estimated_pose_topic  ,"self_localization/pose");
  ros_utils_lib::getPrivateParam<std::string>("~estimated_speed_topic"  , estimated_speed_topic ,"self_localization/speed");
  ros_utils_lib::getPrivateParam<std::string>("~raw_pose_topic"         , raw_pose_topic        ,"ground_truth/pose");
  ros_utils_lib::getPrivateParam<std::string>("~raw_speed_topic"        , raw_speed_topic       ,"ground_truth/speed");
  ros_utils_lib::getPrivateParam<std::string>("~raw_ground_truth_topic" , raw_ground_truth      ,"ground_truth");
}

void BehaviorSelfLocalizeWithGroundTruth::onActivate()
{
  ground_pose_sub = nh.subscribe("/" + nspace + "/" + raw_pose_topic, 1, &BehaviorSelfLocalizeWithGroundTruth::groundPoseCallBack, this);
  ground_speed_sub = nh.subscribe("/" + nspace + "/" + raw_speed_topic, 1, &BehaviorSelfLocalizeWithGroundTruth::groundSpeedCallBack, this);
  ground_truth_sub = nh.subscribe("/" + nspace + "/" + raw_ground_truth, 1, &BehaviorSelfLocalizeWithGroundTruth::groundTruthCallBack, this);

  self_localization_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/" + estimated_pose_topic, 1, true);
  self_localization_speed_pub = nh.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/" + estimated_speed_topic, 1, true);
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

void BehaviorSelfLocalizeWithGroundTruth::groundTruthCallBack(const nav_msgs::Odometry &message)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose = message.pose.pose;
  self_localization_pose_pub.publish(pose);
  geometry_msgs::TwistStamped speed;
  speed.header.stamp = ros::Time::now();
  speed.twist = message.twist.twist;
  self_localization_speed_pub.publish(speed);
}
