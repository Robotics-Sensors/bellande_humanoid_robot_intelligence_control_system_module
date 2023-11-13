/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Kayman, Jay Song */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES

#include <boost/thread.hpp>
#include <cmath>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "action_file_define.h"
#include "humanoid_robot_intelligence_control_system_action_module_msgs/IsRunning.h"
#include "humanoid_robot_intelligence_control_system_action_module_msgs/StartAction.h"
#include "humanoid_robot_intelligence_control_system_controller_msgs/StatusMsg.h"
#include "humanoid_robot_intelligence_control_system_framework_common/motion_module.h"

namespace humanoid_robot_intelligence_control_system_op {

class ActionModule : public humanoid_robot_intelligence_control_system_framework::MotionModule,
                     public humanoid_robot_intelligence_control_system_framework::Singleton<ActionModule> {
public:
  ActionModule();
  virtual ~ActionModule();

  void initialize(const int control_cycle_msec,
                  humanoid_robot_intelligence_control_system_framework::Robot *robot);
  void process(std::map<std::string, humanoid_robot_intelligence_control_system_framework::Dynamixel *> dxls,
               std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  bool loadFile(std::string file_name);
  bool createFile(std::string file_name);

  bool start(int page_number);
  bool start(std::string page_name);
  bool start(int page_number, action_file_define::Page *page);

  void onModuleEnable();
  void onModuleDisable();

  void brake();
  bool isRunning(int *playing_page_num, int *playing_step_num);
  bool loadPage(int page_number, action_file_define::Page *page);
  bool savePage(int page_number, action_file_define::Page *page);
  void resetPage(action_file_define::Page *page);

  void enableAllJoints();
  void
  actionPlayProcess(std::map<std::string, humanoid_robot_intelligence_control_system_framework::Dynamixel *> dxls);

private:
  const int PRE_SECTION;
  const int MAIN_SECTION;
  const int POST_SECTION;
  const int PAUSE_SECTION;
  const int ZERO_FINISH;
  const int NONE_ZERO_FINISH;
  const bool DEBUG_PRINT;

  void queueThread();

  bool verifyChecksum(action_file_define::Page *page);
  void setChecksum(action_file_define::Page *page);

  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);

  bool
  isRunningServiceCallback(humanoid_robot_intelligence_control_system_action_module_msgs::IsRunning::Request &req,
                           humanoid_robot_intelligence_control_system_action_module_msgs::IsRunning::Response &res);

  void pageNumberCallback(const std_msgs::Int32::ConstPtr &msg);
  void
  startActionCallback(const humanoid_robot_intelligence_control_system_action_module_msgs::StartAction::ConstPtr &msg);

  int convertRadTow4095(double rad);
  double convertw4095ToRad(int w4095);
  std::string convertIntToString(int n);

  std::map<std::string, bool> action_joints_enable_;
  std::map<std::string, humanoid_robot_intelligence_control_system_framework::DynamixelState *> action_result_;
  int control_cycle_msec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Publisher status_msg_pub_;
  ros::Publisher done_msg_pub_;
  /////////////////////////////////////////////////////////////////////////
  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;
  FILE *action_file_;
  action_file_define::Page play_page_;
  action_file_define::Page next_play_page_;
  action_file_define::Step current_step_;

  int play_page_idx_;
  bool first_driving_start_;
  int page_step_count_;

  bool playing_;
  bool stop_playing_;
  bool playing_finished_;

  bool action_module_enabled_;
  bool previous_running_;
  bool present_running_;
};

} // namespace humanoid_robot_intelligence_control_system_op

#endif /* HUMANOID_ROBOT_ACTION_MOTION_MODULE_H_ */
