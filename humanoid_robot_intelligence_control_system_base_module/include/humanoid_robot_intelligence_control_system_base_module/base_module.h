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

/* Authors: SCH, Kayman */

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <boost/thread.hpp>
#include <map>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Pose.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include "humanoid_robot_intelligence_control_system_kinematics_dynamics/humanoid_robot_intelligence_control_system_kinematics_dynamics.h"
#include "humanoid_robot_intelligence_control_system_controller_msgs/JointCtrlModule.h"
#include "humanoid_robot_intelligence_control_system_controller_msgs/SetModule.h"
#include "humanoid_robot_intelligence_control_system_controller_msgs/StatusMsg.h"
#include "humanoid_robot_intelligence_control_system_framework_common/motion_module.h"
#include "humanoid_robot_intelligence_control_system_math/humanoid_robot_intelligence_control_system_math.h"

#include "base_module_state.h"

namespace humanoid_robot_intelligence_control_system_op {

class BaseJointData {
public:
  double position_;
  double velocity_;
  double effort_;

  int p_gain_;
  int i_gain_;
  int d_gain_;
};

class BaseJointState {

public:
  BaseJointData curr_joint_state_[MAX_JOINT_ID + 1];
  BaseJointData goal_joint_state_[MAX_JOINT_ID + 1];
  BaseJointData fake_joint_state_[MAX_JOINT_ID + 1];
};

class BaseModule : public humanoid_robot_intelligence_control_system_framework::MotionModule,
                   public humanoid_robot_intelligence_control_system_framework::Singleton<BaseModule> {
public:
  BaseModule();
  virtual ~BaseModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec,
                  humanoid_robot_intelligence_control_system_framework::Robot *robot);
  void process(std::map<std::string, humanoid_robot_intelligence_control_system_framework::Dynamixel *> dxls,
               std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::String::ConstPtr &msg);

  /* ROS Calculation Functions */
  void initPoseTrajGenerateProc();

  void poseGenerateProc(Eigen::MatrixXd joint_angle_pose);
  void poseGenerateProc(std::map<std::string, double> &joint_angle_pose);

  /* Parameter */
  BaseModuleState *base_module_state_;
  BaseJointState *joint_state_;

private:
  void queueThread();
  void setCtrlModule(std::string module);
  void callServiceSettingModule(const std::string &module_name);
  void parseInitPoseData(const std::string &path);
  void publishStatusMsg(unsigned int type, std::string msg);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread tra_gene_tread_;

  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;

  ros::ServiceClient set_module_client_;

  std::map<std::string, int> joint_name_to_id_;

  bool has_goal_joints_;
  bool ini_pose_only_;

  std::string init_pose_file_path_;
};

} // namespace humanoid_robot_intelligence_control_system_op

#endif /* BASEMODULE_H_ */
