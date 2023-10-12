#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "libraries/utils.hpp"
#include "modules/interpolate_trajectory/interpolate_trajectory.hpp"
#include "tasks/task_monitor.hpp"
#include "tasks/task_state_machine.hpp"

const float lower_bounder[6] = {-0.2f, -0.54f, -0.1f, -0.98f, -1.05f, -1.35f};
const float upper_bounder[6] = {0.54f, 0.54f, 0.31f, 0.98f, 1.57f, 1.35f};

ros::Publisher arm_pose_publisher, arm_spd_publisher;
geometry_msgs::Pose airbot_pose_msg;
geometry_msgs::Twist airbot_spd_msg;

bool recv_newpos = false;
bool recv_newjoint = false;
bool recv_newtrajectory = false;
std::vector<float> xyz_rpy(6);
std::vector<float> joints_angle(6);
std::vector<std::vector<double>> trajectory;
size_t trajectory_index = 0;

/**
 * Compute the euler angles zyx corresponding to quaternion
 * @param [in]  quaternion [wxyz]
 * @return The corresponding eulerAnglesZyx [rpy]
 */
std::vector<double> QuaternionToEuler(const std::vector<double>& quat) {
  std::vector<double> euler(3);
  euler[2] = std::atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                        1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
  euler[1] = std::asin(2.0 * (quat[0] * quat[2] - quat[3] * quat[1]));
  euler[0] = std::atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]),
                        1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
  return euler;
}

// 笛卡尔转关节，然后插值
std::vector<std::vector<double>> interpolate_target_pose_with_current_pose(
    std::vector<float> target_xyz_rpy, double execute_time) {
  static ServoController servo;
  std::vector<float> target_joint_values;
  std::vector<float> current_joint_values;
  servo.Move2Point(target_xyz_rpy, &target_joint_values);
  servo.Move2Point(CommandBase::ReadPos(), &current_joint_values);
  std::vector<double> t = {0, execute_time};
  std::vector<std::vector<double>> y = {
      std::vector<double>{current_joint_values.begin(),
                          current_joint_values.end()},
      std::vector<double>{target_joint_values.begin(),
                          target_joint_values.end()}};

  std::vector<std::vector<double>> joint_cmd;
  const float kCONTROL_PERIOD = 0.005;
  joint_cmd =
      interpolate_trajectory::JointTrajectoryInterpolate(t, y, kCONTROL_PERIOD);
  auto vel_cmd = interpolate_trajectory::CalculateDifference(200, joint_cmd);
  return joint_cmd;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_ptr) {
  if (recv_newtrajectory) return;
  xyz_rpy[0] = pose_ptr->position.x;
  xyz_rpy[1] = pose_ptr->position.y;
  xyz_rpy[2] = pose_ptr->position.z;
  std::vector<double> quat(4);
  quat[0] = pose_ptr->orientation.w;
  quat[1] = pose_ptr->orientation.x;
  quat[2] = pose_ptr->orientation.y;
  quat[3] = pose_ptr->orientation.z;
  std::vector<double> euler = QuaternionToEuler(quat);
  // :TODO: 确认一下欧拉角的旋转顺序xyz
  xyz_rpy[3] = static_cast<float>(euler[0]);
  xyz_rpy[4] = static_cast<float>(euler[1]);
  xyz_rpy[5] = static_cast<float>(euler[2]);
  for (u_int8_t i : {0, 1, 2, 3, 4, 5}) {
    xyz_rpy[i] = arm::limit(xyz_rpy[i], lower_bounder[i], upper_bounder[i]);
  }
  bool interpolate = true;
  if (interpolate) {
    trajectory = interpolate_target_pose_with_current_pose(xyz_rpy, 1.0);
    recv_newtrajectory = true;
  } else
    recv_newpos = true;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& joint_ptr) {
  if (joint_ptr->position.size() == 6) {
    for (size_t i = 0; i < 6; i++) {
      joints_angle[i] = joint_ptr->position[i];
    }
    recv_newjoint = true;
  } else {
    ROS_WARN("Wrong size of JointState msg.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "airbot_play_modules_usr_use_two_hand_test");
  ros::NodeHandle node;
  ros::Rate loop_rate(200);

  arm_pose_publisher =
      node.advertise<geometry_msgs::Pose>("/airbot_play1/arm_pose", 10);
  arm_spd_publisher =
      node.advertise<geometry_msgs::Twist>("/airbot_play1/arm_speed", 10);

  ros::Subscriber pose_sub = node.subscribe<geometry_msgs::Pose>(
      "/airbot_play1/set_pose", 2, poseCallback);
  ros::Subscriber joint_sub = node.subscribe<sensor_msgs::JointState>(
      "/airbot_play1/set_joint", 2, jointCallback);

  std::vector<float> arm_pose(7), arm_spd(6);

  std::string model_path = ros::package::getPath("ros_interface") +
                           "/models/urdf/airbot_play_v1.urdf";
  KdlModel::SetModelPath(model_path);

  arm::FsmTask fsm_task;
  arm::MonitorTask monitor_task;

  ROS_INFO("arm control begin");

  const float joint_lower_bounder[6] = {-1.57f, -3.14f, 0.00f,
                                        -1.57f, -1.00f, -3.14f};
  const float joint_upper_bounder[6] = {1.57f, 0.00f, 1.57f,
                                        1.57f, 1.00f, 3.14f};

  // const float joint_lower_bounder[6] = {-2.09 + 0.17, -3.14 + 0.17, 0.0f ,
  // -2.61f + 0.17, -1.57f + 0.17, -3.14f + 0.17}; const float
  // joint_upper_bounder[6] = { 3.14 - 0.17,  0.0        , 2.35f - 0.17,  1.57f
  // - 0.17,  1.57f - 0.17,  3.14f - 0.17};

  std::vector<float> joint_spd_limit(6);
  for (size_t i = 0; i < 6; i++) {
    joint_spd_limit[i] = 4.0f;
  }
  CommandBase::add_joint_task_spd(joint_spd_limit);
  fsm_task.Handle();
  bool normal_state = true;
  while (ros::ok()) {
    if (recv_newpos) {
      if (!normal_state) {
        CommandBase::Switch2Normal();
        normal_state = true;
      }
      CommandBase::set_target_task_pos(xyz_rpy);
      recv_newpos = false;
    }

    if (recv_newjoint) {
      if (normal_state) {
        CommandBase::Switch2JointSpace();
        normal_state = false;
      }
      for (u_int8_t i : {0, 1, 2, 3, 4, 5}) {
        joints_angle[i] = arm::limit(joints_angle[i], joint_lower_bounder[i],
                                     joint_upper_bounder[i]);
      }
      CommandBase::add_target_joint_pos(joints_angle);
      // CommandBase::set_target_joint_pos(joints_angle);
      recv_newjoint = false;
    }

    if (recv_newtrajectory) {
      if (normal_state) {
        CommandBase::Switch2JointSpace();
        normal_state = false;
      }
      auto joint_point_double = trajectory[trajectory_index];
      auto joint_point = std::vector<float>{joint_point_double.begin(),
                                            joint_point_double.end()};
      CommandBase::add_target_joint_pos(joint_point);
      trajectory_index++;
      if (trajectory_index >= trajectory.size()) {
        trajectory_index = 0;
        recv_newtrajectory = false;
      }
    }

    monitor_task.Handle();
    fsm_task.Handle();

    arm_pose = CommandBase::ReadPos();
    arm_spd = CommandBase::ReadVel();

    airbot_pose_msg.position.x = arm_pose[0];
    airbot_pose_msg.position.y = arm_pose[1];
    airbot_pose_msg.position.z = arm_pose[2];
    airbot_pose_msg.orientation.x = arm_pose[3];
    airbot_pose_msg.orientation.y = arm_pose[4];
    airbot_pose_msg.orientation.z = arm_pose[5];
    airbot_pose_msg.orientation.w = arm_pose[6];
    // publish arm pose msg
    arm_pose_publisher.publish(airbot_pose_msg);

    airbot_spd_msg.linear.x = arm_spd[0];
    airbot_spd_msg.linear.y = arm_spd[1];
    airbot_spd_msg.linear.z = arm_spd[2];
    airbot_spd_msg.angular.x = arm_spd[3];
    airbot_spd_msg.angular.y = arm_spd[4];
    airbot_spd_msg.angular.z = arm_spd[5];
    // publisht arm pose speed msg
    arm_spd_publisher.publish(airbot_spd_msg);

    // clear arm pose msg
    airbot_pose_msg = geometry_msgs::Pose();
    // clear arm pose speed msg
    airbot_spd_msg = geometry_msgs::Twist();

    ros::spinOnce();
    loop_rate.sleep();
  }

  if (!normal_state) {
    CommandBase::Switch2Normal();
    normal_state = true;
  }
  xyz_rpy[0] = 0.0;
  xyz_rpy[1] = 0.0;
  xyz_rpy[2] = 0.0;
  xyz_rpy[3] = 0.0;
  xyz_rpy[4] = 0.0;
  xyz_rpy[5] = 0.0;
  CommandBase::set_target_task_pos(xyz_rpy);
  for (int i = 0; i < 200; i++) {
    fsm_task.Handle();
    loop_rate.sleep();
  }

  return 0;
}
