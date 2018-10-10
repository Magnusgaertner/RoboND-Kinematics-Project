//
// Created by magnus on 18.06.18.
//

/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya, Magnus Gärtner

#include <kuka_arm/stomp_trajectory_sampler.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>

#include "moveit/collision_plugin_loader/collision_plugin_loader.h"

void setStartStateToCurrentStateStomp(moveit::planning_interface::MoveGroupInterface &move_group) {
  robot_state::RobotState robot_state = *move_group.getCurrentState();
  geometry_msgs::PoseStamped eef_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
  move_group.setStartState(robot_state);
}

void setPoseTargetStomp(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose &pose) {
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(move_group.getName());

  Eigen::Affine3d target_pose1;
  tf::poseMsgToEigen(pose, target_pose1);
  robot_state::RobotState robot_state = *move_group.getCurrentState();
  robot_state.setFromIK(joint_model_group, target_pose1, 5, 1.0);

  move_group.setJointValueTarget(robot_state);

}

StompTrajectorySampler::StompTrajectorySampler(ros::NodeHandle nh)
    : nh_(nh),
      cycle_counter(0),
      move_group(PLANNING_GROUP),
      eef_group(GRIPPER_GROUP) {
  /*
   * Setup:
   * Load robot model, robot state, set planning_scene
   * Define the move_group for planning and control purpose
   */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene_ptr (new planning_scene::PlanningScene(kinematic_model));
  planning_scene::PlanningScene& planning_scene = *planning_scene_ptr;
  robot_state::RobotState robot_kinematic_state(kinematic_model);


  // set Stom as the planner and set allowed planning time
  move_group.setPlannerId("STOMP");
  move_group.setPlanningTime(100.0);
  eef_group.setPlannerId("STOMP");
  eef_group.setPlanningTime(100.0);

  ROS_INFO("My Active planning scene: %s",planning_scene.getActiveCollisionDetectorName().c_str());

  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  gripper_joint_model_group =
      eef_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);


  if(ros::ok()){

    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();


    //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //Eigen::Affine3d instr_pose = Eigen::Affine3d::Identity();
    //text_pose.translation().z() = 4.0;
    //instr_pose.translation().z() = 3.5;

    //visual_tools.publishText(text_pose, "Welcome to tsdf integration moveit",
                             //rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    //visual_tools.trigger();
    //visual_tools.prompt("next step");


    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;

    setStartStateToCurrentStateStomp(move_group);
    setPoseTargetStomp(move_group, target_pose);

    move_group.setMaxVelocityScalingFactor(0.2);
    eef_group.setMaxVelocityScalingFactor(1.0);

    // define plan object which will hold the planned trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Visualizing plan to target: %s",
             success ? "SUCCEEDED" : "FAILED");

    //visualize_plan(visual_tools, text_pose, target_pose, my_plan);
    //visual_tools.trigger();
    //visual_tools.prompt("next step");

    success = move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Moving to pick location: %s",
             success ? "SUCCEEDED" : "FAILED");
    /*
    setStartStateToCurrentStateStomp(move_group);
    setPoseTargetStomp(move_group, target_reach);

    success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Target reach: %s",
             success ? "SUCCEEDED" : "FAILED");*/
  }
}

void StompTrajectorySampler::visualize_plan(moveit_visual_tools::MoveItVisualTools &visual_tools,
                                            const Eigen::Affine3d &text_pose, const geometry_msgs::Pose &target_pose,
                                            const moveit::planning_interface::MoveGroupInterface::Plan &my_plan) const {// Visualize the plan
  visual_tools.publishAxisLabeled(target_pose, "target_pose");
  visual_tools.publishText(text_pose, "Displaying plan to target location",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
}



bool StompTrajectorySampler::OperateGripper(const bool &close_gripper) {
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
      eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
                                                 gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper) {
    gripper_joint_positions[0] = 0.02;  // radians
    gripper_joint_positions[1] = 0.02;  // radians
  } else {
    gripper_joint_positions[0] = -0.01;  // radians
    gripper_joint_positions[1] = -0.01;  // radians
  }

  eef_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = eef_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  return success;
}

bool StompTrajectorySampler::OpenGripper() {
  bool success = OperateGripper(false);
  ROS_INFO("Gripper actuation: Opening %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool StompTrajectorySampler::CloseGripper() {
  bool success = OperateGripper(true);
  ROS_INFO("Gripper actuation: Closing %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}




StompTrajectorySampler::~StompTrajectorySampler() {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "stomp_trajectory_sampler");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  StompTrajectorySampler plan_sampler(nh);
  return 0;
}
