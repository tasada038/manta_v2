// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(
  std::shared_future<rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::SharedPtr> future)
{
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("SUCCEEDED result code\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Goal was canceled\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  std::cout << "feedback->desired.positions :";
  for (auto & x : feedback->desired.positions) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
  std::cout << "feedback->desired.velocities :";
  for (auto & x : feedback->desired.velocities) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("trajectory_test_node");

  std::cout << "node created" << std::endl;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  bool response =
    action_client->wait_for_action_server(std::chrono::seconds(1));
  if (!response) {
    throw std::runtime_error("could not get action server");
  }
  std::cout << "Created action server" << std::endl;

  /*
  // convert_pyyaml_back_tripod.py
  std::vector<double> l_r1 = {
      30.0, 25.97, 21.91, 17.9, 14.0, 10.25, 6.7, 3.41, 0.41, -2.26, 
      -4.57, -6.5, -8.02, -9.12, -9.79, -10.02, -9.8, -9.15, -8.06, -6.55, 
      -4.64, -2.34, 0.31, 3.28, 6.55, 10.06, 13.78, 17.65, 21.62, 25.63, 
      29.61, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 
      30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 
      30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 
      30.0
  };

  std::vector<double> r_r1 = {
      -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 
      -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 
      -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 
      -30.0, -25.97, -21.91, -17.9, -14.0, -10.25, -6.7, -3.41, -0.41, 2.26, 
      4.57, 6.5, 8.02, 9.12, 9.79, 10.02, 9.8, 9.15, 8.06, 6.55, 
      4.64, 2.34, -0.31, -3.28, -6.55, -10.06, -13.78, -17.65, -21.62, -25.63, 
      -29.61
  };

  std::vector<double> ltop_p = {
      -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
      -40.0, -38.05, -36.2, -34.45, -32.8, -31.25, -29.8, -28.45, -27.2, -26.05,
      -25.0, -24.05, -23.2, -22.45, -21.8, -21.25, -20.8, -20.45, -20.2, -20.05,
      -20.0, -20.05, -20.2, -20.45, -20.8, -21.25, -21.8, -22.45, -23.2, -24.05,
      -25.0, -26.05, -27.2, -28.45, -29.8, -31.25, -32.8, -34.45, -36.2, -38.05,
      -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, 
      -40.0
  };

  std::vector<double> lmid_p = {
      20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
      20.0, 18.05, 16.2, 14.45, 12.8, 11.25, 9.8, 8.45, 7.2, 6.05,
      5.0, 4.05, 3.2, 2.45, 1.8, 1.25, 0.8, 0.45, 0.2, 0.05,
      0.0, 0.05, 0.2, 0.45, 0.8, 1.25, 1.8, 2.45, 3.2, 4.05,
      5.0, 6.05, 7.2, 8.45, 9.8, 11.25, 12.8, 14.45, 16.2, 18.05,
      20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
      20.0
  };

  std::vector<double> lbtm_p = {
      10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
      10.0, 11.95, 13.8, 15.55, 17.2, 18.75, 20.2, 21.55, 22.8, 23.95,
      25.0, 25.95, 26.8, 27.55, 28.2, 28.75, 29.2, 29.55, 29.8, 29.95,
      30.0, 29.95, 29.8, 29.55, 29.2, 28.75, 28.2, 27.55, 26.8, 25.95,
      25.0, 23.95, 22.8, 21.55, 20.2, 18.75, 17.2, 15.55, 13.8, 11.95,
      10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
      10.0
  };

  std::vector<double> rtop_p = {
      20.0, 20.05, 20.2, 20.45, 20.8, 21.25, 21.8, 22.45, 23.2, 24.05,
      25.0, 26.05, 27.2, 28.45, 29.8, 31.25, 32.8, 34.45, 36.2, 38.05,
      40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
      40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
      40.0, 40.0, 38.05, 36.2, 34.45, 32.8, 31.25, 29.8, 28.45, 27.2,
      26.05, 25.0, 24.05, 23.2, 22.45, 21.8, 21.25, 20.8, 20.45, 20.2,
      20.05
  };

  std::vector<double> rmid_p = {
      0.0, -0.05, -0.2, -0.45, -0.8, -1.25, -1.8, -2.45, -3.2, -4.05,
      -5.0, -6.05, -7.2, -8.45, -9.8, -11.25, -12.8, -14.45, -16.2, -18.05,
      -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0,
      -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0,
      -20.0, -20.0, -18.05, -16.2, -14.45, -12.8, -11.25, -9.8, -8.45, -7.2,
      -6.05, -5.0, -4.05, -3.2, -2.45, -1.8, -1.25, -0.8, -0.45, -0.2,
      -0.05
  };

  std::vector<double> rbtm_p = {
      -30.0, -29.95, -29.8, -29.55, -29.2, -28.75, -28.2, -27.55, -26.8, -25.95,
      -25.0, -23.95, -22.8, -21.55, -20.2, -18.75, -17.2, -15.55, -13.8, -11.95,
      -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
      -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
      -10.0, -10.0, -11.95, -13.8, -15.55, -17.2, -18.75, -20.2, -21.55, -22.8,
      -23.95, -25.0, -25.95, -26.8, -27.55, -28.2, -28.75, -29.2, -29.55, -29.8,
      -29.95
  };
  */


  std::vector<double> l_r1 = {
    30.0, 27.98, 25.96, 23.95, 22.0, 20.12, 18.35, 16.71, 15.21, 13.88,
    12.72, 11.76, 11.0, 10.45, 10.12, 10.01, 10.12, 10.45, 11.0, 11.76,
    12.72, 13.87, 15.21, 16.7, 18.34, 20.11, 21.98, 23.92, 25.92, 27.94,
    29.94, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
    30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
    30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
    30.0
  };

  std::vector<double> r_r1 = {
    -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
    -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
    -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
    -30.0, -27.98, -25.96, -23.95, -22.0, -20.12, -18.35, -16.71, -15.21, -13.88,
    -12.72, -11.76, -11.0, -10.45, -10.12, -10.01, -10.12, -10.45, -11.0, -11.76,
    -12.72, -13.87, -15.21, -16.7, -18.34, -20.11, -21.98, -23.92, -25.92, -27.94,
    -29.94
  };

  std::vector<double> ltop_p = {
    -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
    -40.0, -36.1, -32.4, -28.9, -25.6, -22.5, -19.6, -16.9, -14.4, -12.1,
    -10.0, -8.1, -6.4, -4.9, -3.6, -2.5, -1.6, -0.9, -0.4, -0.1,
    0.0, -0.1, -0.4, -0.9, -1.6, -2.5, -3.6, -4.9, -6.4, -8.1,
    -10.0, -12.1, -14.4, -16.9, -19.6, -22.5, -25.6, -28.9, -32.4, -36.1,
    -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
    -40.0
  };

  std::vector<double> lmid_p = {
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    5.0, 4.03, 3.1, 2.23, 1.4, 0.62, -0.1, -0.78, -1.4, -1.97,
    -2.5, -2.97, -3.4, -3.77, -4.1, -4.38, -4.6, -4.78, -4.9, -4.97,
    -5.0, -4.97, -4.9, -4.78, -4.6, -4.38, -4.1, -3.77, -3.4, -2.98,
    -2.5, -1.97, -1.4, -0.78, -0.1, 0.62, 1.4, 2.23, 3.1, 4.02,
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    5.0
  };

  std::vector<double> lbtm_p = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 3.9, 7.6, 11.1, 14.4, 17.5, 20.4, 23.1, 25.6, 27.9,
    30.0, 31.9, 33.6, 35.1, 36.4, 37.5, 38.4, 39.1, 39.6, 39.9,
    40.0, 39.9, 39.6, 39.1, 38.4, 37.5, 36.4, 35.1, 33.6, 31.9,
    30.0, 27.9, 25.6, 23.1, 20.4, 17.5, 14.4, 11.1, 7.6, 3.9,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0
  };

  std::vector<double> rtop_p = {
    0.0, 0.1, 0.4, 0.9, 1.6, 2.5, 3.6, 4.9, 6.4, 8.1,
    10.0, 12.1, 14.4, 16.9, 19.6, 22.5, 25.6, 28.9, 32.4, 36.1,
    40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
    40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
    40.0, 40.0, 36.1, 32.4, 28.9, 25.6, 22.5, 19.6, 16.9, 14.4,
    12.1, 10.0, 8.1, 6.4, 4.9, 3.6, 2.5, 1.6, 0.9, 0.4,
    0.1
  };

  std::vector<double> rmid_p = {
    5.0, 4.97, 4.9, 4.78, 4.6, 4.38, 4.1, 3.77, 3.4, 2.98,
    2.5, 1.97, 1.4, 0.78, 0.1, -0.62, -1.4, -2.23, -3.1, -4.02,
    -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
    -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
    -5.0, -5.0, -4.03, -3.1, -2.23, -1.4, -0.62, 0.1, 0.78, 1.4,
    1.97, 2.5, 2.97, 3.4, 3.77, 4.1, 4.38, 4.6, 4.78, 4.9,
    4.97
  };

  std::vector<double> rbtm_p = {
    -40.0, -39.9, -39.6, -39.1, -38.4, -37.5, -36.4, -35.1, -33.6, -31.9,
    -30.0, -27.9, -25.6, -23.1, -20.4, -17.5, -14.4, -11.1, -7.6, -3.9,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -3.9, -7.6, -11.1, -14.4, -17.5, -20.4, -23.1, -25.6,
    -27.9, -30.0, -31.9, -33.6, -35.1, -36.4, -37.5, -38.4, -39.1, -39.6,
    -39.9
  };


  static double r2 = 45;

  // Servo Angle Power
  static double power_th1 = 0.8;
  static double power_phi = 1.0;
  static double power_th2 = 1.0;

  auto angle_len =  r_r1.size();

  for (std::vector<double>::size_type i=0; i != angle_len; i++){
    l_r1.at(i) = l_r1.at(i) * power_th1;
    r_r1.at(i) = r_r1.at(i) * power_th1;

    ltop_p.at(i) = ltop_p.at(i) * power_phi;
    lmid_p.at(i) = lmid_p.at(i) * power_phi;
    lbtm_p.at(i) = lbtm_p.at(i) * power_phi;

    rtop_p.at(i) = rtop_p.at(i) * power_phi;
    rmid_p.at(i) = rmid_p.at(i) * power_phi;
    rbtm_p.at(i) = rbtm_p.at(i) * power_phi;

  }
  // Convert URDF Position parameter
  for (size_t i=0; i != angle_len; i++){
    l_r1.at(i) = l_r1.at(i)/40 * -1;
    r_r1.at(i) = r_r1.at(i)/40 * -1;

    ltop_p.at(i) = ltop_p.at(i)/60;
    lmid_p.at(i) = lmid_p.at(i)/60;
    lbtm_p.at(i) = lbtm_p.at(i)/60;

    rtop_p.at(i) = rtop_p.at(i)/60;
    rmid_p.at(i) = rmid_p.at(i)/60;
    rbtm_p.at(i) = rbtm_p.at(i)/60;
  }

  r2 = r2 * power_th2;
  r2 = r2/60;

  std::vector<std::string> joint_names = {
      "rev_link_Rtop_roll1", "rev_link_Rtop_pitch", "rev_link_Rtop_roll2",
      "rev_link_Rmid_roll1", "rev_link_Rmid_pitch", "rev_link_Rmid_roll2",
      "rev_link_Rbtm_roll1", "rev_link_Rbtm_pitch", "rev_link_Rbtm_roll2",
      "rev_link_Ltop_roll1", "rev_link_Ltop_pitch", "rev_link_Ltop_roll2",
      "rev_link_Lmid_roll1", "rev_link_Lmid_pitch", "rev_link_Lmid_roll2",
      "rev_link_Lbtm_roll1", "rev_link_Lbtm_pitch", "rev_link_Lbtm_roll2",
      "rev_link_head"
    };

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(joint_names.size());
  static double duration_time = 0.02;

  for (size_t i=0; i != angle_len; i++){
    point.time_from_start = rclcpp::Duration::from_seconds(duration_time*i);  // start asap
    point.positions[18] = 0;
    /*
    for (size_t j=0; j != joint_names.size()/3; j++){
        if (j<=2){
          // Right
          point.positions[3*j] = r_r1.at(i);
          point.positions[3*j+2] = -1*r2;

          // Rtop
          if (j == 0){
            point.positions[3*j] = 10;
            point.positions[3*j+1] = -20;

            // point.positions[3*j+1] = -1*rtop_p.at(i);

          }

          // Rmid
          else if (j == 1){
            point.positions[3*j] = -1*l_r1.at(i);
            point.positions[3*j+1] = -1*rmid_p.at(i);
          }
          else{
            point.positions[3*j+1] = -1*rbtm_p.at(i);

          }
        }
        else{
          // Left
          point.positions[3*j] = l_r1.at(i);
          point.positions[3*j+2] = r2;

          // Ltop
          if (j == 3){
            point.positions[3*j] = -10;
            point.positions[3*j+1] = 20;
            // point.positions[3*j+1] = -1*ltop_p.at(i);
          }

          // Lmid
          else if (j == 4){
            point.positions[3*j] = -1*r_r1.at(i);
            point.positions[3*j+1] = -1*lmid_p.at(i);
    
          }

          // Lbtm
          else{
            point.positions[3*j+1] = -1*lbtm_p.at(i);

          }

        }
    }
    */

    for (size_t j=0; j != joint_names.size()/3; j++){
        if (j<=2){
          // Right
          point.positions[3*j] = r_r1.at(i);
          point.positions[3*j+2] = -1*r2;

          if (j == 0){
            point.positions[3*j+1] = -1*rtop_p.at(i);

          }

          else if (j == 1){
            point.positions[3*j] = -1*l_r1.at(i);
            point.positions[3*j+1] = -1*rmid_p.at(i);
          }
          else{
            point.positions[3*j+1] = -1*rbtm_p.at(i);

          }
        }
        else{
          // Left
          point.positions[3*j] = l_r1.at(i);
          point.positions[3*j+2] = r2;

          if (j == 3){
            point.positions[3*j+1] = -1*ltop_p.at(i);
          }
          else if (j == 4){
            point.positions[3*j] = -1*r_r1.at(i);
            point.positions[3*j+1] = -1*lmid_p.at(i);
    
          }
          else{
            point.positions[3*j+1] = -1*lbtm_p.at(i);

          }

        }
    }
    points.push_back(point);
  }

  while(1){
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
    opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
    opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
    opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    goal_msg.trajectory.joint_names = joint_names;
    goal_msg.trajectory.points = points;

    auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);


    if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
      return 1;
    }
    RCLCPP_ERROR(node->get_logger(), "send goal call ok :)");


    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
      goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
      return 1;
    }
    RCLCPP_ERROR(node->get_logger(), "Goal was accepted by server");


    // Wait for the server to be done with the goal
    auto result_future = action_client->async_get_result(goal_handle);
    RCLCPP_INFO(node->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
      return 1;
    } 
  }

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;

  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);


  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    return 1;
  }
  RCLCPP_ERROR(node->get_logger(), "send goal call ok :)");


  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }
  RCLCPP_ERROR(node->get_logger(), "Goal was accepted by server");


  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  } 

  std::cout << "async_send_goal" << std::endl;
  rclcpp::shutdown();

  return 0;
}
