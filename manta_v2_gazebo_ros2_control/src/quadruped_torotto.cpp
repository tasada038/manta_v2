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
  // convert_pyyaml_tripod.py
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
      -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, 
      -20.0, -21.95, -23.8, -25.55, -27.2, -28.75, -30.2, -31.55, -32.8, -33.95, 
      -35.0, -35.95, -36.8, -37.55, -38.2, -38.75, -39.2, -39.55, -39.8, -39.95, 
      -40.0, -39.95, -39.8, -39.55, -39.2, -38.75, -38.2, -37.55, -36.8, -35.95, 
      -35.0, -33.95, -32.8, -31.55, -30.2, -28.75, -27.2, -25.55, -23.8, -21.95, 
      -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, 
      -20.0
  };

  std::vector<double> lmid_p = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
      0.0, 1.07, 2.26, 3.55, 4.9, 6.29, 7.7, 9.11, 10.49, 11.84, 
      13.12, 14.33, 15.45, 16.46, 17.36, 18.14, 18.78, 19.29, 19.65, 19.86, 
      19.92, 19.83, 19.58, 19.19, 18.65, 17.97, 17.15, 16.21, 15.15, 13.98, 
      12.72, 11.38, 9.97, 8.52, 7.03, 5.54, 4.06, 2.61, 1.22, -0.09, -1.28,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
      0.0
  };

  std::vector<double> lbtm_p = {
      30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 
      30.0, 28.05, 26.2, 24.45, 22.8, 21.25, 19.8, 18.45, 17.2, 16.05, 
      15.0, 14.05, 13.2, 12.45, 11.8, 11.25, 10.8, 10.45, 10.2, 10.05, 
      10.0, 10.05, 10.2, 10.45, 10.8, 11.25, 11.8, 12.45, 13.2, 14.05, 
      15.0, 16.05, 17.2, 18.45, 19.8, 21.25, 22.8, 24.45, 26.2, 28.05, 
      30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 
      30.0
  };

  std::vector<double> rtop_p = {
      40.0, 39.95, 39.8, 39.55, 39.2, 38.75, 38.2, 37.55, 36.8, 35.95, 
      35.0, 33.95, 32.8, 31.55, 30.2, 28.75, 27.2, 25.55, 23.8, 21.95, 
      20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 
      20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 
      20.0, 20.0, 21.95, 23.8, 25.55, 27.2, 28.75, 30.2, 31.55, 32.8, 
      33.95, 35.0, 35.95, 36.8, 37.55, 38.2, 38.75, 39.2, 39.55, 39.8, 
      39.95 
  };

  std::vector<double> rmid_p = {
      -20.0, -19.95, -19.8, -19.55, -19.2, -18.75, -18.2, -17.55, -16.8, -15.95, 
      -15.0, -13.95, -12.8, -11.55, -10.2, -8.75, -7.2, -5.55, -3.8, -1.95, 
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
      0.0, 0.0, -1.95, -3.8, -5.55, -7.2, -8.75, -10.2, -11.55, -12.8, 
      -13.95, -15.0, -15.95, -16.8, -17.55, -18.2, -18.75, -19.2, -19.55, -19.8, 
      -19.95
  };

  std::vector<double> rbtm_p = {
      -10.0, -10.05, -10.2, -10.45, -10.8, -11.25, -11.8, -12.45, -13.2, -14.05, 
      -15.0, -16.05, -17.2, -18.45, -19.8, -21.25, -22.8, -24.45, -26.2, -28.05, 
      -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 
      -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 
      -30.0, -30.0, -28.05, -26.2, -24.45, -22.8, -21.25, -19.8, -18.45, -17.2, 
      -16.05, -15.0, -14.05, -13.2, -12.45, -11.8, -11.25, -10.8, -10.45, -10.2, 
      -10.05
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -3.9, -7.6, -11.1, -14.4, -17.5, -20.4, -23.1, -25.6, -27.9,
    -30.0, -31.9, -33.6, -35.1, -36.4, -37.5, -38.4, -39.1, -39.6, -39.9,
    -40.0, -39.9, -39.6, -39.1, -38.4, -37.5, -36.4, -35.1, -33.6, -31.9,
    -30.0, -27.9, -25.6, -23.1, -20.4, -17.5, -14.4, -11.1, -7.6, -3.9,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0
  };

  std::vector<double> lmid_p = {
    -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
    -5.0, -4.03, -3.1, -2.23, -1.4, -0.62, 0.1, 0.78, 1.4, 1.97,
    2.5, 2.97, 3.4, 3.77, 4.1, 4.38, 4.6, 4.78, 4.9, 4.97,
    5.0, 4.97, 4.9, 4.78, 4.6, 4.38, 4.1, 3.77, 3.4, 2.98,
    2.5, 1.97, 1.4, 0.78, 0.1, -0.62, -1.4, -2.23, -3.1, -4.02,
    -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
    -5.0
  };

  std::vector<double> lbtm_p = {
    40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
    40.0, 36.1, 32.4, 28.9, 25.6, 22.5, 19.6, 16.9, 14.4, 12.1,
    10.0, 8.1, 6.4, 4.9, 3.6, 2.5, 1.6, 0.9, 0.4, 0.1,
    0.0, 0.1, 0.4, 0.9, 1.6, 2.5, 3.6, 4.9, 6.4, 8.1,
    10.0, 12.1, 14.4, 16.9, 19.6, 22.5, 25.6, 28.9, 32.4, 36.1,
    40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
    40.0
  };

  std::vector<double> rtop_p = {
    40.0, 39.9, 39.6, 39.1, 38.4, 37.5, 36.4, 35.1, 33.6, 31.9,
    30.0, 27.9, 25.6, 23.1, 20.4, 17.5, 14.4, 11.1, 7.6, 3.9,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 3.9, 7.6, 11.1, 14.4, 17.5, 20.4, 23.1, 25.6,
    27.9, 30.0, 31.9, 33.6, 35.1, 36.4, 37.5, 38.4, 39.1, 39.6,
    39.9
  };

  std::vector<double> rmid_p = {
    -5.0, -4.97, -4.9, -4.78, -4.6, -4.38, -4.1, -3.77, -3.4, -2.98,
    -2.5, -1.97, -1.4, -0.78, -0.1, 0.62, 1.4, 2.23, 3.1, 4.02,
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 4.03, 3.1, 2.23, 1.4, 0.62, -0.1, -0.78, -1.4,
    -1.97, -2.5, -2.97, -3.4, -3.77, -4.1, -4.38, -4.6, -4.78, -4.9,
    -4.97
  };

  std::vector<double> rbtm_p = {
    0.0, -0.35, -0.91, -1.67, -2.62, -3.77, -5.09, -6.59, -8.26, -10.09,
    -12.07, -14.2, -16.47, -18.87, -21.41, -24.06, -26.84, -29.72, -32.72, -35.83,
    -39.04, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
    -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
    -40.0, -40.0, -36.07, -32.29, -28.67, -25.22, -21.97, -18.92, -16.08, -13.46,
    -11.06, -8.89, -6.95, -5.25, -3.78, -2.55, -1.55, -0.79, -0.25, 0.05,
    0.14
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
    for (size_t j=0; j != joint_names.size()/3; j++){
        if (j<=2){
          // Right
          point.positions[3*j] = r_r1.at(i);
          point.positions[3*j+2] = -1*r2;

          if (j == 0){
            point.positions[3*j+1] = -1*rtop_p.at(i);
          }

          else if (j == 1){
            point.positions[3*j] = 0.0;
            point.positions[3*j+1] = 0.0;
            point.positions[3*j+2] = 0.0;
          }
          else{
            point.positions[3*j] = -1*l_r1.at(i);
            point.positions[3*j+1] = lbtm_p.at(i);

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
            point.positions[3*j] = 0.0;
            point.positions[3*j+1] = 0.0;
            point.positions[3*j+2] = 0.0;
          }
          else{
            point.positions[3*j] = -1*r_r1.at(i);
            point.positions[3*j+1] = rbtm_p.at(i);

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
