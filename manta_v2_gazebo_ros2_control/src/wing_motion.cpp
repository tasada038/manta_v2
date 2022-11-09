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
#include <array>
#include <algorithm>

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

  std::vector<double> mid_th1 = {
    20.74, 18.35, 14.82, 11.12, 6.21, 
    2.26, -2.26, -5.87, -8.49, -10.2, 
    -12.5, -13.08, -12.5, -11.92, -10.2, 
    -8.49, -6.21, -4.51, -1.69, 0.56, 
    4.29, 6.21, 8.83, 11.35, 13.08, 
    14.82, 16.58, 17.76, 18.35, 20.14, 
    21.95, 24.41, 26.28, 28.18, 29.47, 
    30.12, 29.73, 29.21, 27.54, 25.03, 
    21.95
  };

  std::vector<double> mid_th1_shift = {
    11.12, 6.21, 
    2.26, -2.26, -5.87, -8.49, -10.2, 
    -12.5, -13.08, -12.5, -11.92, -10.2, 
    -8.49, -6.21, -4.51, -1.69, 0.56, 
    4.29, 6.21, 8.83, 11.35, 13.08, 
    14.82, 16.58, 17.76, 18.35, 20.14, 
    21.95, 24.41, 26.28, 28.18, 29.47, 
    30.12, 29.73, 29.21, 27.54, 25.03, 
    21.95, 20.74, 18.35, 14.82,
  };

  std::vector<double> mid_th1_shift2 = {
    -2.26, -5.87, -8.49, -10.2, 
    -12.5, -13.08, -12.5, -11.92, -10.2, 
    -8.49, -6.21, -4.51, -1.69, 0.56, 
    4.29, 6.21, 8.83, 11.35, 13.08, 
    14.82, 16.58, 17.76, 18.35, 20.14, 
    21.95, 24.41, 26.28, 28.18, 29.47, 
    30.12, 29.73, 29.21, 27.54, 25.03, 
    21.95, 20.74, 18.35, 14.82, 11.12, 6.21, 2.26,
  };

  std::vector<double> mid_phi = {
    10.5, 14, 16.5, 19.5, 23.5, 24, 25, 26, 27, 26.5,
    26, 25.5, 24, 23, 19.5, 17, 14.5, 12, 8, 6,
    4, 2, 0.5, -4, -5.5, -7, -9, -10, -10.5, -12 ,
    -12.5, -13, -12.5, -10, -8, -6, -4, 1, 4, 6, 8
  };

  std::vector<double> mid_th2 = {
    46.47, 48.18, 50.92, 51.2, 50.5, 
    48.18, 50.92, 38.62, 30.82, 21.88, 
    14.57, 12.83, -2.67, -10.28, -18.09, 
    -25.27, -30.3, -33.98, -35.61,-35.61, 
    -34.3, -30.82, -26.06, -22.36, -15.3, 
    -11.19, -4.01, 0.0, 7.13, 10.28, 
    14.2, 16.97, 19.5, 21.4, 24.78, 
    26.26, 30.1, 33.13, 37.26, 40.7, 
    44.94
  };

  // Servo Angle Power
  static double power_th1 = 1.2;
  static double power_phi = 1.2;
  static double power_th2 = 0.9;

  auto angle_len =  mid_phi.size();

  for (std::vector<double>::size_type i=0; i != angle_len; i++){
    mid_th1.at(i) = mid_th1.at(i) * power_th1;
    mid_th1_shift.at(i) = mid_th1_shift.at(i) * power_th1;
    mid_th1_shift2.at(i) = mid_th1_shift2.at(i) * power_th1;
    mid_phi.at(i) = mid_phi.at(i) * power_phi;
    mid_th2.at(i) = mid_th2.at(i) * power_th2;
  }
  // Convert URDF Position parameter
  for (size_t i=0; i != angle_len; i++){
    mid_th1.at(i) = mid_th1.at(i)/40 * -1;
    mid_th1_shift.at(i) = mid_th1_shift.at(i)/40 * -1;
    mid_th1_shift2.at(i) = mid_th1_shift2.at(i)/40 * -1;
    mid_phi.at(i) = mid_phi.at(i)/60;
    mid_th2.at(i) = mid_th2.at(i)/60;
  }



  //std::rotate_copy(mid_th1.begin(), mid_th1.begin()+shift_t, mid_th1.begin(), mid_th1_shift.begin());
  //std::rotate_copy(mid_th1.begin(), mid_th1.begin()+shift_t*2, mid_th1.begin(), mid_th1_shift2.begin());

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
  static double duration_time = 0.08;

  
  for (size_t i=0; i != angle_len; i++){
    point.time_from_start = rclcpp::Duration::from_seconds(duration_time*i);  // start asap
    for (size_t j=0; j != joint_names.size()/3; j++){

        if (j<=2){
          point.positions[3*j+1] = mid_phi.at(i);
          point.positions[3*j+2] = mid_th2.at(i);
          if (j==0){
            point.positions[3*j] = mid_th1.at(i);
          }
          else if (j==1){
            point.positions[3*j] = mid_th1.at(i); 
          }
          else {
            point.positions[3*j] = mid_th1.at(i);          
          }       
        }
        else{
          point.positions[3*j+1] = -1*mid_phi.at(i);
          point.positions[3*j+2] = -1*mid_th2.at(i);
          if (j==3){
            point.positions[3*j] = -1*mid_th1.at(i);
          }
          else if (j==4){
            point.positions[3*j] = -1*mid_th1.at(i);
          }
          else {
            point.positions[3*j] = -1*mid_th1.at(i);
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
