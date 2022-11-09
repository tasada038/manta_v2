#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "Manta";
  static const std::string PLANNING_GROUP_LTOP = "Ltop";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_ltop(move_group_node, PLANNING_GROUP_LTOP);
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const moveit::core::JointModelGroup* ltop_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LTOP);

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.5;

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  RCLCPP_INFO(LOGGER, "Joint Names:");
  std::copy(move_group.getJointNames().begin(), move_group.getJointNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_ltop;

  bool success;


  // Motion1 - joint group
  std::vector<double> joint_group_positions;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (size_t i=0; i != joint_group_positions.size()/3; i++){
      if (i<=2){
        joint_group_positions[3*i] = -0.2;
        joint_group_positions[3*i+1] = 0.2;
        joint_group_positions[3*i+2] = 0.2;
      }
      else{
        joint_group_positions[3*i] = 0.2;
        joint_group_positions[3*i+1] = -0.2;
        joint_group_positions[3*i+2] = -0.2;
      }
  }

  move_group.setJointValueTarget(joint_group_positions);
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();


  // Motion2 - ltop group
  std::vector<double> ltop_group_positions;
  moveit::core::RobotStatePtr ltop_current_state = move_group_ltop.getCurrentState();
  ltop_current_state->copyJointGroupPositions(ltop_model_group, ltop_group_positions);
  ltop_group_positions[0] += 0.5;
  move_group_ltop.setJointValueTarget(ltop_group_positions);
  move_group_ltop.setPlanningTime(20.0);

  success = (move_group_ltop.plan(my_plan_ltop) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_ltop.move();
  move_group_ltop.clearPathConstraints();

  // Motion3 - group state
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(move_group.getNamedTargetValues("Init"));
  move_group.setPlanningTime(10.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();

  // Motion4 - group state
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(move_group.getNamedTargetValues("Up"));
  move_group.setPlanningTime(10.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();

  // Motion5 - group state
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(move_group.getNamedTargetValues("Init"));
  move_group.setPlanningTime(10.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();

  rclcpp::shutdown();
  return 0;
}
