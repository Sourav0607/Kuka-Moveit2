#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "kuka_msgs/action/kuka_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>


#include <memory>
#include <thread>


using namespace std::placeholders;

namespace kuka_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<kuka_msgs::action::KukaTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<kuka_msgs::action::KukaTask>::SharedPtr action_server_;
  // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  // std::vector<double> arm_joint_goal_, gripper_joint_goal_;
  std::vector<double> arm_joint_goal_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const kuka_msgs::action::KukaTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with task_number %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<kuka_msgs::action::KukaTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(arm_move_group_){
      arm_move_group_->stop();
    }
    // if(gripper_move_group_){
    //   gripper_move_group_->stop();
    // }
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<kuka_msgs::action::KukaTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kuka_msgs::action::KukaTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    if(!arm_move_group_){
      arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    }
    // if(!gripper_move_group_){
    //   gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    // }
    auto result = std::make_shared<kuka_msgs::action::KukaTask::Result>();
    if (goal_handle->get_goal()->task_number==0){
      arm_joint_goal_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // gripper_joint_goal_ = {-0.7, -0.7};
    }else if (goal_handle->get_goal()->task_number==1){
      arm_joint_goal_ = {1.34, -1.34, 1.29, 2.77, -0.47, 0.0};
      // gripper_joint_goal_ = {-0.7, -0.7};
    }else if (goal_handle->get_goal()->task_number==2){
      arm_joint_goal_ = {-0.94, -0.24, 0.87, -3.22, -0.38, 2.67};
      // gripper_joint_goal_ = {-0.7, -0.7};
    }else {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      return;
    }

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    // gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

    bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
    // bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);

    if(!arm_within_bounds){
      RCLCPP_ERROR(get_logger(), "Target position out of boundaries");
      return;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan;
    bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan)== moveit::core::MoveItErrorCode::SUCCESS);

    if (arm_plan_success){
      arm_move_group_-> move();
      // gripper_move_group_->move();

    }else {
      RCLCPP_ERROR(get_logger(), "One or more planner failed");
      return;
    }

    result-> success = true;
    goal_handle->succeed(result);
  }
};
}  // namespace arduinobot_cpp_examples

RCLCPP_COMPONENTS_REGISTER_NODE(kuka_remote::TaskServer)