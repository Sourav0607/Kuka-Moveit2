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
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  std::vector<double> arm_joint_goal_;
  std::vector<double> gripper_joint_goal_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const kuka_msgs::action::KukaTask::Goal> goal)
  {
    if (!goal->task_name.empty()) {
      RCLCPP_INFO(get_logger(), "Received goal request with task_name '%s'", goal->task_name.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Received goal request with task_number %d", goal->task_number);
    }
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
    if(gripper_move_group_){
      gripper_move_group_->stop();
    }
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
    if(!gripper_move_group_){
      gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    }
    
    auto result = std::make_shared<kuka_msgs::action::KukaTask::Result>();
    int task_number = goal_handle->get_goal()->task_number;
    std::string task_name = goal_handle->get_goal()->task_name;
    
    // If task_name is provided, map it to task_number
    if (!task_name.empty()) {
      if (task_name == "home") {
        task_number = 0;
      } else if (task_name == "pnp" || task_name == "pick_and_place") {
        task_number = 1;
      } else if (task_name == "place") {
        task_number = 2;
      } else {
        RCLCPP_ERROR(get_logger(), "Unknown task_name: %s", task_name.c_str());
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      RCLCPP_INFO(get_logger(), "Received task_name '%s', mapped to task_number %d", task_name.c_str(), task_number);
    } else {
      RCLCPP_INFO(get_logger(), "Received goal request with task_number %d", task_number);
    }
    
    if (task_number == 0){
      // Task 0: Go to home position
      arm_joint_goal_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
      bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
      
      if(!arm_within_bounds){
        RCLCPP_ERROR(get_logger(), "Target position out of boundaries");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (arm_plan_success){
        RCLCPP_INFO(get_logger(), "Moving to home position");
        arm_move_group_->move();
      }else {
        RCLCPP_ERROR(get_logger(), "Planning failed for home position");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
    }else if (task_number == 1){
      // Task 1: Complete pick sequence
      // Step 1: Move arm to pick position
      RCLCPP_INFO(get_logger(), "Step 1: Moving arm to pick position");
      arm_joint_goal_ = {0.13, -0.57, 1.69, 1.41, 0.0, 0.017};
      
      arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
      bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
      
      if(!arm_within_bounds){
        RCLCPP_ERROR(get_logger(), "Arm target position out of boundaries");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Plan and execute arm movement to pick position
      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      auto plan_result = arm_move_group_->plan(arm_plan);
      
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(get_logger(), "Plan successful, executing...");
        auto execute_result = arm_move_group_->execute(arm_plan);
        
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS){
          RCLCPP_INFO(get_logger(), "Reached pick position");
          // Wait to ensure arm has fully stopped and stabilized
          rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to execute arm movement, error code: %d", execute_result.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan arm movement, error code: %d", plan_result.val);
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Step 2: Close gripper
      RCLCPP_INFO(get_logger(), "Step 2: Closing gripper");
      
      std::map<std::string, double> gripper_closed_map;
      gripper_closed_map["gripper_left_finger_joint"] = 0.06;
      gripper_move_group_->setJointValueTarget(gripper_closed_map);
      
      // Plan and execute gripper close
      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
      auto gripper_plan_result = gripper_move_group_->plan(gripper_plan);
      
      if (gripper_plan_result == moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(get_logger(), "Gripper plan successful, closing...");
        auto gripper_execute_result = gripper_move_group_->execute(gripper_plan);
        
        if (gripper_execute_result == moveit::core::MoveItErrorCode::SUCCESS){
          RCLCPP_INFO(get_logger(), "Gripper closed successfully");
          // Wait for gripper to fully close
          rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to execute gripper close, error code: %d", gripper_execute_result.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan gripper close, error code: %d", gripper_plan_result.val);
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Step 3: Move arm to place position
      RCLCPP_INFO(get_logger(), "Step 3: Moving arm to place position");
      arm_joint_goal_ = {-0.94, -0.24, 0.87, -3.22, -0.38, 2.67};
      
      arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
      arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
      
      if(!arm_within_bounds){
        RCLCPP_ERROR(get_logger(), "Place position out of boundaries");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Plan and execute arm movement to place position
      plan_result = arm_move_group_->plan(arm_plan);
      
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(get_logger(), "Plan successful, moving to place...");
        auto execute_result = arm_move_group_->execute(arm_plan);
        
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS){
          RCLCPP_INFO(get_logger(), "Reached place position");
          // Wait for arm to stabilize
          rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to execute place movement, error code: %d", execute_result.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan place movement, error code: %d", plan_result.val);
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Step 4: Open gripper
      RCLCPP_INFO(get_logger(), "Step 4: Opening gripper");
      
      std::map<std::string, double> gripper_open_map;
      gripper_open_map["gripper_left_finger_joint"] = 0.0;
      gripper_move_group_->setJointValueTarget(gripper_open_map);
      
      // Plan and execute gripper open
      gripper_plan_result = gripper_move_group_->plan(gripper_plan);
      
      if (gripper_plan_result == moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(get_logger(), "Gripper plan successful, opening...");
        auto gripper_execute_result = gripper_move_group_->execute(gripper_plan);
        
        if (gripper_execute_result == moveit::core::MoveItErrorCode::SUCCESS){
          RCLCPP_INFO(get_logger(), "Gripper opened successfully");
          // Wait for gripper to fully open
          rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to execute gripper open, error code: %d", gripper_execute_result.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan gripper open, error code: %d", gripper_plan_result.val);
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Step 5: Return to home position
      RCLCPP_INFO(get_logger(), "Step 5: Returning to home position");
      arm_joint_goal_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
      arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
      
      if(!arm_within_bounds){
        RCLCPP_ERROR(get_logger(), "Home position out of boundaries");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      // Plan and execute arm movement to home
      plan_result = arm_move_group_->plan(arm_plan);
      
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(get_logger(), "Plan successful, returning home...");
        auto execute_result = arm_move_group_->execute(arm_plan);
        
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS){
          RCLCPP_INFO(get_logger(), "Reached home position");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to execute home movement, error code: %d", execute_result.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan home movement, error code: %d", plan_result.val);
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
    }else if (task_number == 2){
      // Task 2: Go to place position only
      arm_joint_goal_ = {-0.94, -0.24, 0.87, -3.22, -0.38, 2.67};
      
      arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
      bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
      
      if(!arm_within_bounds){
        RCLCPP_ERROR(get_logger(), "Target position out of boundaries");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (arm_plan_success){
        RCLCPP_INFO(get_logger(), "Moving to place position");
        arm_move_group_->move();
      }else {
        RCLCPP_ERROR(get_logger(), "Planning failed for place position");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
    }else {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }
};
}  // namespace kuka_remote

RCLCPP_COMPONENTS_REGISTER_NODE(kuka_remote::TaskServer)