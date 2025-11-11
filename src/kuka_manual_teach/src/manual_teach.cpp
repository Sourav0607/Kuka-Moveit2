// This program lets you teach a robot by saving poses and replaying them
// You can move the robot in RViz and press 's' to save positions

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <termios.h>  // For keyboard input without pressing Enter
#include <unistd.h>
#include <thread>
#include <chrono>

class ManualTeachNode : public rclcpp::Node
{
public:
  ManualTeachNode()
  : Node("manual_teach")
  {
    RCLCPP_INFO(get_logger(), "Manual Teach Node Started");
  }

  void initialize()
  {
    // Connect to MoveIt for the "arm" planning group
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    
    // Subscribe to joint_states topic to read current robot position
    // This runs automatically whenever new joint data arrives
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = msg;  // Save the latest position
      });
    
    RCLCPP_INFO(get_logger(), "Waiting for joint states...");
    // Wait a bit to make sure we get at least one joint state message
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    run();
  }

private:
  // MoveIt interface to control the robot
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // Storage for all the poses we save
  std::vector<std::vector<double>> saved_positions_;
  
  // Subscription to get robot joint positions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  // The most recent joint state we received
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

  // This function reads a single keypress without waiting for Enter
  char getKey()
  {
    struct termios oldt, newt;
    char c;
    tcgetattr(STDIN_FILENO, &oldt);  // Save current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  // Turn off line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Apply new settings
    c = getchar();  // Read one character
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old settings
    return c;
  }

  void run()
  {
    // Show the user what keys do what
    RCLCPP_INFO(get_logger(), "Manual Teach Controls:");
    RCLCPP_INFO(get_logger(), "  's' - Save current joint state");
    RCLCPP_INFO(get_logger(), "  'm' - Manually enter joint values");
    RCLCPP_INFO(get_logger(), "  'space' - Execute all saved poses");
    RCLCPP_INFO(get_logger(), "  'l' - List saved poses");
    RCLCPP_INFO(get_logger(), "  'c' - Clear all saved poses");
    RCLCPP_INFO(get_logger(), "  'q' - Quit");
    
    // Main loop - keep running until user quits
    while (rclcpp::ok())
    {
      // Process any incoming messages (like joint states)
      rclcpp::spin_some(this->shared_from_this());
      
      // Wait for user to press a key
      char key = getKey();

      // Do different things based on which key was pressed
      if (key == 's')
      {
        saveCurrentPose();
      }
      else if (key == 'm')
      {
        saveManualPose();
      }
      else if (key == ' ')
      {
        executeSavedPoses();
      }
      else if (key == 'l')
      {
        listSavedPoses();
      }
      else if (key == 'c')
      {
        clearPoses();
      }
      else if (key == 'q')
      {
        RCLCPP_INFO(get_logger(), "Quitting manual teach...");
        break;
      }
    }
  }

  void saveCurrentPose()
  {
    RCLCPP_INFO(get_logger(), "Reading current joint state...");
    
    // Spin a few times to make sure we get the latest joint state
    // This fixes the issue where pressing 's' too fast would save old positions
    for (int i = 0; i < 10; ++i)
    {
      rclcpp::spin_some(this->shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::vector<double> current_joints;
    
    // Try to get joint positions from our direct subscription first
    // This works even when timestamps are weird in simulation
    if (latest_joint_state_ && latest_joint_state_->position.size() == 6)
    {
      current_joints = latest_joint_state_->position;
      RCLCPP_INFO(get_logger(), "Got joint state from direct subscription");
    }
    else
    {
      // Backup plan: ask MoveIt for the current position
      // This is slower but more reliable in some cases
      moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(3.0);
      
      if (current_state)
      {
        current_state->copyJointGroupPositions(move_group_->getName(), current_joints);
        RCLCPP_INFO(get_logger(), "Got joint state from MoveIt");
      }
    }
    
    // Make sure we actually got valid joint values
    if (current_joints.empty() || current_joints.size() != 6)
    {
      RCLCPP_WARN(get_logger(), "Cannot read current joint state from robot.");
      RCLCPP_INFO(get_logger(), "Make sure /joint_states topic is publishing. Use 'm' to manually enter values.");
      return;
    }
    
    // Add this pose to our list
    saved_positions_.push_back(current_joints);
    RCLCPP_INFO(get_logger(), "Saved pose #%zu with joint values:", saved_positions_.size());
    printJointValues(current_joints);
  }

  void saveManualPose()
  {
    std::cout << "\nEnter 6 joint values (in radians, separated by spaces):" << std::endl;
    std::cout << "Example: 0.0 -0.5 1.0 0.0 0.5 0.0" << std::endl;
    std::cout << "> ";
    
    // Temporarily allow normal typing so user can enter numbers
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag |= (ICANON | ECHO);  // Turn back on line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    // Read 6 numbers from user
    std::vector<double> joints(6);
    for (int i = 0; i < 6; i++)
    {
      std::cin >> joints[i];
    }
    std::cin.ignore();  // Clear the enter key
    
    // Go back to single-keypress mode
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    // Save the manually entered pose
    saved_positions_.push_back(joints);
    RCLCPP_INFO(get_logger(), "Manually saved pose #%zu:", saved_positions_.size());
    printJointValues(joints);
    std::cout << std::endl;
  }

  void listSavedPoses()
  {
    if (saved_positions_.empty())
    {
      RCLCPP_INFO(get_logger(), "No poses saved yet!");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Saved poses (%zu total):", saved_positions_.size());
    for (size_t i = 0; i < saved_positions_.size(); ++i)
    {
      std::cout << "  Pose #" << (i + 1) << ": ";
      printJointValues(saved_positions_[i]);
    }
  }

  void clearPoses()
  {
    saved_positions_.clear();
    RCLCPP_INFO(get_logger(), "All saved poses cleared!");
  }

  // Helper function to print joint angles nicely
  void printJointValues(const std::vector<double>& joints)
  {
    if (joints.empty())
    {
      std::cout << "[]" << std::endl;
      return;
    }
    
    std::cout << "[";
    for (size_t i = 0; i < joints.size(); ++i)
    {
      std::cout << std::fixed << std::setprecision(3) << joints[i];
      if (i < joints.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }

  void executeSavedPoses()
  {
    if (saved_positions_.empty())
    {
      RCLCPP_WARN(get_logger(), "No poses saved yet!");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing %zu saved poses...", saved_positions_.size());
    
    // Go through each saved pose one by one
    for (size_t i = 0; i < saved_positions_.size(); ++i)
    {
      RCLCPP_INFO(get_logger(), "Executing pose #%zu", i + 1);
      
      // Tell MoveIt where we want to go
      move_group_->setJointValueTarget(saved_positions_[i]);
      
      // Set tolerances - how close is "close enough"
      // 0.01 radians is about 0.57 degrees
      // Without this, execution might fail due to tiny errors
      move_group_->setGoalJointTolerance(0.01);
      move_group_->setGoalPositionTolerance(0.01);
      move_group_->setGoalOrientationTolerance(0.01);
      
      // Ask MoveIt to plan a path to the target
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (plan_success)
      {
        RCLCPP_INFO(get_logger(), "Plan succeeded, executing...");
        
        // Execute the planned motion
        auto result = move_group_->execute(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(get_logger(), " Pose #%zu executed successfully", i + 1);
        }
        else
        {
          RCLCPP_WARN(get_logger(), " Pose #%zu execution aborted or failed", i + 1);
        }
        
        // Wait a bit between poses to let the robot stabilize
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
      }
      else
      {
        RCLCPP_ERROR(get_logger(), " Planning failed for pose #%zu", i + 1);
      }
    }
    
    RCLCPP_INFO(get_logger(), "Finished executing all poses");
  }
};

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Create our manual teach node
  auto node = std::make_shared<ManualTeachNode>();
  
  // Start the interactive teaching interface
  node->initialize();
  
  // Clean up when done
  rclcpp::shutdown();
  return 0;
}
