#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <termios.h>
#include <unistd.h>

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
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    
    // Subscribe directly to joint_states as a backup
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = msg;
      });
    
    RCLCPP_INFO(get_logger(), "Waiting for joint states...");
    // Give some time for the first joint state message
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    run();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<std::vector<double>> saved_positions_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

  char getKey()
  {
    struct termios oldt, newt;
    char c;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
  }

  void run()
  {
    RCLCPP_INFO(get_logger(), "Manual Teach Controls:");
    RCLCPP_INFO(get_logger(), "  's' - Save current joint state");
    RCLCPP_INFO(get_logger(), "  'm' - Manually enter joint values");
    RCLCPP_INFO(get_logger(), "  'space' - Execute all saved poses");
    RCLCPP_INFO(get_logger(), "  'l' - List saved poses");
    RCLCPP_INFO(get_logger(), "  'c' - Clear all saved poses");
    RCLCPP_INFO(get_logger(), "  'q' - Quit");
    
    while (rclcpp::ok())
    {
      // Spin to receive joint_states messages
      rclcpp::spin_some(this->shared_from_this());
      
      char key = getKey();

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
    
    std::vector<double> current_joints;
    
    // First try to use our direct subscription (works even with bad timestamps)
    if (latest_joint_state_ && latest_joint_state_->position.size() == 6)
    {
      current_joints = latest_joint_state_->position;
      RCLCPP_INFO(get_logger(), "Got joint state from direct subscription");
    }
    else
    {
      // Fallback: Try to get from MoveIt's current state monitor
      moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(3.0);
      
      if (current_state)
      {
        current_state->copyJointGroupPositions(move_group_->getName(), current_joints);
        RCLCPP_INFO(get_logger(), "Got joint state from MoveIt");
      }
    }
    
    // Check if we got valid joint values
    if (current_joints.empty() || current_joints.size() != 6)
    {
      RCLCPP_WARN(get_logger(), "Cannot read current joint state from robot.");
      RCLCPP_INFO(get_logger(), "Make sure /joint_states topic is publishing. Use 'm' to manually enter values.");
      return;
    }
    
    saved_positions_.push_back(current_joints);
    RCLCPP_INFO(get_logger(), "Saved pose #%zu with joint values:", saved_positions_.size());
    printJointValues(current_joints);
  }

  void saveManualPose()
  {
    std::cout << "\nEnter 6 joint values (in radians, separated by spaces):" << std::endl;
    std::cout << "Example: 0.0 -0.5 1.0 0.0 0.5 0.0" << std::endl;
    std::cout << "> ";
    
    // Temporarily restore terminal settings for input
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    std::vector<double> joints(6);
    for (int i = 0; i < 6; i++)
    {
      std::cin >> joints[i];
    }
    std::cin.ignore();
    
    // Restore non-canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
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
    
    for (size_t i = 0; i < saved_positions_.size(); ++i)
    {
      RCLCPP_INFO(get_logger(), "Executing pose #%zu", i + 1);
      
      // Set the target
      move_group_->setJointValueTarget(saved_positions_[i]);
      
      // Increase tolerances to make execution more robust
      move_group_->setGoalJointTolerance(0.01);  // 0.01 radians tolerance
      move_group_->setGoalPositionTolerance(0.01);
      move_group_->setGoalOrientationTolerance(0.01);
      
      // Plan the motion
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (plan_success)
      {
        RCLCPP_INFO(get_logger(), "Plan succeeded, executing...");
        
        // Execute with asyncExecute and check result
        auto result = move_group_->execute(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(get_logger(), " Pose #%zu executed successfully", i + 1);
        }
        else
        {
          RCLCPP_WARN(get_logger(), " Pose #%zu execution aborted or failed", i + 1);
        }
        
        // Wait a bit between poses to stabilize
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualTeachNode>();
  node->initialize();
  rclcpp::shutdown();
  return 0;
}
