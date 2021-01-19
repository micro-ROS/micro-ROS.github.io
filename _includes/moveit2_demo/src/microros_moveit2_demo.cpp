#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/empty.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("microros_moveit2_demo");

class MoveItNode
{
public:
  MoveItNode(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    arm_ = std::make_shared<moveit::planning_interface::PlanningComponent>("panda_arm", moveit_cpp_);

    // Init micro-ROS subscribers
    auto trigger_callback = [this](const std_msgs::msg::Empty::SharedPtr msg) -> void
    {
      geometry_msgs::msg::PoseStamped goal_pose = last_pose_;

      double mod = sqrt( 
         goal_pose.pose.orientation.x * goal_pose.pose.orientation.x + 
         goal_pose.pose.orientation.y * goal_pose.pose.orientation.y +
         goal_pose.pose.orientation.z * goal_pose.pose.orientation.z +
         goal_pose.pose.orientation.w * goal_pose.pose.orientation.w  
      );

      goal_pose.pose.orientation.x *= 1/mod;
      goal_pose.pose.orientation.y *= 1/mod;
      goal_pose.pose.orientation.z *= 1/mod;
      goal_pose.pose.orientation.w *= 1/mod;

      arm_->setGoal(goal_pose, "panda_link8");

      // Run actual plan
      RCLCPP_INFO(LOGGER, "Plan to goal: %f %f %f", goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z);
      const auto plan_solution = arm_->plan();
      if (plan_solution)
      {
        RCLCPP_INFO(LOGGER, "arm.execute()");
        arm_->execute();
      }
    };

    auto tf_callback = [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) -> void
    {
      if(msg->transforms[0].child_frame_id == "/inertial_unit"){
        last_pose_.pose.orientation.w = msg->transforms[0].transform.rotation.w;
        last_pose_.pose.orientation.x = msg->transforms[0].transform.rotation.x;
        last_pose_.pose.orientation.y = msg->transforms[0].transform.rotation.y;
        last_pose_.pose.orientation.z = msg->transforms[0].transform.rotation.z;
        
        last_pose_.pose.position.x = msg->transforms[0].transform.translation.x;
        last_pose_.pose.position.y = msg->transforms[0].transform.translation.y;
        last_pose_.pose.position.z = msg->transforms[0].transform.translation.z;
      }
    };

    trigger_sub_ = node_->create_subscription<std_msgs::msg::Empty>("trigger_moveit2", rclcpp::QoS(rclcpp::KeepLast(1)), trigger_callback);
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(rclcpp::KeepLast(1)), tf_callback);
    last_pose_.header.frame_id = "panda_link0";
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  moveit::planning_interface::PlanningComponentPtr arm_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

  geometry_msgs::msg::PoseStamped last_pose_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_node", "", node_options);

  MoveItNode microros_demo(node);
  std::thread run_demo([&microros_demo]() {
    microros_demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
