/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_cpp_interface_demo.cpp
 *      Project   : moveit_servo
 *      Created   : 07/13/2020
 *      Author    : Adam Pettinger
 */

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/empty.hpp>

// Servo
#include <moveit_servo/servo_parameters.cpp>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_demo_node.cpp");

class ServoCppDemo
{
public:
  ServoCppDemo(rclcpp::Node::SharedPtr node) : node_(node), movement_mode_jog_(true)
  {
    // Create the planning_scene_monitor
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", tf_buffer_, "planning_scene_monitor");

    // Get the planning_scene_monitor to publish scene diff's for RViz visualization
    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/moveit_servo/publish_planning_scene");
      planning_scene_monitor_->startSceneMonitor();
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    }

    // Init micro-ROS subscribers
    auto trigger_callback = [this](const std_msgs::msg::Empty::SharedPtr msg) -> void
    {
      RCLCPP_INFO(LOGGER, "Changing mode");
      movement_mode_jog_ = !movement_mode_jog_;
    };

    auto tf_callback = [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) -> void
    {
      if(msg->transforms[0].child_frame_id == "/inertial_unit"){
        if(movement_mode_jog_){
          RCLCPP_INFO(LOGGER, "jog mode - TF: %f %f %f",msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z);
          auto msg_out = std::make_unique<control_msgs::msg::JointJog>();
          msg_out->header.stamp = node_->now();
          msg_out->joint_names.push_back("panda_joint1");
          msg_out->velocities.push_back(msg->transforms[0].transform.rotation.x*5);
          msg_out->joint_names.push_back("panda_joint6");
          msg_out->velocities.push_back(msg->transforms[0].transform.rotation.y*5);
          joint_cmd_pub_->publish(std::move(msg_out));
        }else{
          RCLCPP_INFO(LOGGER, "twist mode - TF: %f %f %f",msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z);
          auto msg_out = std::make_unique<geometry_msgs::msg::TwistStamped>();
          msg_out->header.stamp = node_->now();
          msg_out->header.frame_id = "panda_link0";
          msg_out->twist.linear.x = msg->transforms[0].transform.rotation.y;
          msg_out->twist.linear.y = msg->transforms[0].transform.rotation.x;
          twist_cmd_pub_->publish(std::move(msg_out));
        }
      }
    };

    trigger_sub_ = node_->create_subscription<std_msgs::msg::Empty>("trigger_moveit2", rclcpp::QoS(rclcpp::KeepLast(1)), trigger_callback);
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(rclcpp::KeepLast(1)), tf_callback);

    // We need 2 different publishers for the different command types
    joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_server/delta_joint_cmds", 10);
    twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_server/delta_twist_cmds", 10);
  }

  void start()
  {
    // Get Servo Parameters
    auto servo_parameters = std::make_shared<moveit_servo::ServoParameters>();
    if (!moveit_servo::readParameters(servo_parameters, node_, LOGGER))
    {
      RCLCPP_ERROR(LOGGER, "Could not get parameters");
    }

    // Create Servo and start it
    servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor_);
    while (!servo_->waitForInitialized() && rclcpp::ok())
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, 5000, "Waiting for ServoCalcs to recieve joint states");
    }
    servo_->start();
  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

  bool  movement_mode_jog_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::unique_ptr<moveit_servo::Servo> servo_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up..
  rclcpp::sleep_for(std::chrono::seconds(4));

  // We need to initialize the planning_scene_monitor here before publishing the collision object
  ServoCppDemo demo(node);

  // Create collision object, in the way of servoing
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.6;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);

  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the collision object to the planning scene
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  scene_pub->publish(ps);

  // Start the Servo object, and start publishing commands to it
  demo.start();

  // Spin
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
