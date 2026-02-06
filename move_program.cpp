#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "custom_interface/msg/cust.hpp"
#include "custom_interface/msg/bool_flag.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class UR5eMotionNode : public rclcpp::Node
{
public:
  UR5eMotionNode()
  : Node("ur5e_motion_node"), shape_received_(false)
  {
    move_service_ = this->create_service<example_interfaces::srv::SetBool>(
      "move_to_pose",
      std::bind(&UR5eMotionNode::move_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_home_1_service_ = this->create_service<example_interfaces::srv::SetBool>(
      "reset_home_1",
      std::bind(&UR5eMotionNode::reset_home_1_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_home_2_service_ = this->create_service<example_interfaces::srv::SetBool>(
      "reset_home_2",
      std::bind(&UR5eMotionNode::reset_home_2_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_home_3_service_ = this->create_service<example_interfaces::srv::SetBool>(
      "reset_home_3",
      std::bind(&UR5eMotionNode::reset_home_3_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_home_4_service_ = this->create_service<example_interfaces::srv::SetBool>(
      "reset_home_4",
      std::bind(&UR5eMotionNode::reset_home_4_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_home_5_service_ = this->create_service<example_interfaces::srv::SetBool>(
      "reset_home_5",
      std::bind(&UR5eMotionNode::reset_home_5_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    shape_sub_ = this->create_subscription<custom_interface::msg::Cust>(
      "shape_detected_topic", 10,
      std::bind(&UR5eMotionNode::shape_callback, this, std::placeholders::_1)
    );

    flag_pub_ = this->create_publisher<custom_interface::msg::BoolFlag>("robot_status_topic", 10);
    timer_ = this->create_wall_timer(10s, std::bind(&UR5eMotionNode::flag_timer_callback, this));

    last_shape_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "✅ UR5e Motion Node initialized");
  }

private:
  void shape_callback(const custom_interface::msg::Cust::SharedPtr msg)
  {
    auto now = this->get_clock()->now();
    double seconds_since_last = (now - last_shape_time_).seconds();

    if (seconds_since_last >= 10.0) {
      shape_ = msg->shape;
      shape_x_ = msg->x;
      shape_y_ = msg->y;
      shape_z_ = msg->z;

      shape_received_ = true;
      last_shape_time_ = now;

      RCLCPP_INFO(this->get_logger(), "✅ New shape received: %s", shape_.c_str());
      RCLCPP_INFO(this->get_logger(), "📍 Coordinates: X=%.3f, Y=%.3f, Z=%.3f", shape_x_, shape_y_, shape_z_);
    } else {
      shape_received_ = false;
      RCLCPP_INFO(this->get_logger(), "⏳ Shape ignored due to throttling.");
    }
  }

  void flag_timer_callback()
  {
    custom_interface::msg::BoolFlag flag_msg;
    flag_msg.flag = shape_received_;
    flag_pub_->publish(flag_msg);

    RCLCPP_INFO(this->get_logger(), "🔁 Published BoolFlag: %s", shape_received_ ? "True" : "False");
    shape_received_ = false;
  }

  void move_callback(
    const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data && !shape_.empty()) {
      RCLCPP_INFO(this->get_logger(), "⚙️ Starting MoveIt plan for shape: %s", shape_.c_str());

      moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "ur_manipulator");

      geometry_msgs::msg::Pose goal_pose;
      tf2::Quaternion quat;
      quat.setRPY(0, 0, 0);
      goal_pose.orientation = tf2::toMsg(quat);

      goal_pose.position.x = shape_x_;
      goal_pose.position.y = shape_y_;
      goal_pose.position.z = shape_z_;

      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(goal_pose);
      move_group.setPlanningTime(10.0);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(this->get_logger(), "✅ Motion executed.");
        response->success = true;
        response->message = "Motion executed.";
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Motion planning failed.");
        response->success = false;
        response->message = "Motion planning failed.";
      }
    } else {
      response->success = false;
      response->message = "No shape received or request denied.";
    }
  }

  void reset_home_1_callback(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                             std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data) {
      moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "ur_manipulator");
      geometry_msgs::msg::Pose pose;
      pose.position.x = -0.110;
      pose.position.y = 0.270;
      pose.position.z = 1.032;
      pose.orientation.x = -0.650;
      pose.orientation.y = -0.305;
      pose.orientation.z = 0.298;
      pose.orientation.w = 0.628;

      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(pose);
      move_group.setPlanningTime(10.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        response->success = true;
        response->message = "Robot reset to home 1 position.";
      } else {
        response->success = false;
        response->message = "Reset motion planning failed.";
      }
    }
  }

  void reset_home_2_callback(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                             std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data) {
      moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "ur_manipulator");
      geometry_msgs::msg::Pose pose;
      pose.position.x = -0.162;
      pose.position.y = 0.039;
      pose.position.z = 1.065;
      pose.orientation.x = -0.290;
      pose.orientation.y = -0.622;
      pose.orientation.z = 0.662;
      pose.orientation.w = 0.302;

      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(pose);
      move_group.setPlanningTime(10.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        response->success = true;
        response->message = "Robot reset to home 2 position.";
      } else {
        response->success = false;
        response->message = "Reset motion planning failed.";
      }
    }
  }

  void reset_home_3_callback(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                             std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data) {
      moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "ur_manipulator");
      geometry_msgs::msg::Pose pose;
      pose.position.x = -0.162;
      pose.position.y = 0.039;
      pose.position.z = 0.887;
      pose.orientation.x = -0.290;
      pose.orientation.y = -0.622;
      pose.orientation.z = 0.662;
      pose.orientation.w = 0.302;

      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(pose);
      move_group.setPlanningTime(10.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        response->success = true;
        response->message = "Robot reset to home 3 position.";
      } else {
        response->success = false;
        response->message = "Reset motion planning failed.";
      }
    }
  }

  void reset_home_4_callback(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                             std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data) {
      moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "ur_manipulator");
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.000;
      pose.position.y = 0.575;
      pose.position.z = 1.051;
      pose.orientation.x = -0.325;
      pose.orientation.y = 0.630;
      pose.orientation.z = -0.625;
      pose.orientation.w = 0.327;

      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(pose);
      move_group.setPlanningTime(10.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        response->success = true;
        response->message = "Robot reset to home 4 position.";
      } else {
        response->success = false;
        response->message = "Reset motion planning failed.";
      }
    }
  }

  void reset_home_5_callback(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                             std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data) {
      moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "ur_manipulator");
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.125;
      pose.position.y = 0.243;
      pose.position.z = 1.033;
      pose.orientation.x = -0.722;
      pose.orientation.y = -0.013;
      pose.orientation.z = 0.015;
      pose.orientation.w = 0.691;

      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(pose);
      move_group.setPlanningTime(10.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        response->success = true;
        response->message = "Robot reset to home 5 position.";
      } else {
        response->success = false;
        response->message = "Reset motion planning failed.";
      }
    }
  }

  rclcpp::Subscription<custom_interface::msg::Cust>::SharedPtr shape_sub_;
  rclcpp::Publisher<custom_interface::msg::BoolFlag>::SharedPtr flag_pub_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr move_service_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_home_1_service_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_home_2_service_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_home_3_service_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_home_4_service_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_home_5_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_shape_time_;
  std::string shape_;
  double shape_x_, shape_y_, shape_z_;
  bool shape_received_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5eMotionNode>());
  rclcpp::shutdown();
  return 0;
}

