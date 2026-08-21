#include "spin_towards_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/utils.h>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{


  SpinTowardsTrajectory::SpinTowardsTrajectory(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : SpinAction(xml_tag_name, "spin", conf)
  { }
  
  
  void SpinTowardsTrajectory::on_tick()
  {
      geometry_msgs::msg::PoseStamped current_pose;
      std::shared_ptr<tf2_ros::Buffer> tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
      std::string global_frame_="map";
      std::string robot_base_frame_="base_link";
      bool pose_exist=true;
      if (!nav2_util::getCurrentPose(
      current_pose,
      *tf_,
      global_frame_,
      robot_base_frame_) || current_pose.header.frame_id.empty())
      {
          RCLCPP_ERROR(rclcpp::get_logger("SpinTowardsTrajectory"), "Failed to get current_pose from blackboard");
          pose_exist=false;
      }

      nav_msgs::msg::Path path;
      bool path_exist=true;
      if (!getInput("path", path) || path.poses.empty()) {
          RCLCPP_ERROR(rclcpp::get_logger("SpinTowardsTrajectory"), "Failed to get path from blackboard");
          path_exist=false;
      }
      
      if(!pose_exist || !path_exist)
        return;
      
      double robot_yaw = tf2::getYaw(current_pose.pose.orientation);
      double dx, dy;
      
      for (size_t i = 0; i < path.poses.size(); i++) {
        dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
        dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
        if(std::hypot(dx, dy)>0.3){
          break;
        }
      }
      double target_global_yaw = std::atan2(dy, dx);
      
      double angle_diff = target_global_yaw - robot_yaw;
      angle_diff = std::fmod(angle_diff + M_PI, 2 * M_PI);
      if (angle_diff < 0) angle_diff += 2 * M_PI;
      angle_diff -= M_PI;

      RCLCPP_INFO(rclcpp::get_logger("SpinTowardsTrajectory"), "Target yaw: %f", angle_diff);
      goal_.target_yaw=-angle_diff;//>0 ? 1.57 : -1.57;
      return;
      
  }
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpinTowardsTrajectory>("SpinTowardsTrajectory");
}

