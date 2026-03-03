#include "nav2_behavior_tree/plugins/action/spin_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_behavior_tree
{
class SpinTowardsTrajectory : public SpinAction
{
public:
  SpinTowardsTrajectory(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
  
  static BT::PortsList providedPorts()
  {
      return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path"),
      });
  }
  
private:
  void on_tick() override;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  nav_msgs::msg::Path::SharedPtr current_global_path;
  geometry_msgs::msg::PoseStamped current_pose;
  
  bool getCurrentPose(geometry_msgs::msg::PoseStamped);
};

}  // namespace nav2_behavior_tree
