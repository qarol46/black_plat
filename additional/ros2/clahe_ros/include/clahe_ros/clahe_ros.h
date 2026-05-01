#ifndef CLAHE_ROS_H
#define CLAHE_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class ClaheRos : public rclcpp::Node
{
public:
  explicit ClaheRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  double clahe_clip_limit_sky_;
  int clahe_grid_size_sky_;
  double clahe_clip_limit_asphalt_;
  int clahe_grid_size_asphalt_;
  int rect_sky_x_;
  int rect_sky_y_;
  int rect_sky_w_;
  int rect_sky_h_;
  int rect_asphalt_x_;
  int rect_asphalt_y_;
  int rect_asphalt_w_;
  int rect_asphalt_h_;
  double gamma_;

  image_transport::Publisher  image_pub_;
  image_transport::Subscriber image_sub_;
};

#endif  // CLAHE_ROS_H