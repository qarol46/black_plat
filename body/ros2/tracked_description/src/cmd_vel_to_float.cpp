#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

class CmdVelToFloat : public rclcpp::Node
{
public:
  CmdVelToFloat()
  : Node("cmd_vel_to_float"), geom_pos_(0.0)
  {
    using std::placeholders::_1;

    pub_lin_  = create_publisher<std_msgs::msg::Float64>(
        "/lin_vel_controller/commands", 10);
    pub_ang_  = create_publisher<std_msgs::msg::Float64>(
        "/ang_vel_controller/commands", 10);
    pub_geom_ = create_publisher<std_msgs::msg::Float64>(
        "/geom_position_controller/commands", 10);

    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdVelToFloat::cbTwist, this, _1));

    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&CmdVelToFloat::cbJoy, this, _1));
  }

private:
  /* приём /cmd_vel -------------------------------------------------------- */
  void cbTwist(const geometry_msgs::msg::Twist & msg)
  {
    std_msgs::msg::Float64 lin, ang;
    lin.data = msg.linear.x;
    ang.data = msg.angular.z;
    pub_lin_->publish(lin);
    pub_ang_->publish(ang);
  }

  /* приём /joy для флиппера ---------------------------------------------- */
  void cbJoy(const sensor_msgs::msg::Joy & js)
  {
    bool down = js.buttons.size() > 0 && js.buttons[0];   // A
    bool up   = js.buttons.size() > 3 && js.buttons[3];   // Y

    if (down) geom_pos_ -= 0.05;
    if (up)   geom_pos_ += 0.05;

    std_msgs::msg::Float64 g;
    g.data = geom_pos_;
    pub_geom_->publish(g);
  }

  /* поля ------------------------------------------------------------------ */
  double geom_pos_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lin_, pub_ang_, pub_geom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr       sub_joy_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToFloat>());
  rclcpp::shutdown();
  return 0;
}
