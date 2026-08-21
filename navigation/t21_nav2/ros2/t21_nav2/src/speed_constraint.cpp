#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>


class SpeedConstraint : public rclcpp::Node
{
    public:
        SpeedConstraint()
        : Node("recovery_node")
        {
            rclcpp::QoS qos(2);
            qos.keep_last(2);
            qos.reliable();
            qos.durability_volatile();

            vel = this->create_subscription<geometry_msgs::msg::Twist>(
             "/cmd_vel_smoothed", qos, std::bind(&SpeedConstraint::recovery_action, this, std::placeholders::_1));
            cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", qos);
        }

        
    private:
    
        void recovery_action(const geometry_msgs::msg::Twist::SharedPtr msg){
            pub_cmd_vel.linear.x=msg->linear.x;
            if(msg->angular.z<0.5 && msg->angular.z>0)
                pub_cmd_vel.angular.z=0.5;
            else if(msg->angular.z>-0.5 && msg->angular.z<0)
                pub_cmd_vel.angular.z=-0.5;
            else
                pub_cmd_vel.angular.z=msg->angular.z;
            cmd_vel->publish(pub_cmd_vel);
            //RCLCPP_INFO(this->get_logger(), "Input angular.z: %.2f, Output angular.z: %.2f", 
             //msg->angular.z, pub_cmd_vel.angular.z);
        }
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel;
        geometry_msgs::msg::Twist pub_cmd_vel;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedConstraint>());
  rclcpp::shutdown();

  return 0;
}