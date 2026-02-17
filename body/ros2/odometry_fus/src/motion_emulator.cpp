#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"

class MotionEmulator : public rclcpp::Node
{
public:
    MotionEmulator(double distance, double angle)
    : Node("motion_emulator"),
      target_distance_(distance),
      target_angle_(angle * M_PI / 180.0), // конвертируем градусы в радианы
      linear_speed_(0.5),
      angular_speed_(0.5),
      position_tolerance_(0.01),
      angle_tolerance_(0.01),
      target_reached_(false),
      motion_phase_(0) // 0 - движение вперед, 1 - поворот
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MotionEmulator::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Starting motion emulator");
        RCLCPP_INFO(this->get_logger(), "Target distance: %.2f m", target_distance_);
        RCLCPP_INFO(this->get_logger(), "Target angle: %.2f degrees", angle);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (target_reached_) return;

        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Получаем yaw из кватерниона
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        // Запоминаем начальную позицию при первом вызове
        if (!initial_position_set_) {
            initial_x_ = current_x_;
            initial_y_ = current_y_;
            initial_yaw_ = current_yaw_;
            initial_position_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial position captured");
        }

        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        if (motion_phase_ == 0 && target_distance_ > 0) {
            handle_forward_motion(*cmd_msg);
        } else if (motion_phase_ == 1 && std::abs(target_angle_) > 0.001) {
            handle_relative_turn_motion(*cmd_msg);
        } else {
            target_reached_ = true;
            cmd_msg->linear.x = 0.0;
            cmd_msg->angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Motion completed");
            rclcpp::shutdown();
        }

        cmd_pub_->publish(std::move(cmd_msg));
    }

    void handle_forward_motion(geometry_msgs::msg::Twist& msg)
    {
        double dx = current_x_ - initial_x_;
        double dy = current_y_ - initial_y_;
        double distance = std::hypot(dx, dy);

        if (distance < target_distance_ - position_tolerance_) {
            msg.linear.x = linear_speed_;
        } else {
            msg.linear.x = 0.0;
            motion_phase_ = 1; // Переходим к фазе поворота
            turn_start_yaw_ = current_yaw_; // Запоминаем текущий yaw как начальный для поворота
            RCLCPP_INFO(this->get_logger(), "Distance target reached: %.3f m", distance);
        }
    }

    void handle_relative_turn_motion(geometry_msgs::msg::Twist& msg)
    {
        // Вычисляем относительный угол поворота от начальной ориентации
        double angle_turned = normalize_angle(current_yaw_ - turn_start_yaw_);

        if (std::abs(angle_turned) < std::abs(target_angle_) - angle_tolerance_) {
            msg.angular.z = (target_angle_ > 0) ? angular_speed_ : -angular_speed_;
        } else {
            msg.angular.z = 0.0;
            target_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Turn completed: %.2f degrees (relative)", 
                       angle_turned * 180.0 / M_PI);
        }
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Параметры движения
    double target_distance_;
    double target_angle_;
    double linear_speed_;
    double angular_speed_;
    double position_tolerance_;
    double angle_tolerance_;

    // Текущее состояние
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double initial_x_ = 0.0;
    double initial_y_ = 0.0;
    double initial_yaw_ = 0.0;
    double turn_start_yaw_ = 0.0;
    bool initial_position_set_ = false;
    int motion_phase_; // 0 - движение вперед, 1 - поворот
    bool target_reached_ = false;

    // ROS интерфейсы
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <distance_m> <angle_deg>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    
    double distance = std::stod(argv[1]);
    double angle = std::stod(argv[2]);
    
    auto node = std::make_shared<MotionEmulator>(distance, angle);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}