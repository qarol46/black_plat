#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class MotionEmulator : public rclcpp::Node
{
public:
    MotionEmulator(double distance, double angle, bool do_forward, bool do_turn)
    : Node("motion_emulator"),
      target_distance_(distance),
      target_angle_(angle * M_PI / 180.0), // конвертируем градусы в радианы
      do_forward_(do_forward),
      do_turn_(do_turn),
      linear_speed_(0.1),
      angular_speed_(0.5),
      position_tolerance_(0.01),
      angle_tolerance_(0.02),
      target_reached_(false)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "diff_drive_controller/odom", 10, std::bind(&MotionEmulator::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Starting motion emulator");
        if (do_forward_) {
            RCLCPP_INFO(this->get_logger(), "Target distance: %.2f m", target_distance_);
        }
        if (do_turn_) {
            RCLCPP_INFO(this->get_logger(), "Target angle: %.2f degrees", angle);
        }
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
            RCLCPP_INFO(this->get_logger(), "Initial yaw: %.2f degrees", initial_yaw_ * 180.0 / M_PI);
        }

        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        bool forward_complete = true;
        bool turn_complete = true;

        // Обработка движения вперед
        if (do_forward_) {
            forward_complete = handle_forward_motion(*cmd_msg);
        }

        // Обработка поворота
        if (do_turn_) {
            turn_complete = handle_turn_motion(*cmd_msg);
        }

        // Если все запрошенные движения завершены
        if ((!do_forward_ || forward_complete) && (!do_turn_ || turn_complete)) {
            cmd_msg->linear.x = 0.0;
            cmd_msg->angular.z = 0.0;
            target_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Motion completed");
            rclcpp::shutdown();
        }

        cmd_pub_->publish(std::move(cmd_msg));
    }

    bool handle_forward_motion(geometry_msgs::msg::Twist& msg)
    {
        double dx = current_x_ - initial_x_;
        double dy = current_y_ - initial_y_;
        double distance = std::hypot(dx, dy);

        RCLCPP_DEBUG(this->get_logger(), "Distance progress: %.3f/%.3f m", distance, target_distance_);

        if (distance < target_distance_ - position_tolerance_) {
            msg.linear.x = linear_speed_;
            return false; // движение не завершено
        }
        return true; // движение завершено
    }

    bool handle_turn_motion(geometry_msgs::msg::Twist& msg)
    {
        // Вычисляем целевой абсолютный угол
        double target_abs_yaw = normalize_angle(initial_yaw_ + target_angle_);
        
        // Вычисляем разницу между текущим и целевым углом
        double angle_diff = normalize_angle(target_abs_yaw - current_yaw_);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Turn progress: Current=%.2f°, Target=%.2f°, Diff=%.2f°",
                    current_yaw_ * 180.0 / M_PI,
                    target_abs_yaw * 180.0 / M_PI,
                    angle_diff * 180.0 / M_PI);

        // Если разница больше допуска
        if (std::abs(angle_diff) > angle_tolerance_) {
            // Определяем направление поворота (по кратчайшему пути)
            msg.angular.z = (angle_diff > 0) ? angular_speed_ : -angular_speed_;
            return false; // поворот не завершен
        }
        return true; // поворот завершен
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
    bool do_forward_;
    bool do_turn_;
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
    bool initial_position_set_ = false;
    bool target_reached_ = false;

    // ROS интерфейсы
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Парсинг аргументов командной строки
    bool do_forward = false;
    bool do_turn = false;
    double distance = 0.0;
    double angle = 0.0;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--forward" && i + 1 < argc) {
            do_forward = true;
            distance = std::stod(argv[++i]);
        } else if (arg == "--turn" && i + 1 < argc) {
            do_turn = true;
            angle = std::stod(argv[++i]);
        } else {
            std::cerr << "Usage: " << argv[0] << " [--forward <distance_m>] [--turn <angle_deg>]" << std::endl;
            return 1;
        }
    }
    
    if (!do_forward && !do_turn) {
        std::cerr << "Error: You must specify at least one motion type (--forward or --turn)" << std::endl;
        return 1;
    }
    
    auto node = std::make_shared<MotionEmulator>(distance, angle, do_forward, do_turn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}