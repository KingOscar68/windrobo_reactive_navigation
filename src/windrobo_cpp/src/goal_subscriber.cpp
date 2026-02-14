#include "rclcpp/rclcpp.hpp"
#include "windrobo_msgs/msg/goal.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>

class ReactiveNavigator : public rclcpp::Node
{
public:
    ReactiveNavigator() : Node("reactive_controller")
    {
        goal_sub_ = this->create_subscription<windrobo_msgs::msg::Goal>(
            "final_goal", 10,
            std::bind(&ReactiveNavigator::goal_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&ReactiveNavigator::odom_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&ReactiveNavigator::scan_callback, this, std::placeholders::_1));

        env_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "environment_ready", 10,
            std::bind(&ReactiveNavigator::env_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Reactive Controller Ready");
    }

private:

    // 🔧 Parámetros ajustables
    double max_linear_ = 0.25;
    double max_angular_ = 0.4;
    double obstacle_threshold_ = 0.45;
    double goal_tolerance_ = 0.15;

    bool goal_received_ = false;
    bool environment_ready_ = false;

    double goal_x_, goal_y_;
    double robot_x_, robot_y_, robot_yaw_;

    std::vector<float> ranges_;
    float angle_min_, angle_increment_;

    // 🔥 Memoria de evitación
    int turn_direction_ = 0;      // -1 derecha, +1 izquierda
    bool in_avoidance_ = false;

    rclcpp::Subscription<windrobo_msgs::msg::Goal>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr env_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    void goal_callback(const windrobo_msgs::msg::Goal::SharedPtr msg)
    {
        goal_x_ = msg->goal_x;
        goal_y_ = msg->goal_y;
        goal_received_ = true;
    }

    void env_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        environment_ready_ = msg->data;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        navigate();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        ranges_ = msg->ranges;
        angle_min_ = msg->angle_min;
        angle_increment_ = msg->angle_increment;
    }

    float get_distance_at_angle(double angle_deg)
    {
        double angle_rad = angle_deg * M_PI / 180.0;

        while(angle_rad < angle_min_)
            angle_rad += 2*M_PI;

        while(angle_rad > angle_min_ + ranges_.size()*angle_increment_)
            angle_rad -= 2*M_PI;

        int index = (angle_rad - angle_min_) / angle_increment_;

        if(index >= 0 && index < (int)ranges_.size())
        {
            if(std::isfinite(ranges_[index]))
                return ranges_[index];
        }

        return std::numeric_limits<float>::infinity();
    }

    void navigate()
    {
        if(!goal_received_ || !environment_ready_ || ranges_.empty())
            return;

        geometry_msgs::msg::Twist cmd;

        double dx = goal_x_ - robot_x_;
        double dy = goal_y_ - robot_y_;
        double distance = std::sqrt(dx*dx + dy*dy);

        if(distance < goal_tolerance_)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        // 🔎 Buscar distancia mínima en sectores críticos
        std::vector<double> angles = {
            0,
            15, 30, 45, 60, 75, 90,
            360-15, 360-30, 360-45, 360-60, 360-75, 360-90
        };

        float min_distance = std::numeric_limits<float>::infinity();

        for(double angle : angles)
        {
            float d = get_distance_at_angle(angle);
            if(d < min_distance)
                min_distance = d;
        }

        // ===============================
        // 🚀 ESPACIO LIBRE
        // ===============================
        if(min_distance > obstacle_threshold_)
        {
            in_avoidance_ = false;

            double goal_angle = std::atan2(dy, dx);
            double error = goal_angle - robot_yaw_;

            while(error > M_PI) error -= 2*M_PI;
            while(error < -M_PI) error += 2*M_PI;

            cmd.linear.x = max_linear_;
            cmd.angular.z = std::clamp(error, -max_angular_, max_angular_);
        }
        // ===============================
        // 🛑 EVITACIÓN
        // ===============================
        else
        {
            // 🔥 Reducir velocidad proporcionalmente
            double ratio = min_distance / obstacle_threshold_;
            double slow_factor = ratio * ratio;   // reducción cuadrática
            slow_factor = std::clamp(slow_factor, 0.0, 1.0);

            cmd.linear.x = max_linear_ * slow_factor;

            if(!in_avoidance_)
            {
                float left_avg =
                    (get_distance_at_angle(45) +
                    get_distance_at_angle(60) +
                    get_distance_at_angle(75)) / 3.0;

                float right_avg =
                    (get_distance_at_angle(360-45) +
                    get_distance_at_angle(360-60) +
                    get_distance_at_angle(360-75)) / 3.0;

                if(left_avg > right_avg)
                    turn_direction_ = -1;
                else
                    turn_direction_ = 1;

                in_avoidance_ = true;
            }

            // 🔥 Giro proporcional a qué tan cerca está el obstáculo
            double turn_intensity = 1.0 - slow_factor;
            turn_intensity = std::clamp(turn_intensity, 0.0, 1.0);

            cmd.angular.z = turn_direction_ * max_angular_ * (0.5 + turn_intensity);

        }


        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveNavigator>());
    rclcpp::shutdown();
    return 0;
}
