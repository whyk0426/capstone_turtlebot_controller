#ifndef TURTLEBOT_CONTROLLER_HPP
#define TURTLEBOT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include <cmath>

#include "std_msgs/msg/string.hpp"

class TurtlebotController : public rclcpp::Node{
    public:
        TurtlebotController();

    private:
        void tf_timer_callback();
        void cmd_timer_callback();
        void topic_callback(const std_msgs::msg::String::SharedPtr msg);

        std::string robot_name;
        double goal_x, goal_y;
        double goal_th = 0;

        double real_x = 0, real_y = 0, real_th = 0;
        double k_l[1] = {1};
        double k_a[1] = {1};

        bool tf_flag = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    rclcpp::TimerBase::SharedPtr cmd_timer, tf_timer;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listner;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_subscription;
};

#endif // TURTLEBOT_CONTROLLER_HPP