#ifndef TURTLEBOT_CONTROLLER_HPP
#define TURTLEBOT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

enum SCAN_STATE {PARALLELING, GAP_TUNNING, LEFT_FACE,};

class TurtlebotController : public rclcpp::Node{
    public:
        TurtlebotController();

    private:
        void tf_timer_callback();
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void cmd_timer_callback();

        std::string robot_name;
        std::string scan_name;
        double goal_x = 0.0;
        double goal_y = 0.0;
        double goal_th = 0.0;

        double real_x = 0, real_y = 0, real_th = 0;
        double k_p[1] = {0.05};
        double k_l[1] = {0.05};

        bool tf_flag = false;
        bool scan_flag = false;
        bool left_face_flag = false;

        SCAN_STATE scan_state = PARALLELING;

        double wild_lidar_distance[360] = {0};
        double lidar_distance[360] = {0};

    //Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber; 
    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    //TF Listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    //Timer
    rclcpp::TimerBase::SharedPtr cmd_timer, tf_timer;  
};

#endif // TURTLEBOT_CONTROLLER_HPP