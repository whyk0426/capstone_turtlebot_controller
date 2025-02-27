#include "turtlebot_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

TurtlebotController::TurtlebotController() : Node("cmd_publisher"){
    //Parameter
    this->declare_parameter("robot_name", "robot_name");
    robot_name = this->get_parameter("robot_name").as_string();
    this->declare_parameter("scan_name", "/scan");
    scan_name = this->get_parameter("scan_name").as_string();

    //subscriber
    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TurtlebotController::scan_callback, this, _1));
    
    //publisher
    cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //TF Listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    //Timer
    tf_timer = this->create_wall_timer(10ms, std::bind(&TurtlebotController::tf_timer_callback, this));
    cmd_timer = this->create_wall_timer(100ms, std::bind(&TurtlebotController::cmd_timer_callback, this));
}


void TurtlebotController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    float angle_min = msg->angle_min;
    // float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;
    scan_flag = true;

    std::fill(std::begin(lidar_distance), std::end(lidar_distance), 0.0);
    RCLCPP_WARN(this->get_logger(), "##########################################");
    size_t num_points = msg->ranges.size();
    for(size_t i=0; i<num_points; i++){
        int index = static_cast<int>((angle_min + i * angle_increment) * (180.0 / M_PI));
        if (index >= 0 && index < 360) {
            lidar_distance[index] = std::isnan(msg->ranges[i]) ? 0.0 : msg->ranges[i];
        }
        
        RCLCPP_WARN(this->get_logger(), "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        RCLCPP_WARN(this->get_logger(), "[index]: {%f}", lidar_distance[index]);
    }
}


void TurtlebotController::cmd_timer_callback(){
    if (!tf_flag)
        return;

    double front_distance = 0.0;
    double front_left_distance = 0.0;
    double front_right_distance = 0.0;

    double right_distance = 0.0;
    double right_front_distance = 0.0;
    double right_back_distance = 0.0;

    double closest_distance = std::numeric_limits<double>::max();

    geometry_msgs::msg::Twist cmd_vel;

    for (int i = -2; i < 3; i++){
        int j = i + 360;
        if (j > 359)
            j -= 360; 
        front_distance += lidar_distance[j];
        // RCLCPP_INFO(this->get_logger(), "front_distance {%f}", lidar_distance[j]);
        right_distance += lidar_distance[i+270];
        // RCLCPP_INFO(this->get_logger(), "right_distance {%f}", lidar_distance[i+270]);
    }
    for (int i = -4; i < 5; i++){
        front_left_distance += lidar_distance[i+5];
        front_right_distance += lidar_distance[i+355];
        right_front_distance += lidar_distance[i+275];
        right_back_distance += lidar_distance[i+265];
        // RCLCPP_INFO(this->get_logger(), "front_left_distance {%f}", lidar_distance[i+5]);
        // RCLCPP_INFO(this->get_logger(), "front_right_distance {%f}", lidar_distance[i+355]);
        // RCLCPP_INFO(this->get_logger(), "right_front_distance {%f}", lidar_distance[i+275]);
        // RCLCPP_INFO(this->get_logger(), "right_back_distance {%f}", lidar_distance[i+265]);
    }
    for (int i = 0; i < 13; i++){
        if (closest_distance > lidar_distance[i+270] && lidar_distance[i+270] != 0){
            closest_distance = lidar_distance[i+270];
        }
    }

    front_distance /= 5;
    right_distance /= 5;
    front_left_distance /= 9;
    front_right_distance /= 9;
    right_front_distance /= 9;
    right_back_distance /= 9;
    // RCLCPP_INFO(this->get_logger(), "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    // RCLCPP_INFO(this->get_logger(), "front_distance {%f}", front_distance);
    // RCLCPP_INFO(this->get_logger(), "right_distance {%f}", right_distance);
    double error_th = goal_th - real_th;
    if (error_th > M_PI)
        error_th -= 2 * M_PI;
    else if (error_th < - M_PI)
        error_th += 2 * M_PI;

    cmd_vel.linear.x = 0.2;

    switch(scan_state){
        case PARALLELING:{
            if (front_distance < 0.5)
                scan_state = LEFT_FACE;
                
            if (closest_distance < 0.1)
                cmd_vel.linear.x = 0.0;

            double right_gap = right_back_distance - right_front_distance;
            if(abs(right_gap) < 0.01){
                cmd_vel.angular.z = 0;
                scan_state = GAP_TUNNING;
            }
            else if(right_gap > 0.01){
                if (!theta_flag){
                goal_th = real_th + angular_calculator(right_back_distance, right_front_distance);
                theta_flag = true;
                }
                cmd_vel.angular.z = 0.5;
                if (abs(right_gap) < 0.01){
                    cmd_vel.angular.z = 0;
                    scan_state = GAP_TUNNING;
                    theta_flag = false;
                }
            }
            else if(right_gap < -0.01){
                if (!theta_flag){
                goal_th = real_th - angular_calculator(right_front_distance, right_back_distance);
                theta_flag = true;
                }
                if (abs(right_gap) < 0.01){
                    cmd_vel.angular.z = 0;
                    scan_state = GAP_TUNNING;
                    theta_flag = false;
                }
            }
            RCLCPP_INFO(this->get_logger(), "case: PARALLELING");
            break;
        }
        case GAP_TUNNING:{
            double error_gap = 0.3 - right_distance;
            if (abs(error_gap) < 0.02){
                cmd_vel.angular.z = 0;
                scan_state = PARALLELING;
            }
            else if(error_gap > 0.02){
                if(!theta_flag){
                    goal_th = real_th + M_PI / 6;
                    theta_flag = true;
                }
                cmd_vel.angular.z = 0.2 * (2 - sqrt(3)) / error_gap;
                if (abs(error_th) < 0.01){
                    if(!theta_flag2){
                        goal_th = real_th - M_PI / 6;
                        theta_flag2 = true;
                    }
                }
                if (theta_flag2){
                    cmd_vel.angular.z = - 0.2 * (2 - sqrt(3)) / error_gap;
                    if(abs(error_th) < 0.01){
                        cmd_vel.angular.z = 0;
                        theta_flag = false;
                        theta_flag2 = false;
                    }
                }
            }
            else if(error_gap < - 0.02){
                if(!theta_flag){
                    goal_th = real_th - M_PI / 6;
                    theta_flag = true;
                }
                cmd_vel.angular.z = - 0.2 * (2 - sqrt(3)) / error_gap;
                if (abs(error_th) < 0.01){
                    if(!theta_flag2){
                        goal_th = real_th + M_PI / 6;
                        theta_flag2 = true;
                    }
                }
                if (theta_flag2){
                    cmd_vel.angular.z = 0.2 * (2 - sqrt(3)) / error_gap;
                    if(abs(error_th) < 0.01){
                        cmd_vel.angular.z = 0;
                        theta_flag = false;
                        theta_flag2 = false;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "case: GAP_TUNNING");
            break;
        }
        case LEFT_FACE:{
            if (!theta_flag){
            goal_th = real_th + M_PI / 2 - angular_calculator(front_right_distance, front_left_distance);
            theta_flag = true;
            }
            cmd_vel.angular.z = 1.0;
            if (abs(error_th) < 0.01){
                cmd_vel.angular.z = 0;
                scan_state = PARALLELING;
                theta_flag = false;
            }
            RCLCPP_INFO(this->get_logger(), "case: LEFT_FACE");
            break;
        }
    }
    // cmd_publisher->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "w = {%f}", cmd_vel.angular.z);
}


double TurtlebotController::angular_calculator(double d1, double d2){
    double l = sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * cos(M_PI / 18));
    double ratio = d1 * sin(M_PI / 18) / l;
    ratio = std::clamp(ratio, -1.0, 1.0);
    double alpha = asin(ratio) - M_PI * 85 / 180;

    return alpha;
}


void TurtlebotController::tf_timer_callback(){
    geometry_msgs::msg::TransformStamped t;

    try{
        t = tf_buffer->lookupTransform(
            "map", robot_name + "_imu_link", tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
        return;
    }
    double z = t.transform.rotation.z;
    double w = t.transform.rotation.w;

    real_x = t.transform.translation.x;
    real_y = t.transform.translation.y;
    real_th = 2 * atan2(z,w);

    tf_flag = true;
}

