#include "turtlebot_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

TurtlebotController::TurtlebotController() : Node("cmd_publisher"){
    //Parameter
    this->declare_parameter("robot_name", "robot_name");
    robot_name = this->get_parameter("robot_name").as_string();
    this->declare_parameter("scan_name", "scan_name");
    scan_name = this->get_parameter("scan_name").as_string();

    //subscriber
    rclcpp::QoS qos = rclcpp::QoS(10).best_effort();
    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_name, qos, std::bind(&TurtlebotController::scan_callback, this ,std::placeholders::_1));
    
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
    float angle_increment = msg->angle_increment;
    scan_flag = true;

    std::fill(std::begin(wild_lidar_distance), std::end(wild_lidar_distance), 0.0);
    std::fill(std::begin(lidar_distance), std::end(lidar_distance), 0.0);
    
    size_t num_points = msg->ranges.size();
    for(size_t i=0; i<num_points; i++){
        int index = static_cast<int>((i * angle_increment) * (180.0 / M_PI));
        if (index >= 0 && index < 360) {
            wild_lidar_distance[index] = std::isnan(msg->ranges[i]) ? 0.0 : msg->ranges[i];
        }
    }
    for (int j = 0; j < 360; j++){
        double distance_sum = 0.0;
        int distance_sum_ea = 0;

        for (int k = -2; k <= 2; k++){
            int neighbor_k = (j + k + 360) % 360;
            if (wild_lidar_distance[neighbor_k] != 0){
                distance_sum += wild_lidar_distance[neighbor_k];
                distance_sum_ea++;
            }
        }
        lidar_distance[j] = (distance_sum_ea != 0) ? (distance_sum / distance_sum_ea) : 0.0;
    }
}


void TurtlebotController::cmd_timer_callback(){
    if (!tf_flag)
        return;

    double front_closest = std::numeric_limits<double>::max();
    double right_closest = std::numeric_limits<double>::max();
    int front_closest_degree;
    int right_closest_degree;

    geometry_msgs::msg::Twist cmd_vel;

    for (int i = -20; i <= 20; i++){
        int neighbor_i = (i + 360) % 360;
        if (front_closest > lidar_distance[neighbor_i] && lidar_distance[neighbor_i] != 0){
            front_closest = lidar_distance[neighbor_i];
            front_closest_degree = neighbor_i;
        }
    }
    for (int i = -30; i <= 50; i++){
        if (right_closest > lidar_distance[i + 270] && lidar_distance[i + 270] != 0){
            right_closest = lidar_distance[i + 270];
            right_closest_degree = i + 270;
        }
    }

    RCLCPP_INFO(this->get_logger(), "front_closest {%d}={%f}", front_closest_degree, front_closest);
    RCLCPP_INFO(this->get_logger(), "right_closest {%d}={%f}", right_closest_degree, right_closest);

    cmd_vel.linear.x = 2.0;

    switch(scan_state){
        case PARALLELING:{
            if (front_closest < 0.3)
                scan_state = LEFT_FACE;
            
            int right_error_degree = right_closest_degree - 270;
            cmd_vel.angular.z = k_p[0] * right_error_degree;
            RCLCPP_INFO(this->get_logger(), "case: PARALLELING");
            break;
        }
        case LEFT_FACE:{
            left_face_flag = true;
            if (front_closest_degree > 180)
                front_closest_degree -= 360;
            int front_error_degree = front_closest_degree + 90;
            cmd_vel.angular.z = k_l[0] * front_error_degree;
            if (front_closest > right_closest)
                scan_state = PARALLELING;
            RCLCPP_INFO(this->get_logger(), "case: LEFT_FACE");
            break;
        }
        case GAP_TUNNING:{
            cmd_vel.angular.z = 0;
            scan_state = PARALLELING;
            // double error_gap = 0.3 - right_distance;
            // if (abs(error_gap) < 0.02){
            //     cmd_vel.angular.z = 0;
            //     scan_state = PARALLELING;
            // }
            // else if(error_gap > 0.02){
            //     if(!theta_flag){
            //         goal_th = real_th + M_PI / 6;
            //         theta_flag = true;
            //     }
            //     cmd_vel.angular.z = 0.2 * (2 - sqrt(3)) / error_gap;
            //     if (abs(error_th) < 0.01){
            //         if(!theta_flag2){
            //             goal_th = real_th - M_PI / 6;
            //             theta_flag2 = true;
            //         }
            //     }
            //     if (theta_flag2){
            //         cmd_vel.angular.z = - 0.2 * (2 - sqrt(3)) / error_gap;
            //         if(abs(error_th) < 0.01){
            //             cmd_vel.angular.z = 0;
            //             theta_flag = false;
            //             theta_flag2 = false;
            //         }
            //     }
            // }
            // else if(error_gap < - 0.02){
            //     if(!theta_flag){
            //         goal_th = real_th - M_PI / 6;
            //         theta_flag = true;
            //     }
            //     cmd_vel.angular.z = - 0.2 * (2 - sqrt(3)) / error_gap;
            //     if (abs(error_th) < 0.01){
            //         if(!theta_flag2){
            //             goal_th = real_th + M_PI / 6;
            //             theta_flag2 = true;
            //         }
            //     }
            //     if (theta_flag2){
            //         cmd_vel.angular.z = 0.2 * (2 - sqrt(3)) / error_gap;
            //         if(abs(error_th) < 0.01){
            //             cmd_vel.angular.z = 0;
            //             theta_flag = false;
            //             theta_flag2 = false;
            //         }
            //     }
            // }
            RCLCPP_INFO(this->get_logger(), "case: GAP_TUNNING");
            break;
        }
    }
    if (cmd_vel.angular.z > 2.84)
        cmd_vel.angular.z = 2.84;
    if (cmd_vel.angular.z < -2.84)
        cmd_vel.angular.z = -2.84; 
    cmd_publisher->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "v = {%f}, w = {%f}", cmd_vel.linear.x, cmd_vel.angular.z);
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

