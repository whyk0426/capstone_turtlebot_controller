#include "turtlebot_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

const double pi = 3.14159265358979323846; 

TurtlebotController::TurtlebotController() : Node("turtlebot_controller"){
    cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    arrival_pulisher = this->create_publisher<std_msgs::msg::String>("/arrival", 10);

    topic_subscription = this->create_subscription<geometry_msgs::msg::Point>(
        "/goal_state", 10, std::bind(&TurtlebotController::topic_callback, this, std::placeholders::_1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    cmd_timer = this->create_wall_timer(
        10ms, std::bind(&TurtlebotController::cmd_timer_callback, this));
    tf_timer = this->create_wall_timer(
        10ms, std::bind(&TurtlebotController::tf_timer_callback, this));

    this->declare_parameter("robot_name", "robot_name");
    robot_name = this->get_parameter("robot_name").as_string();

    // this->declare_parameter("goal_x", 0.0);
    // goal_x = this->get_parameter("goal_x").as_double();

    // this->declare_parameter("goal_y", 0.0);
    // goal_y = this->get_parameter("goal_y").as_double();

    // this->declare_parameter("goal_th", 0.0);
    // goal_th = this->get_parameter("goal_th").as_double();
}


void TurtlebotController::tf_timer_callback(){
    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer->lookupTransform(
        "map", robot_name + "_imu_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s",
        ex.what());
      return;
    }
    double z = t.transform.rotation.z;
    double w = t.transform.rotation.w;

    real_x = t.transform.translation.x;
    real_y = t.transform.translation.y;
    real_th = 2 * atan2(z,w);

    tf_flag = true;
}

void TurtlebotController::cmd_timer_callback(){
    if (!tf_flag)
      return;

    geometry_msgs::msg::Twist cmd_vel;

    double x_d = goal_x - real_x;
    double y_d = goal_y - real_y;
    double goal_th = atan2(y_d, x_d);

    double error_distance = sqrt(x_d * x_d + y_d * y_d);
    double error_theta = goal_th - real_th;
      if(error_theta > pi)
        error_theta -= 2 * pi;
      else if (error_theta < - pi)
        error_theta += 2 * pi;

    cmd_vel.linear.x = k_l[0] * error_distance;
    cmd_vel.angular.z = k_a[0] * error_theta;

    if (error_distance < 0.1){
      RCLCPP_WARN(this->get_logger(), "distance arrived");
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;

      std_msgs::msg::String msg;
      msg.data = "Arriving";
      arrival_pulisher->publish(msg);

      // if(abs(error_theta) <0.1){
      //   RCLCPP_WARN(this->get_logger(), "angular arrived");
      //   cmd_vel.angular.z = 0;
      // }
    }
    if (cmd_vel.linear.x > 0.2)
      cmd_vel.linear.x = 0.2;
    else if (cmd_vel.linear.x < -0.2)
      cmd_vel.linear.x = -0.2;
    
    if (cmd_vel.angular.z > 2.84)
      cmd_vel.angular.z = 2.84;
    else if (cmd_vel.angular.z < -2.84)
      cmd_vel.angular.z = -2.84; 
    
    RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    RCLCPP_INFO(this->get_logger(), "error_distance:{%f}, linear.x {%f}",error_distance, cmd_vel.linear.x);

    cmd_publisher->publish(cmd_vel);
}


void TurtlebotController::topic_callback(const geometry_msgs::msg::Point::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "#############################################################");
    RCLCPP_INFO(this->get_logger(), "Received goal state: goal_x=%f, goal_y=%f", msg->x, msg->y);
    
    goal_x = msg->x;
    goal_y = msg->y;
}

