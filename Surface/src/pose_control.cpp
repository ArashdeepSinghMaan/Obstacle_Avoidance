#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include "std_msgs/msg/float64.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Extract yaw from quaternion
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;  // Return yaw in radians
}


// PID Controller Class
class PIDController {
public:
    PIDController(double kp, double ki, double kd) 
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double error, double dt) {
        if (dt < 1e-6) return 0.0;  // Prevent division by zero
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

// MultiSubscriberNode Class Definition
class MultiSubscriberNode : public rclcpp::Node {
public:
    MultiSubscriberNode()
        : Node("multi_subscriber_node"),
          pid_submarine_x_(8.0, 0.4, 0.3),
          pid_submarine_y_(1.0, 0.0, 0.1),
          pid_submarine_z_(1.0, 0.0, 0.1),
          pid_wamv_x_(1.0, 0.0, 0.1),
          pid_wamv_y_(1.0, 0.0, 0.1),
          pid_wamv_orientation_(1.0, 0.0, 0.1)  // Corrected initialization list
    {
        // Initialize Subscribers
        sub_submarine_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_lrauv_modified/submarine/odometry", 10,
            std::bind(&MultiSubscriberNode::submarine_callback, this, std::placeholders::_1));

        sub_wamv_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wamv/ground_truth/odometry", 10,
            std::bind(&MultiSubscriberNode::wamv_callback, this, std::placeholders::_1));

        // Initialize Publishers
        pub_submarine_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/propeller/thrust", 10);
        pub_wamv_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        pub_wamv_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
        pub_submarine_fin_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/vertical/fin/pos", 10);

        // Set Desired Positions & Orientations
        desired_submarine_x_ = 12.0;
        desired_submarine_y_ = 12.0;
        desired_wamv_x_ = 10.0;
        desired_wamv_y_ = 20.0;

        last_time_ = this->now();
    }

private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wamv_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_right_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_;

    // Desired Positions
    double desired_submarine_x_=8;
    double desired_submarine_y_=12;
    double desired_wamv_x_=6;
    double desired_wamv_y_=12;

    // PID Controllers
    PIDController pid_submarine_x_, pid_submarine_y_, pid_submarine_z_;
    PIDController pid_wamv_x_, pid_wamv_y_, pid_wamv_orientation_;
    PIDController pid_submarine_yaw_{1.0, 0.1, 0.2};  // 

    rclcpp::Time last_time_;

    void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt < 1e-6) return;  // Prevent zero division

    // Extract Current Position
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // Compute Error
    double error_x = desired_submarine_x_ - current_x;
    RCLCPP_INFO(this->get_logger(), "Submarine Error X: [%.2f]", error_x);

    double error_y = desired_submarine_y_ - current_y;
    RCLCPP_INFO(this->get_logger(), "Submarine Error Y: [%.2f]", error_y);

    // PID Control Output for X
    double thrust_x = -pid_submarine_x_.compute(error_x, dt);

    // Apply thrust saturation
    thrust_x = std::clamp(thrust_x, -5.0, 5.0);

    // Apply Thrust based on Error direction.
    
    if (abs(error_x) <= 0.8) {
        thrust_x = 0;
    }

    // Publish Control Commands for X
    auto thrust_msg = std_msgs::msg::Float64();
    thrust_msg.data = thrust_x;
    pub_submarine_thrust_->publish(thrust_msg);
    RCLCPP_INFO(this->get_logger(), "Submarine Thrust X: [%.2f]", thrust_x);

    // ----- Yaw Control Logic -----
    double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

    // Compute desired angle to the target in the global frame
    double desired_angle_global = std::atan2(desired_submarine_y_ , desired_submarine_x_ );

    // Adjust desired angle for the submarine's frame:  Add PI (180 degrees)
    double desired_angle_submarine = desired_angle_global + M_PI;

    // Normalize the angle to be within -PI to PI.  Important!
    while (desired_angle_submarine > M_PI) {
        desired_angle_submarine -= 2.0 * M_PI;
    }
    while (desired_angle_submarine < -M_PI) {
        desired_angle_submarine += 2.0 * M_PI;
    }

    // Calculate the yaw error in the submarine's frame
    double yaw_error = desired_angle_submarine - current_yaw;

    // Normalize the error to be within -PI to PI.  Important!
    while (yaw_error > M_PI) {
        yaw_error -= 2.0 * M_PI;
    }
    while (yaw_error < -M_PI) {
        yaw_error += 2.0 * M_PI;
    }

    RCLCPP_INFO(this->get_logger(), "Yaw Error: [%.2f rad]", yaw_error);

    // PID Control for Vertical Fin
    double fin_position = pid_submarine_yaw_.compute(yaw_error, dt);
    RCLCPP_INFO(this->get_logger(), "Fin COntrol Input: [%.2f rad]", fin_position);
    fin_position = std::clamp(fin_position, -3.14, 3.14);  // Limit fin angle

    // Publish to Vertical Fin
    auto fin_msg = std_msgs::msg::Float64();

    // Convert to degrees if required by plugin
    fin_msg.data = -fin_position ;//* (180.0 / M_PI)

    pub_submarine_fin_->publish(fin_msg);

    RCLCPP_INFO(this->get_logger(), "Vertical Fin Position: [%.2f]", fin_position);
}


    void wamv_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt < 1e-6) return;  // Prevent zero division

        // Extract Current Position
        double current_x = msg->pose.pose.position.x;

        // Compute Error
        double error_x = desired_wamv_x_ - current_x;
        RCLCPP_INFO(this->get_logger(), "WAMV Error: [%.2f]", error_x);

        // PID Control Output
        double thrust_x = pid_wamv_x_.compute(error_x, dt);

        // Apply thrust saturation
        thrust_x = std::clamp(thrust_x, -5.0, 5.0);

        // Publish Control Commands
        auto thrust_msg = std_msgs::msg::Float64();
        thrust_msg.data = thrust_x;
        pub_wamv_left_thrust_->publish(thrust_msg);
        pub_wamv_right_thrust_->publish(thrust_msg);

        RCLCPP_INFO(this->get_logger(), "WAMV Thrust: [%.2f]", thrust_x);
    }

};

// Main Function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
