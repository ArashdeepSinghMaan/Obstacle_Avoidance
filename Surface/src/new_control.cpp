#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//With DMM
// Extract yaw from quaternion
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
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
          pid_submarine_x_(1.0, 0.4, 0.3),
          pid_submarine_y_(1.0, 0.0, 0.1),
          pid_submarine_yaw_(1.0, 0.1, 0.2)
    {
        // Subscribe to submarine odometry
        sub_submarine_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_lrauv_modified/submarine/odometry", 10,
            std::bind(&MultiSubscriberNode::submarine_callback, this, std::placeholders::_1));

        // Subscribe to front LiDAR sensor
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/my_lrauv_modified/submarine/Laser_scan_front", 10,
            std::bind(&MultiSubscriberNode::lidar_callback, this, std::placeholders::_1));

        // Initialize Publishers
        pub_submarine_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/propeller/thrust", 10);
        pub_submarine_fin_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/horizontal/fin/pos", 10);

        // Set Desired Positions
        desired_submarine_x_ = 12.0;
        desired_submarine_y_ = 12.0;

        last_time_ = this->now();
    }

private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_;

    // Desired Positions
    double desired_submarine_x_;
    double desired_submarine_y_;

    // PID Controllers
    PIDController pid_submarine_x_, pid_submarine_y_;
    PIDController pid_submarine_yaw_;

    rclcpp::Time last_time_;

    // Obstacle Data
    Eigen::Vector3d last_obstacle_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_obstacle_normal_ = Eigen::Vector3d::Zero();
    double last_obstacle_dist_ = std::numeric_limits<double>::max();
    double safe_distance_ = 3.0;  // Safety distance for DMM

    // LiDAR Callback - Extracts Obstacle Data
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double min_distance = std::numeric_limits<double>::max();
        Eigen::Vector3d obstacle_normal(0, 0, 0);
        Eigen::Vector3d obstacle_position(0, 0, 0);

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double range = msg->ranges[i];
            if (range < min_distance && range > msg->range_min) {
                min_distance = range;
                double angle = msg->angle_min + i * msg->angle_increment;
                obstacle_position.x() = range * cos(angle);
                obstacle_position.y() = range * sin(angle);
                obstacle_normal = obstacle_position.normalized();
             //   RCLCPP_INFO(this->get_logger(), "Obstacle Detected: [%.2f]", min_distance);
              //  RCLCPP_INFO(this->get_logger(), "Obstacle Normal: [%.2f, %.2f, %.2f]", 
          //  obstacle_normal.x(), obstacle_normal.y(), obstacle_normal.z());
//;;
            }
        }

        last_obstacle_pos_ = obstacle_position;
        last_obstacle_normal_ = obstacle_normal;
        last_obstacle_dist_ = min_distance;
    }

    //  Submarine Callback - Applies DMM and Controls Thrust/Yaw
    void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt < 1e-6) return;  // Prevent zero division

        // Extract Current Position
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        // Compute Desired Velocity Before Modulation
        Eigen::Vector3d v_d(desired_submarine_x_ - current_x,
                            desired_submarine_y_ - current_y, 
                            0);
        v_d.normalize();
      //  v_d *= 8.0;  // Desired speed (tune this value)

        // --- Apply Dynamic Modulation Matrix (DMM) ---
        double lambda = 1.0;
        if (last_obstacle_dist_ < safe_distance_) {
            lambda = 1.0 - (last_obstacle_dist_ * last_obstacle_dist_) / (safe_distance_ * safe_distance_);
        }

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d nnT = last_obstacle_normal_ * last_obstacle_normal_.transpose();
        Eigen::Matrix3d M = I - (1.0 - lambda) * nnT;

        Eigen::Vector3d v_mod = M * v_d;  // Apply DMM to desired velocity
     RCLCPP_INFO(this->get_logger(), "Modulated Velocity Vector: [%.2f, %.2f, %.2f]", 
           v_mod.x(), v_mod.y(), v_mod.z());
;
        double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
        // Compute PID-based thrust
        //double thrust_x = -pid_submarine_x_.compute(v_mod.x(), dt);
        double thrust_x= -v_mod.x();
        thrust_x = std::clamp(thrust_x, -5.0, 5.0);
/*
        if (std::abs(v_mod.x()) <= 0.8) {
            thrust_x = 0;
        }
*/
        // Publish Modified Thrust
        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data = thrust_x;
        pub_submarine_thrust_->publish(thrust_msg);

        RCLCPP_INFO(this->get_logger(), "Modulated Thrust X: [%.2f]", thrust_x);

        // --- Yaw Control for Orientation Correction ---
      //  double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
       // RCLCPP_INFO(this->get_logger(),"Current Yaw:[%.2f]",current_yaw);
        double desired_angle = std::atan2(v_mod.y(), v_mod.x());
        double yaw_error = desired_angle - current_yaw;

        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

       // double fin_position = pid_submarine_yaw_.compute(yaw_error, dt);
       double fin_position=1.2*v_mod.y();
        fin_position = std::clamp(fin_position, -6.28, 6.28);

        // Publish Fin Control
        std_msgs::msg::Float64 fin_msg;
        fin_msg.data = -fin_position;
        pub_submarine_fin_->publish(fin_msg);

       RCLCPP_INFO(this->get_logger(), " Fin Position: [%.2f]",  fin_position);
    }
};

//  Main Function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
