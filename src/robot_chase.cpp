#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node {
public:
    RobotChase() : rclcpp::Node::Node("robot_chase") {

        target_frame_ = this->declare_parameter<std::string>("target_frame", "morty/base_link");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);
        timer_ = this->create_wall_timer(250ms, [this](){return this->on_timer();});
    }
private:
    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    const float kp_yaw_ = 0.7;
    const float kp_distance_ = 0.7; 
    const float max_linear_velocity_ = 0.5;
    const float max_angular_velocity_ = 1.0;

    
    void on_timer() {
    
        std::string fromFrame = target_frame_.c_str();
        std::string toFrame = "rick/base_link";

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform( toFrame, fromFrame, tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in transformation: %s", ex.what());
            return;
        }
        
        auto x = tf.transform.translation.x;
        auto y = tf.transform.translation.y;
        RCLCPP_DEBUG(this->get_logger(), "X: %.2f, Y: %.2f, Z: %.2f.", x, y, tf.transform.translation.z);

        float error_yaw = kp_yaw_ * std::atan2(y, x);
        float error_distance = kp_distance_ * std::sqrt(pow(x, 2) + pow(y, 2));
        float min_linear_velocity_ = (0.5 * error_distance);
        RCLCPP_DEBUG(this->get_logger(), "Error distance: %.4f", error_distance);
        geometry_msgs::msg::Twist velocity;
        if (error_distance > 0.15) {
            velocity.angular.z = std::min(error_yaw, max_angular_velocity_);
            velocity.linear.x = std::min(error_distance, max_linear_velocity_);
        }
        else if (error_distance < 0.15 && error_distance > 0.1){
            velocity.angular.z = error_yaw;
            velocity.linear.x = min_linear_velocity_;
        }
        else {
            velocity.angular.z = 0;
            velocity.linear.x = 0;
        }
        publisher_->publish(velocity);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotChase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}