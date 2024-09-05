#include <random>  
#include <chrono>  
#include <thread>  
#include <rclcpp/rclcpp.hpp>  
#include <geometry_msgs/msg/pose.hpp>  
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
// #include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
  
using namespace std::chrono_literals;  
  
class RandomTargetPublisher : public rclcpp::Node  
{  
public:  
    RandomTargetPublisher()  
    : Node("random_target_publisher") 
    {  
        RCLCPP_INFO(this->get_logger(), "****");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        slide_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("person_goal_pose_angular", 10, std::bind(&RandomTargetPublisher::slidesubscribecallback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RandomTargetPublisher::cmdvelsubscribecallback, this, std::placeholders::_1));
    }  
  
private:  
    void cmdvelsubscribecallback(geometry_msgs::msg::Twist::SharedPtr msg){
        vel_x = msg->linear.x;
    }
    void slidesubscribecallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {  
        // pose_x = msg->position.x;
        // pose_y = msg->position.y;
        double r, p, y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3(q).getRPY(r, p, y);
        double pose_z = y;

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                "map", "base_link",
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
            }
        // RCLCPP_INFO(rclcpp::get_logger("tf"), "tf: %f, %f", t.transform.translation.x,t.transform.translation.y);
        // robot_x = t.transform.translation.x;
        // robot_y = t.transform.translation.y;
        geometry_msgs::msg::Quaternion q_ = t.transform.rotation;
        Eigen::Quaterniond eigen_q(q_.w, q_.x, q_.y, q_.z);
        double yaw = atan2(2.0 * (eigen_q.w() * eigen_q.z() + eigen_q.x() * eigen_q.y()),  
                        1.0 - 2.0 * (eigen_q.y() * eigen_q.y() + eigen_q.z() * eigen_q.z())); 
        difference = pose_z - yaw;
        if (difference < -M_PI) {
        difference = 2 * M_PI + difference;
        } 
        if (difference > M_PI) {  
        difference = -2 * M_PI + difference;  
        } 
        auto msg_twist =  geometry_msgs::msg::Twist();
        if(vel_x < 0.05){
            msg_twist.linear.x = 0;
            msg_twist.angular.z = difference  ;
            cmd_vel_pub_->publish(msg_twist);
        }
        
        // RCLCPP_INFO(rclcpp::get_logger("distance"), "distance in goal theta: %f", pose_z);
    }    
    double difference;       
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slide_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    double vel_x;

};  
  
int main(int argc, char * argv[])  
{  
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<RandomTargetPublisher>();  
    rclcpp::spin(node);  
    rclcpp::shutdown();  
    return 0;  
}