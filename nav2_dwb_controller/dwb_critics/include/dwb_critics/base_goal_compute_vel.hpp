#ifndef DWB_CRITICS__BASE_GOAL_COMPUTE_VEL_HPP_
#define DWB_CRITICS__BASE_GOAL_COMPUTE_VEL_HPP_

#include <string>
#include <vector>
#include <deque>
#include "dwb_core/trajectory_critic.hpp"
#include <rclcpp/rclcpp.hpp>  
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/pose.hpp>  
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace dwb_critics
{
class BaseGoalComputeVelCritic : public dwb_core::TrajectoryCritic
{
public:
    void onInit() override;
    bool prepare(
        const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
        const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
    double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
    double predict_person_vel_time;
    double max_distance_,min_x_,nav_x_,max_xt_,min_distance_,max_x_;
    double difference;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double predict_person_vel, predict_person_distance;
    double pose_x, pose_y, pose_z;
    double robot_x, robot_y;
    double previous_goal_x;
    double previous_goal_y;
    // std::queue<double> goal_messages;
    rclcpp::Time start_time_;
    bool update_time;
    double next_time;
    bool follow_person_ = false;
    void followingpersonsubscribecallback(const std_msgs::msg::Bool::SharedPtr msg)
    {  
        if(msg->data){
            follow_person_ = true;
        }
        else{
            follow_person_ = false;
        }
    }  
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slide_subscription_;
    void slidesubscribecallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {  
        pose_x = msg->position.x;
        pose_y = msg->position.y;
        double r, p, y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3(q).getRPY(r, p, y);
        pose_z = y;
        // RCLCPP_INFO(rclcpp::get_logger("distance"), "distance in goal theta: %f", pose_z);
    } 
    std::deque<double> queue_dxy;

};

}  // namespace dwb_critics
#endif  
