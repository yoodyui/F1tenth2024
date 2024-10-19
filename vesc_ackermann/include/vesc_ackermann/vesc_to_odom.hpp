#ifndef VESC_ACKERMANN__VESC_TO_ODOM_HPP_
#define VESC_ACKERMANN__VESC_TO_ODOM_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/time.hpp>
#include <chrono>

namespace vesc_ackermann
{

using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;

class VescToOdom : public rclcpp::Node
{
public:
  explicit VescToOdom(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;

  /// Odometry state
  double x_;
  double y_;
  double yaw_;

  // BP: Additional variables
  double current_speed_;             // Current speed
  double current_angular_velocity_;  // Current angular velocity
  bool initial_yaw_set_;             // Flag to check if initial yaw is set
  double initial_yaw_;               // Initial yaw angle
  
  // ROS services
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  //BP
  rclcpp::Subscription<Float64>::SharedPtr yaw_sub_;
  rclcpp::Subscription<Float64>::SharedPtr speed_sub_;
  // Time
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Time last_state_time_;
  // publish the subscribe yaw and xvel
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;


  // ROS callbacks
  void updateCallback();
  void yawCallback(const Float64::SharedPtr msg);
  void speedCallback(const Float64::SharedPtr msg);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_HPP_
