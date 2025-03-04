#include "vesc_ackermann/vesc_to_odom.hpp"
#include <cmath>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>


namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
    : Node("vesc_to_odom_node", options),
      odom_frame_("odom"),
      base_frame_("base_link"),
      publish_tf_(true), // Always publish TF
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      current_speed_(0.0),
      current_angular_velocity_(0.0),
      initial_yaw_set_(false),
      initial_yaw_(0.0)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  //BP subscribe to yaw from arduino
  yaw_sub_ = create_subscription<std_msgs::msg::Float64>("yaw", 10, std::bind(&VescToOdom::yawCallback, this, _1));

  //BP subscribe to current_speed from arduino
  speed_sub_ = create_subscription<std_msgs::msg::Float64>("xvel", 10, std::bind(&VescToOdom::speedCallback, this, _1));
        

  // Set the update rate to 10 Hz (0.1 seconds)
  update_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&VescToOdom::updateCallback, this));
}

// BP Define callback functions for each message
void VescToOdom::yawCallback(const std_msgs::msg::Float64::SharedPtr msg) {

  double new_yaw = msg->data * (M_PI / 180.0); // Convert from degrees to radians

  // Set the initial yaw on the first callback to align yaw to 0 when heading straight
  if (!initial_yaw_set_) {
    initial_yaw_ = new_yaw;
    initial_yaw_set_ = true;
  }

  // Adjust yaw by subtracting the initial yaw to align the forward direction to 0
  double adjusted_yaw = new_yaw - initial_yaw_;


  // Track time for angular velocity
  static double old_yaw = new_yaw;
  static rclcpp::Time old_time = this->now();  // Initialize time

  // Get current time and calculate time difference (dt)
  rclcpp::Time current_time = this->now();
  double dt = (current_time - old_time).seconds();

  // Ensure dt is valid and not zero (or too small)
  if (dt <= 0.0) {
    dt = 1e-6;  // Set a small value to prevent division by zero
  }

  /// Calculate change in yaw (delta yaw) and angular velocity
  double delta_yaw = adjusted_yaw - old_yaw;

  // Optionally wrap yaw to ensure it stays within the range [-π, π]
  delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));

  // Update angular velocity based on delta_yaw and dt
  current_angular_velocity_ = delta_yaw / dt;

  // Update old values
  old_yaw = adjusted_yaw;
  old_time = current_time;

  // Update the class variable for yaw to use in other functions
  yaw_ = adjusted_yaw;

}
//BP
void VescToOdom::speedCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  current_speed_ = msg->data;
}

//BP
void VescToOdom::updateCallback()
{
  // Static variables to track the last update time
  static rclcpp::Time last_time = this->now();  // Initialize with current time

  // Get the current time
  rclcpp::Time current_time = this->now();

  // Calculate the time difference (dt) in seconds
  double dt = (current_time - last_time).seconds();  // dt is in seconds

  // Ensure that dt is valid and not zero (or too small)
  if (dt == 0.0) {
    dt = 1e-6;  // Set a small value to prevent division by zero
  }

  // Calculate linear velocities (based on the current speed and yaw)
  double x_dot = current_speed_ * cos(yaw_);
  double y_dot = current_speed_ * sin(yaw_);

  // Ensure current_speed and yaw_ are valid (not NaN or infinity)
  if (std::isnan(current_speed_) || std::isnan(yaw_) || std::isinf(current_speed_) || std::isinf(yaw_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid values: current_speed or yaw is NaN/inf.");
    return;  // Skip the update if invalid values are detected
  }

  // Update position using the calculated velocities and time step dt
  x_ += x_dot * dt;
  y_ += y_dot * dt;

  // Log the updated positions for debugging
  RCLCPP_INFO(this->get_logger(), "Updated position: x_ = %f, y_ = %f", x_, y_);

  // Update last_time to current_time for the next iteration
  last_time = current_time;
    
  // Publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = current_time;  // Use the current time for the odom message
  odom.child_frame_id = base_frame_;

  // Set the position (x, y) and orientation (yaw converted to quaternion)
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);  // Convert yaw to quaternion
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);


  // Set the linear and angular velocities
  odom.twist.twist.linear.x = current_speed_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity_;  // Angular velocity from yaw
  
  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = current_time;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
  
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
