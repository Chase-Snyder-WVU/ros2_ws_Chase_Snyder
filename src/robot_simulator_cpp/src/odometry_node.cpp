/**
 *  @file odometry_node.cpp
 *  @author Chase Snyder
 *  
 *  @brief Cpp Odometry tf publisher node for calculating robot
 *         robot pose and publishing velocity using ROS 2 Humble.
 *
 */

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "custom_interfaces/srv/reset_position.hpp"

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode()
  : Node("odometry_node"),
    // initial pose positions declared in the constructor
    x_(0.0),
    y_(0.0)
  {
    //Intialize last_time_
    last_time_ = this->get_clock()->now();

    //Initialize the transform broadcaster (odom to base_link)
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Subscribe to /cmd_vel and call handle_command_velocity on each message
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&OdometryNode::handle_command_velocity, this, std::placeholders::_1));

    //Create ResetPosition service to reset the integrated pose
    reset_service_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "/ResetPosition",
      std::bind(&OdometryNode::handle_reset, this,
                std::placeholders::_1, std::placeholders::_2));

    //Public display for test calls
    RCLCPP_INFO(this->get_logger(), "odometry_node started");

    // Publish initial transform at (0,0) to start 
    // controller_node
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = last_time_;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

private:

  //Called every time a Twist message arrives on /cmd_vel
  void handle_command_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    //Current time from the node's clock
    rclcpp::Time now = this->get_clock()->now();

    //Compute time step in seconds
    double dt = (now - last_time_).seconds();
    //update global variable for last_time_
    last_time_ = now;

    //check for non-positive dt to prevent errors
    if (dt <= 0.0) {
      return;
    }

    //Integrate position from linear velocity
    //Simple mobile robot mode (no differential drive)
    //move along x-axis and y-axos in odom frame, ignore rotation
    x_ += msg->linear.x * dt;
    y_ += msg->linear.y *dt;
    
    ///////////////////////////////
    // Publish updated transform //
    geometry_msgs::msg::TransformStamped t;

    //Time and frame information
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";

    //2D position in odom frame
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    //identity quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    //Send the transformation
    tf_broadcaster_->sendTransform(t);
    //        End Publish         //
    ////////////////////////////////
  }

  //Service callback to reset pose for requested positions
  void handle_reset(
    const std::shared_ptr<custom_interfaces::srv::ResetPosition::Request> request,
    std::shared_ptr<custom_interfaces::srv::ResetPosition::Response> response)
  {
    //Set internal state from the request pose (only x and y used)
    x_ = request->pose.position.x;
    y_ = request->pose.position.y;

    //Sync time so for integration dt
    rclcpp::Time now = this->get_clock()->now();
    //update global variable for last_time_
    last_time_ = now;

    //print positon
    RCLCPP_INFO(this->get_logger(),
      "ResetPosition: x = %.2f, y = %.2f", x_, y_);

    //Update service response
    response->success = true;

    ///////////////////////////////
    // Publish updated transform //
    geometry_msgs::msg::TransformStamped t;
    
    //Time and frame information
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";

    //2D position in odom frame
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    //identity quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    //Send the transformation
    tf_broadcaster_->sendTransform(t);
    //        End Publish         //
    ////////////////////////////////
  }

  //Integrated pose (2D) file-scoped variables
  double x_;
  double y_;

  //Time keeping for integration
  rclcpp::Time last_time_;

  //ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}