#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
  public:
    TestNode()
    : Node("iw_navigation")
    {
      goal1.header.frame_id = "odom_vslam";
      goal1.pose.position.x = 3;
      goal1.pose.position.y = 3;
      goal1.pose.position.z = 0;
      goal1.pose.orientation.x = 0;
      goal1.pose.orientation.y = 0;
      goal1.pose.orientation.z = 0;
      goal1.pose.orientation.w = 1;

      goal2.header.frame_id = "odom_vslam";
      goal2.pose.position.x = -3;
      goal2.pose.position.y = -3;
      goal2.pose.position.z = 0;
      goal2.pose.orientation.x = 0;
      goal2.pose.orientation.y = 0;
      goal2.pose.orientation.z = 0;
      goal2.pose.orientation.w = 1;

      index = 0;
      reachedGoal = true;
      threshold = 0.5;
      navRequest.store(false);

      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/visual_slam/vis/slam_odometry", 10, 
          std::bind(&TestNode::PoseCallback, this, _1));
      subscription_clock_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", 10, 
          std::bind(&TestNode::publishMap2Odom, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", 10);
      service_ = this->create_service<std_srvs::srv::SetBool>(
          "set_navigation",
          std::bind(&TestNode::NavRequestCallback, this,
          std::placeholders::_1, std::placeholders::_2));
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      
      goal_pose_timer_ = this->create_wall_timer(
          3s, std::bind(&TestNode::GoalPoseCallback, this));
    }

  private:
    void publishMap2Odom(const rosgraph_msgs::msg::Clock& curTime)
    {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped transform;
        tf2::TimePoint transform_expiration = tf2_ros::fromMsg(curTime.clock) + tf2::durationFromSec(0.0);
        transform.header.stamp = tf2_ros::toMsg(transform_expiration);
        RCLCPP_INFO(this->get_logger(), "Published transform for time stamp %d" , transform.header.stamp.sec);
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";

        // Set translation (x, y, z)
        transform.transform.translation.x = -6.36564; // Example translation
        transform.transform.translation.y = 1.87924; // Example translation
        transform.transform.translation.z = 0.0;

        // Set rotation (x, y, z, w)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0); // Roll, Pitch, Yaw
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Send the transform
        tf_broadcaster_->sendTransform(transform);
        // RCLCPP_INFO(this->get_logger(), "Published transform from map to odom");
    }
    void NavRequestCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request received: %s", request->data ? "true" : "false");

        // Process the request (e.g., just echoing the value back)
        navRequest.store(request->data);
        response->success = true; // You can set this based on your logic
        response->message = request->data ? "Start navigation" : "Stop navigation";

        RCLCPP_INFO(this->get_logger(), "Response sent: %s", response->message.c_str());
    }

    void PoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if (!navRequest.load()) return;
      std::lock_guard<std::mutex> lock(poseMutex);
      // RCLCPP_INFO(this->get_logger(), 
      // "received pose: %f, %f, %f",
      // msg->pose.pose.position.x,
      // msg->pose.pose.position.y,
      // msg->pose.pose.position.z);
      if (reachedGoal)
        return;

      if (index == 0 && std::fabs(msg->pose.pose.position.x - goal1.pose.position.x) < threshold && 
          std::fabs(msg->pose.pose.position.y - goal1.pose.position.y) < threshold &&
          std::fabs(msg->pose.pose.position.z - goal1.pose.position.z) < threshold)
      {
        RCLCPP_INFO(this->get_logger(), "Reached goal pose1");
        reachedGoal = true;
        index = 1;
      }
      else if (index == 1 && std::fabs(msg->pose.pose.position.x - goal2.pose.position.x) < threshold && 
          std::fabs(msg->pose.pose.position.y - goal2.pose.position.y) < threshold &&
          std::fabs(msg->pose.pose.position.z - goal2.pose.position.z) < threshold)
      {
        RCLCPP_INFO(this->get_logger(), "Reached goal pose2");
        reachedGoal = true;
        index = 0;
      }
    }
    void GoalPoseCallback()
    {
      if (!navRequest.load()) return;
      std::lock_guard<std::mutex> lock(poseMutex);
      if (!reachedGoal)
      {
        return;
      }

      if (index == 0)
      {
        RCLCPP_INFO(this->get_logger(), "Publishing: goal pose1");
        publisher_->publish(goal1);
        reachedGoal = false;
      }
      else if (index == 1)
      {
        RCLCPP_INFO(this->get_logger(), "Publishing: goal pose2");
        publisher_->publish(goal2);
        reachedGoal = false;
      }
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_clock_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr goal_pose_timer_;
    geometry_msgs::msg::PoseStamped goal1, goal2;
    int index;
    bool reachedGoal;
    float threshold;
    std::mutex poseMutex;
    std::atomic<bool> navRequest;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());

  rclcpp::shutdown();
  return 0;
}