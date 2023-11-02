// #include <ros/ros.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <turtlesim/Pose.h>

// // std::string turtle_name;

// void staticCallback(const geometry_msgs::TransformStamped& msg){
//   static tf2_ros::StaticTransformBroadcaster br_static;
//   geometry_msgs::TransformStamped transformStamped;
  
//   transformStamped.header.stamp = ros::Time::now();
//   transformStamped.header.frame_id = msg->header.frame_id;
//   transformStamped.child_frame_id = msg->header.child_frame_id;
//   transformStamped.transform.translation.x = msg->transform.translation.x;
//   transformStamped.transform.translation.y = msg->transform.translation.y;
//   transformStamped.transform.translation.z = msg->transform.translation.y;
//   transformStamped.transform.rotation.x = msg->transform.rotation.x;
//   transformStamped.transform.rotation.y = msg->transform.rotation.y;
//   transformStamped.transform.rotation.z = msg->transform.rotation.z;
//   transformStamped.transform.rotation.w = msg->transform.rotation.w;

//   br_static.sendTransform(transformStamped);
// }

// int main(int argc, char** argv){
//   ros::init(argc, argv, "my_tf2_broadcaster");

//   ros::NodeHandle private_node("~");
//   // if (! private_node.hasParam("turtle"))
//   // {
//   //   if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
//   //   turtle_name = argv[1];
//   // }
//   // else
//   // {
//   //   private_node.getParam("turtle", turtle_name);
//   // }
    
//   ros::NodeHandle node;
//   ros::Subscriber sub = node.subscribe("/utf_static", 10, &staticCallback);
//   ROS_INFO("Spinning until killed publishing Unity Static Transforms...");
//   ros::spin();
//   return 0;
// };

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class UTFC : public rclcpp::Node
{
public:
  UTFC()
  : Node("unity_transform_connector")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/utf_static", 10,
      std::bind(&UTFC::staticCallback, this, std::placeholders::_1));
  }

private:
  void staticCallback(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id = msg->child_frame_id;

    t.transform.translation.x = msg->transform.translation.x;
    t.transform.translation.y = msg->transform.translation.y;
    t.transform.translation.z = msg->transform.translation.z;
    t.transform.rotation.x = msg->transform.rotation.x;
    t.transform.rotation.y = msg->transform.rotation.y;
    t.transform.rotation.z = msg->transform.rotation.z;
    t.transform.rotation.w = msg->transform.rotation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UTFC>());
  rclcpp::shutdown();
  return 0;
}