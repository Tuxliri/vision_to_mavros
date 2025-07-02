#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mavros_msgs/msg/landing_target.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vision_to_mavros");

  // Parameters
  std::string target_frame_id = node->declare_parameter<std::string>("target_frame_id", "camera_frame");
  std::string source_frame_id = node->declare_parameter<std::string>("source_frame_id", "camera_link");
  double output_rate = node->declare_parameter<double>("output_rate", 20.0);
  double roll_cam = node->declare_parameter<double>("roll_cam", 0.0);
  double pitch_cam = node->declare_parameter<double>("pitch_cam", 0.0);
  double yaw_cam = node->declare_parameter<double>("yaw_cam", 1.5707963);
  double gamma_world = node->declare_parameter<double>("gamma_world", -1.5707963);

  bool enable_precland = node->declare_parameter<bool>("enable_precland", false);
  std::string precland_target_frame_id = node->declare_parameter<std::string>("precland_target_frame_id", "landing_target");
  std::string precland_camera_frame_id = node->declare_parameter<std::string>("precland_camera_frame_id", "camera_fisheye2_optical_frame");

  auto camera_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose", 10);
  auto body_path_pubisher = node->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);
  rclcpp::Publisher<mavros_msgs::msg::LandingTarget>::SharedPtr precland_msg_publisher;
  if (enable_precland) {
    precland_msg_publisher = node->create_publisher<mavros_msgs::msg::LandingTarget>("landing_raw", 10);
  }

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::TransformBroadcaster tf_broadcaster(node);

  geometry_msgs::msg::PoseStamped msg_body_pose;
  nav_msgs::msg::Path body_path;

  rclcpp::Time last_tf_time = node->now();
  rclcpp::Time last_precland_tf_time = node->now();

  rclcpp::Rate rate(output_rate);

  while (rclcpp::ok()) {
    // Use latest available transform
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer.lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero);
      if (last_tf_time < transform.header.stamp) {
        last_tf_time = transform.header.stamp;

        tf2::Vector3 position_orig(transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   transform.transform.translation.z);
        tf2::Vector3 position_body;
        position_body.setX(std::cos(gamma_world) * position_orig.x() + std::sin(gamma_world) * position_orig.y());
        position_body.setY(-std::sin(gamma_world) * position_orig.x() + std::cos(gamma_world) * position_orig.y());
        position_body.setZ(position_orig.z());

        tf2::Quaternion quat_cam(transform.transform.rotation.x,
                                 transform.transform.rotation.y,
                                 transform.transform.rotation.z,
                                 transform.transform.rotation.w);
        tf2::Quaternion quat_cam_to_body, quat_rot_z, quat_body;

        // camera → body (intrinsic R-P-Y = X-Y-Z)
        quat_cam_to_body.setRPY(roll_cam, pitch_cam, yaw_cam);
        // world-frame Z correction
        quat_rot_z.setRPY(0, 0, -gamma_world);
        // final orientation of the body frame, expressed in <target_frame_id>
        quat_body = quat_rot_z * quat_cam * quat_cam_to_body;
        quat_body.normalize();

        msg_body_pose.header.stamp = transform.header.stamp;
        msg_body_pose.header.frame_id = transform.header.frame_id;
        msg_body_pose.pose.position.x = position_body.x();
        msg_body_pose.pose.position.y = position_body.y();
        msg_body_pose.pose.position.z = position_body.z();
        msg_body_pose.pose.orientation.x = quat_body.x();
        msg_body_pose.pose.orientation.y = quat_body.y();
        msg_body_pose.pose.orientation.z = quat_body.z();
        msg_body_pose.pose.orientation.w = quat_body.w();

        camera_pose_publisher->publish(msg_body_pose);

        body_path.header.stamp = msg_body_pose.header.stamp;
        body_path.header.frame_id = msg_body_pose.header.frame_id;
        body_path.poses.push_back(msg_body_pose);
        body_path_pubisher->publish(body_path);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = transform.header.stamp;
        tf_msg.header.frame_id = transform.header.frame_id;
        tf_msg.child_frame_id = "body_frame";
        tf_msg.transform.translation.x = position_body.x();
        tf_msg.transform.translation.y = position_body.y();
        tf_msg.transform.translation.z = position_body.z();
        tf_msg.transform.rotation.x = quat_body.x();
        tf_msg.transform.rotation.y = quat_body.y();
        tf_msg.transform.rotation.z = quat_body.z();
        tf_msg.transform.rotation.w = quat_body.w();
        tf_broadcaster.sendTransform(tf_msg);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node->get_logger(), "%s", ex.what());
    }

    if (enable_precland) {
      if (tf_buffer.canTransform(precland_camera_frame_id, precland_target_frame_id, tf2::TimePointZero)) {
        transform = tf_buffer.lookupTransform(precland_camera_frame_id, precland_target_frame_id, tf2::TimePointZero);
        if (last_precland_tf_time < transform.header.stamp) {
          last_precland_tf_time = transform.header.stamp;

          mavros_msgs::msg::LandingTarget msg_landing_target;
          msg_landing_target.header.frame_id = transform.header.frame_id;
          msg_landing_target.header.stamp = transform.header.stamp;
          msg_landing_target.target_num = 0;
          msg_landing_target.frame = mavros_msgs::msg::LandingTarget::LOCAL_NED;
          msg_landing_target.type = mavros_msgs::msg::LandingTarget::VISION_FIDUCIAL;
          msg_landing_target.angle[0] = std::atan2(transform.transform.translation.x, transform.transform.translation.z);
          msg_landing_target.angle[1] = std::atan2(transform.transform.translation.y, transform.transform.translation.z);
          msg_landing_target.distance = std::sqrt(std::pow(transform.transform.translation.x,2)+std::pow(transform.transform.translation.y,2)+std::pow(transform.transform.translation.z,2));
          precland_msg_publisher->publish(msg_landing_target);
          RCLCPP_INFO(node->get_logger(), "Landing target detected");
        }
      }
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

