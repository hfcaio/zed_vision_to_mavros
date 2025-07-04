#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace rclcpp;

class ZedConversion : public Node
{
public:
  ZedConversion() : Node("zed_vision_to_mavros")
  {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    mavros_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/vision_pose/pose", 10);
    zed_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", 10, std::bind(&ZedConversion::zed_callback, this, std::placeholders::_1));
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pub_;
  Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr zed_sub_;

  void zed_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped mavros_msg;
    const std::string target_frame = "odom";               // frame fixo
    const std::string source_frame = msg->header.frame_id; // provavelmente "zed_camera_center"
    try
    {
      if (!tf_buffer_->canTransform(source_frame, target_frame, tf2::TimePointZero))
      {
        RCLCPP_ERROR_ONCE(this->get_logger(),
                          "Could not get transform from %s to %s",
                          source_frame.c_str(), target_frame.c_str());
        return;
      }
      tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(*msg, mavros_msg, target_frame, tf2::Duration(std::chrono::seconds(1)));
      mavros_msg.header.frame_id = target_frame;          // set the frame_id to the target frame
      mavros_msg.header.stamp = this->get_clock()->now(); // update the timestamp
      mavros_pub_->publish(mavros_msg);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Could not transform zed pose to mavros pose: %s", ex.what());
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedConversion>());
  rclcpp::shutdown();
  return 0;
}
