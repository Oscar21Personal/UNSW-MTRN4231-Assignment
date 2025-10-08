#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/msg/camera.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class Perception : public rclcpp::Node
{
public:

  Perception() : Node("perception")
  {
    // Create camera subscriber
    subscription_ = this->create_subscription<interfaces::msg::Camera>("camera", 10, std::bind(&Perception::topic_callback, this, std::placeholders::_1));
    // Create timer
    timer_ = create_wall_timer(1000ms, std::bind(&Perception::timer_callback, this));
    // Create dynamic transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Create perception status publisher
    statusPublisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("perception_status", 10);
  }

private:

  void topic_callback(const interfaces::msg::Camera& msg)
  {
    // Check if item_id is in the array
    bool containItemID = false;
    int index = -1;
    int length = static_cast<int>(cameraMsgArray_.size());
    for (int i = 0; i < length; i++) {
      if (cameraMsgArray_[i].item_id == msg.item_id) {
        containItemID = true;
        index = i;
      }
    }
    // Update camera message array, item ID array, and timer array
    if (containItemID) {
      cameraMsgArray_[index].x = msg.x;
      cameraMsgArray_[index].y = msg.y;
      cameraMsgArray_[index].z = msg.z;
      timerArray_[index] = 0;
    }
    else {
      cameraMsgArray_.push_back(msg);
      timerArray_.push_back(0);
      itemIDArray_.data.push_back(msg.item_id);
    }
  }

  void timer_callback()
  {
    // Check if any item_id removed after 5s
    int removeIndex = -1;
    for (int i = 0; i < static_cast<int>(timerArray_.size()); i++) {
      timerArray_[i]++;
      if (timerArray_[i] > 5) {
        removeIndex = i;
      }
    }

    // Remove non-existing item_id
    if (removeIndex != -1) {
      cameraMsgArray_.erase(cameraMsgArray_.begin() + removeIndex);
      itemIDArray_.data.erase(itemIDArray_.data.begin() + removeIndex);
      timerArray_.erase(timerArray_.begin() + removeIndex);
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.frame_id = "camera_link";
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0); 
    transform_msg.transform.rotation.x = q.x();
    transform_msg.transform.rotation.y = q.y();
    transform_msg.transform.rotation.z = q.z();
    transform_msg.transform.rotation.w = q.w();

    // Boardcast all camera messages
    for (const auto& elem : cameraMsgArray_) {
      
      transform_msg.child_frame_id = std::to_string(elem.item_id);
      transform_msg.transform.translation.x = elem.x;
      transform_msg.transform.translation.y = elem.y;
      transform_msg.transform.translation.z = elem.z;
      
      tf_broadcaster_->sendTransform(transform_msg);
    }

    // Publish perception status
    statusPublisher_->publish(itemIDArray_);
  }

  rclcpp::Subscription<interfaces::msg::Camera>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<interfaces::msg::Camera> cameraMsgArray_;
  std_msgs::msg::Int32MultiArray itemIDArray_;
  std::vector<int> timerArray_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr statusPublisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Perception>());
  rclcpp::shutdown();
  return 0;
}
