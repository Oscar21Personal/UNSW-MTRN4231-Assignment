#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class BrainTransformations : public rclcpp::Node
{
public:
  BrainTransformations() : Node("brain_transformations")
  {
    // Create a static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf2::Quaternion q;

    // Fill out map to base link message
    transformStamped1_.header.frame_id = "map";
    transformStamped1_.child_frame_id = "base_link";
    transformStamped1_.transform.translation.x = 1.0;
    transformStamped1_.transform.translation.y = 1.0;
    transformStamped1_.transform.translation.z = 0.0;
    q.setRPY(0.0, 0.0, 0.0);
    transformStamped1_.transform.rotation.x = q.x();
    transformStamped1_.transform.rotation.y = q.y();
    transformStamped1_.transform.rotation.z = q.z();
    transformStamped1_.transform.rotation.w = q.w();
    // Broadcast base link transformation
    tf_static_broadcaster_->sendTransform(transformStamped1_);

    // Fill out base link to arm link message
    transformStamped2_.header.frame_id = "base_link";
    transformStamped2_.child_frame_id = "arm_link";
    transformStamped2_.transform.translation.x = 0.1;
    transformStamped2_.transform.translation.y = 0.0;
    transformStamped2_.transform.translation.z = 0.2;
    q.setRPY(0.0, 0.0, 0.0);
    transformStamped2_.transform.rotation.x = q.x();
    transformStamped2_.transform.rotation.y = q.y();
    transformStamped2_.transform.rotation.z = q.z();
    transformStamped2_.transform.rotation.w = q.w();
    // Broadcast arm link transformation
    tf_static_broadcaster_->sendTransform(transformStamped2_);

    // Fill out base link to camera link message
    transformStamped3_.header.frame_id = "base_link";
    transformStamped3_.child_frame_id = "camera_link";
    transformStamped3_.transform.translation.x = -0.5;
    transformStamped3_.transform.translation.y = 0.0;
    transformStamped3_.transform.translation.z = 0.5;
    q.setRPY(0.0, M_PI / 6.0, 0.0);
    transformStamped3_.transform.rotation.x = q.x();
    transformStamped3_.transform.rotation.y = q.y();
    transformStamped3_.transform.rotation.z = q.z();
    transformStamped3_.transform.rotation.w = q.w();
    // Broadcast camera link transformation
    tf_static_broadcaster_->sendTransform(transformStamped3_);

  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped transformStamped1_;
  geometry_msgs::msg::TransformStamped transformStamped2_;
  geometry_msgs::msg::TransformStamped transformStamped3_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrainTransformations>());
  rclcpp::shutdown();
  return 0;
}
