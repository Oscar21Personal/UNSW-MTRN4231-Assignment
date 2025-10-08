#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class InventoryTransformations : public rclcpp::Node
{
public:
  InventoryTransformations() : Node("inventory_transformations")
  {
    // Create a static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf2::Quaternion q;

    // Fill out inventory_slot_1 message
    transformStamped1_.header.frame_id = "base_link";
    transformStamped1_.child_frame_id = "inventory_slot_1";
    transformStamped1_.transform.translation.x = 0.25;
    transformStamped1_.transform.translation.y = 0.25;
    transformStamped1_.transform.translation.z = 0.0;
    q.setRPY(0.0, 0.0, M_PI / 4.0);
    transformStamped1_.transform.rotation.x = q.x();
    transformStamped1_.transform.rotation.y = q.y();
    transformStamped1_.transform.rotation.z = q.z();
    transformStamped1_.transform.rotation.w = q.w();
    // Broadcast inventory_slot_1 transformation
    tf_static_broadcaster_->sendTransform(transformStamped1_);

    // Fill out inventory_slot_2 message
    transformStamped2_.header.frame_id = "base_link";
    transformStamped2_.child_frame_id = "inventory_slot_2";
    transformStamped2_.transform.translation.x = 0.25;
    transformStamped2_.transform.translation.y = 0.0;
    transformStamped2_.transform.translation.z = 0.0;
    q.setRPY(0.0, 0.0, 0.0);
    transformStamped2_.transform.rotation.x = q.x();
    transformStamped2_.transform.rotation.y = q.y();
    transformStamped2_.transform.rotation.z = q.z();
    transformStamped2_.transform.rotation.w = q.w();
    // Broadcast inventory_slot_2 transformation
    tf_static_broadcaster_->sendTransform(transformStamped2_);

    // Fill out inventory_slot_3 message
    transformStamped3_.header.frame_id = "base_link";
    transformStamped3_.child_frame_id = "inventory_slot_3";
    transformStamped3_.transform.translation.x = 0.25;
    transformStamped3_.transform.translation.y = -0.25;
    transformStamped3_.transform.translation.z = 0.0;
    q.setRPY(0.0, 0.0, -M_PI / 4.0);
    transformStamped3_.transform.rotation.x = q.x();
    transformStamped3_.transform.rotation.y = q.y();
    transformStamped3_.transform.rotation.z = q.z();
    transformStamped3_.transform.rotation.w = q.w();
    // Broadcast inventory_slot_3 transformation
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
  rclcpp::spin(std::make_shared<InventoryTransformations>());
  rclcpp::shutdown();
  return 0;
}
