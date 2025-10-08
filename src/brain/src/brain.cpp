#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "interfaces/msg/brain_status.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "interfaces/srv/command.hpp"
#include "interfaces/srv/inventory.hpp"
#include "interfaces/srv/arm_movement.hpp"

using namespace std::chrono_literals;

class Brain : public rclcpp::Node
{
public:

  Brain() : Node("brain")
  {
    // Create command service
    srv_ = create_service<interfaces::srv::Command>("command", std::bind(&Brain::action, this, std::placeholders::_1, std::placeholders::_2));
    // Create perception status subscriber
    perceptionStatusSub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("perception_status", 10, std::bind(&Brain::perception_status_callback, this, std::placeholders::_1));
    // Create brain status publisher
    publisher_ = this->create_publisher<interfaces::msg::BrainStatus>("brain_status", 10);
    // Create inventory client
    inventoryClient_ = create_client<interfaces::srv::Inventory>("inventory");
    // Create arm client
    armClient_ = create_client<interfaces::srv::ArmMovement>("arm");
    // Initialise tf buffer and tf listener
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    // Initialise variables
    isBusy_ = false;
  }

private:

  void action(const std::shared_ptr<interfaces::srv::Command::Request> request, std::shared_ptr<interfaces::srv::Command::Response> response)
  {
    if (request->command != "pick" && request->command != "place") {
        // Publish brain status - malformed command
        response->accept = "malformed command";
        brainStatusMsg_.status = "UPDATE: new request " + request->command + " item " + std::to_string(request->item_id) + " - Status " + response->accept;
        publisher_->publish(brainStatusMsg_);
        return;
    }
    if (isBusy_) {
        // Publish brain status - busy
        response->accept = "busy";
        brainStatusMsg_.status = "UPDATE: new request " + request->command + " item " + std::to_string(request->item_id) + " - Status " + response->accept;
        publisher_->publish(brainStatusMsg_);
        return;
    }

    // Publish brain status - accepted
    response->accept = "accepted";
    brainStatusMsg_.status = "UPDATE: new request " + request->command + " item " + std::to_string(request->item_id) + " - Status " + response->accept;
    publisher_->publish(brainStatusMsg_);
    targetItemID_ = request->item_id;

    // Pick command
    if (request->command == "pick" && isBusy_ == false) {

        // Set status to busy
        isBusy_ = true;

        // Check item transformation
        bool flag = false;
        for (const int& elem : perceptionMsg_.data) {
            if (elem == targetItemID_) {
                flag = true;
            }
        }
        if (flag == false) {
            brainStatusMsg_.status = "ERROR: cannot find transformation to item " + std::to_string(targetItemID_);
            publisher_->publish(brainStatusMsg_);
            resetNode();
            return;
        }

        // Lookup coordinates of item and arm link
        try {
            targetFrameTransform_ = tfBuffer_->lookupTransform("base_link", std::to_string(targetItemID_), tf2::TimePointZero, tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "ERROR: cannot lookup transformation from base_link to %s", std::to_string(request->item_id).c_str());
            resetNode();
            return;
        }
        targetItemCoords_[0] = targetFrameTransform_.transform.translation.x;
        targetItemCoords_[1] = targetFrameTransform_.transform.translation.y;
        targetItemCoords_[2] = targetFrameTransform_.transform.translation.z;
        try {
            targetFrameTransform_ = tfBuffer_->lookupTransform("base_link", "arm_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "ERROR: cannot lookup transformation from base_link to arm_link");
            resetNode();
            return;
        }
        armCoords_[0] = targetFrameTransform_.transform.translation.x;
        armCoords_[1] = targetFrameTransform_.transform.translation.y;
        armCoords_[2] = targetFrameTransform_.transform.translation.z;

        // Check item is <= 1m from arm_link
        float distance = sqrt(pow(targetItemCoords_[0] - armCoords_[0], 2) + pow(targetItemCoords_[1] - armCoords_[1], 2) + pow(targetItemCoords_[2] - armCoords_[2], 2));
        if (distance > 1) {
            brainStatusMsg_.status = "ERROR: item " + std::to_string(targetItemID_ ) + " is too far";
            publisher_->publish(brainStatusMsg_);
            resetNode();
            return;
        }

        // Call put_in_inventory
        auto inventoryRequest = std::make_shared<interfaces::srv::Inventory::Request>();
        inventoryRequest->command = "put_in_inventory";
        inventoryRequest->item_id = targetItemID_ ;
        inventoryClient_->async_send_request(
            inventoryRequest,
            std::bind(&Brain::handleInventoryPickResponse, this, std::placeholders::_1)
        );

    }

    // Place command
    if (request->command == "place" && isBusy_ == false) {

        // Set status to busy
        isBusy_ = true;

        // Call get_from_inventory
        auto inventoryRequest = std::make_shared<interfaces::srv::Inventory::Request>();
        inventoryRequest->command = "get_from_inventory";
        inventoryRequest->item_id = targetItemID_ ;
        inventoryClient_->async_send_request(
            inventoryRequest,
            std::bind(&Brain::handleInventoryPlaceResponse, this, std::placeholders::_1)
        );

    }

  }

  void perception_status_callback(const std_msgs::msg::Int32MultiArray& perceptionMsg) 
  {
    perceptionMsg_ = perceptionMsg;
  }

  void handleInventoryPickResponse(rclcpp::Client<interfaces::srv::Inventory>::SharedFuture future) 
  {
    if (future.get()) {
      targetSlot_ = future.get()->response;
      // Check if inventory slots are full
      if (targetSlot_ == -1) {
        brainStatusMsg_.status = "ERROR: inventory is full";
        publisher_->publish(brainStatusMsg_);
        resetNode();
        return;
      }
      brainStatusMsg_.status = "UPDATE: added item " + std::to_string(targetItemID_) + " to inventory slot " + std::to_string(targetSlot_);
      publisher_->publish(brainStatusMsg_);

      // Call arm movement
      auto armRequest = std::make_shared<interfaces::srv::ArmMovement::Request>();
      armRequest->command = "pick";
      armRequest->x = targetItemCoords_[0] - armCoords_[0];
      armRequest->y = targetItemCoords_[1] - armCoords_[1];
      armRequest->z = targetItemCoords_[2] - armCoords_[2];
      armRequest->inventory_slot = targetSlot_;
      armClient_->async_send_request(
        armRequest,
        std::bind(&Brain::handleArmResponse, this, std::placeholders::_1)
      );
      brainStatusMsg_.status = "UPDATE: arm movement request";
      publisher_->publish(brainStatusMsg_);
      
    } else {
      RCLCPP_ERROR(get_logger(), "Inventory service call failed");
      resetNode();
    }

  }

  void handleInventoryPlaceResponse(rclcpp::Client<interfaces::srv::Inventory>::SharedFuture future) 
  {
    if (future.get()) {
      targetSlot_ = future.get()->response;
      // Check item is in inventory
      if (targetSlot_ == -1) {
        brainStatusMsg_.status = "ERROR: item " + std::to_string(targetItemID_) + " is not in inventory";
        publisher_->publish(brainStatusMsg_);
        resetNode();
        return;
      }
      brainStatusMsg_.status = "UPDATE: placing item " + std::to_string(targetItemID_) + " from inventory slot " + std::to_string(targetSlot_);
      publisher_->publish(brainStatusMsg_);

      // Call arm movement
      auto armRequest = std::make_shared<interfaces::srv::ArmMovement::Request>();
      armRequest->command = "place";
      armRequest->x = 0.5;
      armRequest->y = 0.0;
      armRequest->z = 0.0;
      armRequest->inventory_slot = targetSlot_;
      armClient_->async_send_request(
        armRequest,
        std::bind(&Brain::handleArmResponse, this, std::placeholders::_1)
      );
      brainStatusMsg_.status = "UPDATE: arm movement request";
      publisher_->publish(brainStatusMsg_);

    }
    else {
      RCLCPP_ERROR(get_logger(), "Inventory service call failed");
      resetNode();
    }
  }

  void handleArmResponse(rclcpp::Client<interfaces::srv::ArmMovement>::SharedFuture future) 
  {
    if (future.get()) {
      if (future.get()->success) {
        brainStatusMsg_.status = "UPDATE: arm movement finished";
        publisher_->publish(brainStatusMsg_);
        resetNode();
      }
      else {
        brainStatusMsg_.status = "ERROR: arm movement failed to complete";
        publisher_->publish(brainStatusMsg_);
      }
    }
    else {
      RCLCPP_ERROR(get_logger(), "Arm service call failed");
      resetNode();
    }
  }

  void resetNode() {
    isBusy_ = false;
    targetItemID_ = -1;
    targetSlot_ = -1;
    for (int i = 0; i < 3; i ++) {
      targetItemCoords_[i] = 0;
      armCoords_[i] = 0;
    }
  }

  rclcpp::Service<interfaces::srv::Command>::SharedPtr srv_;
  rclcpp::Publisher<interfaces::msg::BrainStatus>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr perceptionStatusSub_;
  rclcpp::Client<interfaces::srv::Inventory>::SharedPtr inventoryClient_;
  rclcpp::Client<interfaces::srv::ArmMovement>::SharedPtr armClient_;
  interfaces::msg::BrainStatus brainStatusMsg_;
  std_msgs::msg::Int32MultiArray perceptionMsg_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  geometry_msgs::msg::TransformStamped targetFrameTransform_;
  int targetItemID_;
  int targetSlot_;
  float targetItemCoords_[3];
  float armCoords_[3];
  bool isBusy_;
  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<Brain>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}