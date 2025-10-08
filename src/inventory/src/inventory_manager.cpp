#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/inventory.hpp"
#include "interfaces/msg/inventory_status.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class InventoryManagerServer : public rclcpp::Node
{
public:

  InventoryManagerServer() : Node("inventory_manager")
  {
    // Create inventory service
    srv_ = create_service<interfaces::srv::Inventory>("inventory", std::bind(&InventoryManagerServer::update, this, std::placeholders::_1, std::placeholders::_2));
    // Create inventory status publisher
    statusPublisher_ = this->create_publisher<interfaces::msg::InventoryStatus>("inventory_status", 10);
    // Create markers publisher
    markerPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("inventory_marker", 10);
    // Create timer
    timer_ = this->create_wall_timer(1000ms, std::bind(&InventoryManagerServer::timer_callback, this));
    // Initialise inventory status
    statusMessage_.slot_1 = -1;
    statusMessage_.slot_2 = -1;
    statusMessage_.slot_3 = -1;
    // Initialise markers
    markerMessage_.type = visualization_msgs::msg::Marker::SPHERE;
    //markerMessage_.ns = "markers";
    markerMessage_.action = visualization_msgs::msg::Marker::ADD;
    markerMessage_.scale.x = 0.1;
    markerMessage_.scale.y = 0.1;
    markerMessage_.scale.z = 0.1;
  }

private:
  // Update inventory status when service is called
  void update(const std::shared_ptr<interfaces::srv::Inventory::Request> request, std::shared_ptr<interfaces::srv::Inventory::Response> response)
  {
    // Invalid condition
    if (request->item_id < 0 || (request->command != "put_in_inventory" && request->command != "get_from_inventory")) {
        response->response = -2;
    }
    // Add item ID into inventory
    if (request->item_id >= 0 && request->command == "put_in_inventory") {
        if (statusMessage_.slot_1 == -1) {
            response->response = 1;
            statusMessage_.slot_1 = request->item_id;
        }
        else if (statusMessage_.slot_2 == -1) {
            response->response = 2;
            statusMessage_.slot_2 = request->item_id;
        }
        else if (statusMessage_.slot_3 == -1) {
            response->response = 3;
            statusMessage_.slot_3 = request->item_id;
        }
        else {
            response->response = -1;
        }
    }
    // Remove item ID from inventory
    if (request->item_id >= 0 && request->command == "get_from_inventory") {
        if (statusMessage_.slot_1 == request->item_id) {
            response->response = 1;
            statusMessage_.slot_1 = -1;
        }
        else if (statusMessage_.slot_2 == request->item_id) {
            response->response = 2;
            statusMessage_.slot_2 = -1;
        }
        else if (statusMessage_.slot_3 == request->item_id) {
            response->response = 3;
            statusMessage_.slot_3 = -1;
        }
        else {
            response->response = -1;
        }
    }
  }

  void timer_callback()
  {
    // Publish inventory status
    statusPublisher_->publish(statusMessage_);

    // Publish markers
    markerMessage_.header.frame_id = "inventory_slot_1";
    markerMessage_.header.stamp = this->now();
    markerMessage_.id = 0;
    if (statusMessage_.slot_1 == -1) {
        setColourGreen();
    } 
    else {
        setColourRed();
    }
    markerPublisher_->publish(markerMessage_);

    markerMessage_.header.frame_id = "inventory_slot_2";
    markerMessage_.header.stamp = this->now();
    markerMessage_.id = 1;
    if (statusMessage_.slot_2 == -1) {
        setColourGreen();
    } 
    else {
        setColourRed();
    }
    markerPublisher_->publish(markerMessage_);

    markerMessage_.header.frame_id = "inventory_slot_3";
    markerMessage_.header.stamp = this->now();
    markerMessage_.id = 2;
    if (statusMessage_.slot_3 == -1) {
        setColourGreen();
    } 
    else {
        setColourRed();
    }
    markerPublisher_->publish(markerMessage_);

  }

  void setColourGreen() {
    markerMessage_.color.r = 0.0f;
    markerMessage_.color.g = 1.0f;
    markerMessage_.color.b = 0.0f;
    markerMessage_.color.a = 1.0;
  }

  void setColourRed() {
    markerMessage_.color.r = 1.0f;
    markerMessage_.color.g = 0.0f;
    markerMessage_.color.b = 0.0f;
    markerMessage_.color.a = 1.0;
  }

  rclcpp::Service<interfaces::srv::Inventory>::SharedPtr srv_;
  rclcpp::Publisher<interfaces::msg::InventoryStatus>::SharedPtr statusPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  interfaces::msg::InventoryStatus statusMessage_;
  visualization_msgs::msg::Marker markerMessage_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<InventoryManagerServer>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
