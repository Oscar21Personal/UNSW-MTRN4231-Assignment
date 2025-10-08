import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.msg import Camera
from std_msgs.msg import Int32MultiArray

class Perception(Node):
    def __init__(self):
        super().__init__('perception')

        self.reset_sub = self.create_subscription(Bool, 'reset', self.reset_callback, 10)
        self.perception_status_sub = self.create_subscription(Int32MultiArray, 'perception_status', self.perception_status_callback, 10)
        self.camera_pub = self.create_publisher(Camera, 'camera', 10)
        self.timer = self.create_timer(1.0, self.publish_camera_message)

        self.get_logger().info("Perception node launched")
        self.items = {}
        self.displayed_perception_status_sub = False
        self.publish_camera_message_once = False

    def reset_callback(self, msg):
        if isinstance(msg.data, bool):
            self.get_logger().info('Reset topic interface is correct')
        else:
            self.get_logger().info('ERROR: Reset topic wrong response type')

    def perception_status_callback(self, msg):
        if self.displayed_perception_status_sub:
            return

        self.displayed_perception_status_sub = True
        try:
            if all(isinstance(i, int) for i in msg.data):
                self.get_logger().info('PerceptionStatus topic interface is correct')
            else:
                self.get_logger().info('ERROR: PerceptionStatus topic wrong response type')
        except Exception as e:
            self.get_logger().info(f'ERROR: Cannot read PerceptionStatus topic data - {e}')

    def publish_camera_message(self):
        if self.publish_camera_message_once:
            return
        
        self.publish_camera_message_once = True
        msg = Camera()
        msg.item_id = 1
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        self.camera_pub.publish(msg)
        self.get_logger().info(f'Published Camera message: item_id={msg.item_id}, x={msg.x}, y={msg.y}, z={msg.z}')

def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
