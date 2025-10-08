import rclpy
from rclpy.node import Node
from rclpy.task import Future
from interfaces.srv import Command
from interfaces.msg import BrainStatus
from std_msgs.msg import Bool

class BrainVerification(Node):
    def __init__(self):
        super().__init__('brain_verification')

        self.reset_sub = self.create_subscription(Bool, 'reset', self.reset_callback, 10)
        self.brain_status_sub = self.create_subscription(BrainStatus, 'brain_status', self.brain_status_callback, 10)

        self.command_client = self.create_client(Command, 'command')
        self.displayed_reset_sub = False
        self.displayed_brain_status_sub = False

        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Command service not available, waiting...')
            
        self.send_command_request()
        self.get_logger().info('Brain verification node launched')

    def send_command_request(self):
        try:
            command_request = Command.Request()
            command_request.command = "test"  
            command_request.item_id = 1 

            future = self.command_client.call_async(command_request)
            future.add_done_callback(self.command_callback)
        except Exception as e:
            self.get_logger().info(f'ERROR: Could not send Command service request - {e}')

    def command_callback(self, future: Future):
        try:
            response = future.result()
            if isinstance(response.accept, str):
                self.get_logger().info('Command has the correct response')
            else:
                self.get_logger().info(f'ERROR: Command has the wrong response type')
        except Exception as e:
            self.get_logger().info(f'ERROR: Command failed to get response')

    def reset_callback(self, msg):
        if self.displayed_reset_sub:
            return
        self.displayed_reset_sub = True
        try:
            if isinstance(msg.data, bool):
                self.get_logger().info('Reset topic interface is correct')
            else:
                self.get_logger().info('ERROR: Reset topic wrong response type')
        except Exception as e:
            self.get_logger().info(f'ERROR: Cannot read Reset topic data - {e}')

    def brain_status_callback(self, msg):
        if self.displayed_brain_status_sub:
            return
        self.displayed_brain_status_sub = True
        try:
            if isinstance(msg.status, str):
                self.get_logger().info('BrainStatus topic interface is correct')
            else:
                self.get_logger().info('ERROR: BrainStatus topic wrong response type')
        except Exception as e:
            self.get_logger().info(f'ERROR: Cannot read BrainStatus topic data - {e}')



def main(args=None):
    rclpy.init(args=args)
    node = BrainVerification()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
