import rclpy
from rclpy.node import Node
from rclpy.task import Future
from interfaces.msg import InventoryStatus
from interfaces.srv import Inventory



class InventoryVerification(Node):
    def __init__(self):
        super().__init__('inventory_verification')

        self.inventory_client = self.create_client(Inventory, 'inventory')
        self.inventory_status_sub = self.create_subscription(InventoryStatus, 'inventory_status',self.inventory_status_callback,10)

        while not self.inventory_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Inventory service not available, waiting...')

        try:
            inventory_request = Inventory.Request()
            inventory_request.command = "this is a string"
            inventory_request.item_id = 0
            self.future = self.inventory_client.call_async(inventory_request)
            self.future.add_done_callback(self.inventory_callback)
        except:
            self.get_logger().info(f'ERROR: Could not send inventory service call')

        self.displayed_sub = False



    def inventory_status_callback(self, msg):
        if self.displayed_sub == True:
            return
        
        self.displayed_sub = True
        try:
            if type(msg.slot_1) == int and type(msg.slot_2) == int and type(msg.slot_3) == int:
                self.get_logger().info(f'Inventory status interface is correct')
            else:
                self.get_logger().info(f'ERROR: Inventory status wrong response type')

        except:
            self.get_logger().info(f'ERROR: Inventory status cannot read data')

    def inventory_callback(self,future: Future):
        response = future.result()
        if type(response.response) == int:
            self.get_logger().info(f'Inventory service interface is correct')
        else:
            self.get_logger().info(f'ERROR: Inventory service wrong response type')


        
def main(args=None):
    rclpy.init(args=args)
    node = InventoryVerification()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
