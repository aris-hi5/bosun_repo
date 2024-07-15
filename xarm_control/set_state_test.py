import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16, GetInt16

class TestSetStateNode(Node):
    def __init__(self):
        super().__init__('test_set_state_node')
        self.set_state_service = self.create_client(SetInt16, '/ufactory/set_state')
        self.get_state_service = self.create_client(GetInt16, '/ufactory/get_state')

        self.wait_for_service(self.set_state_service, 'set state service')
        self.wait_for_service(self.get_state_service, 'get state service')

        self.test_set_state()

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        self.get_logger().info(f'{service_name} is ready')

    def test_set_state(self):
        state_request = SetInt16.Request()
        state_request.data = 1
        response = self.async_service_call(self.set_state_service, state_request, "set_state_service")

        if response is not None:
            self.get_logger().info(f"Response: {response.message}")
            self.get_logger().info("Set state service call succeeded")
            self.check_state()
        else:
            self.get_logger().warning("Set state service call failed")

    def check_state(self):
        state_request = GetInt16.Request()
        response = self.async_service_call(self.get_state_service, state_request, "get_state_service")

        if response is not None:
            state = response.data
            self.get_logger().info(f"Current state: {state}")
        else:
            self.get_logger().warning("Failed to get state")

    def async_service_call(self, client, request, service_name):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"{service_name} call failed: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TestSetStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
