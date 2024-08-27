import json
import rclpy
from rclpy.node import Node
from hi5_message.srv import OrderCall

class DeliveryServiceClient(Node):

    def __init__(self):
        super().__init__('delivery_service_client')
        
        # storagy로 명령을 보내기 위한 client
        self.cli = self.create_client(OrderCall, 'delivery_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()
        
        # kiosk로부터 주문을 받기 위한 service
        self.ser = self.create_service(OrderCall, 'order_call_st', self.order_storagy_callback)

    def order_storagy_callback(self, req, res):
        try:
            if req.data:
                self.get_logger().info(f"{req.data}")
                self.get_logger().info("Order Storagy Service !!")
                response = req.data
                cmd, data = response.split(',', 1)
                self.get_logger().info(
                    f"cmd:{cmd}, status:{data}")
                
                data_dict = json.loads(data)
                table_number = data_dict["table"]
                
                self.send_request(table_number)
                
                res.success = True
                res.message = "order storagy callback"
            else:
                print("delivery manager service call error")
                res.success = False
                res.message = "No data received ORDER_STORAGY"
            return res
        
        except Exception as e:
            print(f"배달매니저 서비스 콜 응답 처리 중 오류 {e}")
            self.client_send("ER,Error processing deliverymanager order request")
            res.success = False
            res.message = f"Order storagy call failed: {e}"
            return res
        
    def send_request(self, table_number):
        req = OrderCall.Request()
        req.data = table_number
        future = self.cli.call_async(req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call successful')
            self.get_logger().info(f'Response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryServiceClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()