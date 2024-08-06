import json
import requests
import rclpy
from rclpy.node import Node
from hi5_message.srv import OrderCall

# url = "http://192.168.1.8:8000/kiosk"
kiosk = "http://172.30.1.5:8000/kiosk"
robot = "http://172.30.1.5:8000/robot"


class DBManagerNode(Node):
    def __init__(self):
        super().__init__('db_manager_node')
        self.kiosk = kiosk
        self.robot = robot
        self.order_call_db_service = self.create_service(OrderCall, "order_call_db", self.ordercall_callback)

    def ordercall_callback(self, req, res):
        try:
            self.get_logger().info("ordercall callback activated in dbmanager")
            if req.data:
                self.get_logger().info(f"{req.data}")
                self.get_logger().info("OrderCall Service !!")
                if "OR" in req.data:
                    headers = {"Content-Type": "application/json", "dataType": "json"}
                    response = requests.post(self.kiosk, headers=headers, data=req.data)
                    self.get_logger().info(f"{response}")
                    self.get_logger().info(f"ororororor{req.data}")
                    # self.get_logger().info(f"hihihihi{response}")
                    self.get_logger().info(f"ororororo{response.json()}") # json형태
                    message = response.json()
                    self.get_logger().info(f"responseresponseresponse{message}")

                    message = json.dumps(message) #dict -> string

                    res.success = True
                    res.message = message
                    return res

                elif "OS" in req.data:
                    headers = {"Content-Type": "application/json", "dataType": "json"}
                    response = requests.post(self.robot, headers=headers, data=req.data)

                    # self.get_logger().info(f"{response}")
                    # self.get_logger().info(f"osososososo{req.data}")
                    # self.get_logger().info(f"osososososo{response.json()}") # json형태
                    # message = response.json()
                    # self.get_logger().info(f"responseresponseresponse{message}")

                    # message = json.dumps(message) #dict -> string

                    res.success = True
                    res.message = ""
                    return res
                else:
                    self.get_logger().info("invalid cmd in request")
                    return res
            else:
                self.get_logger().info("OrderCall no data")
                res.success = False
                res.message = "No data received"
            return res
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"주문 요청 처리 중 오류: {e}")
            res.success = False
            res.message = f"Order call failed: {e}"
            return res
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            res.success = False
            res.message = f"Unexpected error: {e}"
            return res

def main(args=None):
    rclpy.init(args=args)
    node = DBManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
