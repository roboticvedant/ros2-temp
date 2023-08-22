#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from std_srvs.srv import SetBool
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

class MyComprehensiveROSNode(Node):

    def __init__(self):
        super().__init__("comprehensive_ros_node")

        # Publishers
        self.string_publisher = self.create_publisher(String, 'string_topic', 10)
        self.int_publisher = self.create_publisher(Int32, 'int_topic', 10)
        self.twist_publisher = self.create_publisher(Twist, 'twist_topic', 10)
        
        # Subscribers
        self.string_subscriber = self.create_subscription(String, 'string_listen_topic', self.string_callback, 10)
        self.int_subscriber = self.create_subscription(Int32, 'int_listen_topic', self.int_callback, 10)
        self.twist_subscriber = self.create_subscription(Twist, 'twist_listen_topic', self.twist_callback, 10)

        # Service Servers
        self.add_two_ints_server = self.create_service(AddTwoInts, 'add_two_ints', self.callback_add_two_ints)
        self.set_bool_server = self.create_service(SetBool, 'set_bool', self.callback_set_bool)

        # Service Clients
        self.add_two_ints_client = self.create_client(AddTwoInts, 'add_two_ints')
        self.set_bool_client = self.create_client(SetBool, 'set_bool')
        
        self.get_logger().info("Node has been initialized!")

    # Callbacks for Subscribers
    def string_callback(self, msg):
        self.get_logger().info(f"Received String: {msg.data}")

    def int_callback(self, msg):
        self.get_logger().info(f"Received Int: {msg.data}")

    def twist_callback(self, msg):
        self.get_logger().info(f"Received Twist linear x: {msg.linear.x}, angular z: {msg.angular.z}")

    # Callbacks for Service Servers
    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} is {response.sum}")
        return response
    
    def callback_set_bool(self, request, response):
        # For demonstration, we simply acknowledge the boolean value received without taking any specific action.
        response.success = True
        response.message = f"Received boolean value: {request.data}"
        return response

    # Methods to call Service Clients
    def call_add_two_ints_service(self, a, b):
        while not self.add_two_ints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'add_two_ints' is not available, waiting again...")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.add_two_ints_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"AddTwoInts Service call succeeded with result: {future.result().sum}")
        else:
            self.get_logger().info("AddTwoInts Service call failed")

    def call_set_bool_service(self, value):
        while not self.set_bool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'set_bool' is not available, waiting again...")

        request = SetBool.Request()
        request.data = value
        future = self.set_bool_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"SetBool Service call succeeded with message: {future.result().message}")
        else:
            self.get_logger().info("SetBool Service call failed")

def main(args=None):
    rclpy.init(args=args)
    node_instance = MyComprehensiveROSNode()
    rclpy.spin(node_instance)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
