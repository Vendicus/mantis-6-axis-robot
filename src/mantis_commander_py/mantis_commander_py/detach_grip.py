#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

class GripperAutomationManager(Node):
    def __init__(self):
        super().__init__('gripper_automation_manager')
        
        # 1. Publishers creation
        self.attach_pub_box = self.create_publisher(Empty, '/box/attach', 10)
        self.detach_pub_box = self.create_publisher(Empty, '/box/detach', 10)
        self.attach_pub_box1 = self.create_publisher(Empty, '/box1/attach', 10)
        self.detach_pub_box1 = self.create_publisher(Empty, '/box1/detach', 10)
        
        # 2. Service creation
        self.attach_srv_box = self.create_service(Trigger, '/gripper/attach', self.attach_box_callback)
        self.detach_srv_box = self.create_service(Trigger, '/gripper/detach', self.detach_box_callback)
        self.attach_srv_box1 = self.create_service(Trigger, '/gripper/attach_box1', self.attach_box1_callback)
        self.detach_srv_box1 = self.create_service(Trigger, '/gripper/detach_box1', self.detach_box1_callback)
        
        self.get_logger().info('Gripper Automation Manager succesfully started!')

    def attach_box_callback(self, request, response):
        self.get_logger().info('REQUEST RECEIVED: attach object to gripper.');
        
        # sending empty message
        msg = Empty()
        self.attach_pub_box.publish(msg)
        response.success = True
        response.message = "ATTACH signal sent to Gazebo."
        return response

    def detach_box_callback(self, request, response):
        self.get_logger().info('REQUEST RECEIVED: detach object from gripper.');
        
        # Sending empty msg to init communication
        msg = Empty()
        self.detach_pub_box.publish(msg)
        
        response.success = True
        response.message = "DETACH signal for box1 sent to Gazebo."
        return response
    
    def attach_box1_callback(self, request, response):
        self.get_logger().info('REQUEST RECEIVED: attach box2 to gripper.');
        
        # Sending empy msg to init communication
        msg = Empty()
        self.attach_pub_box1.publish(msg)
        response.success = True
        response.message = "ATTACH signal for box2 sent to Gazebo."
        return response
    
    def detach_box1_callback(self, request, response):
        self.get_logger().info('REQUEST RECEIVED: detach box2 from gripper.');
        
        # Sending empy msg to init communication 
        msg = Empty()
        self.detach_pub_box1.publish(msg)
        
        response.success = True
        response.message = "DETACH signal for box2 sent to Gazebo."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperAutomationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()