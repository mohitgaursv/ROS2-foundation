#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class Mynode(Node):
    def __init__(self): #creating a python constructor
        super().__init__("py_test")  #call the parent constructor of the node class
       
        self.get_logger().info("Hello World")
        self.create_timer(1.0,self.timer_callback)
        self.counter_=0

    def timer_callback(self):
        self.get_logger().info("Hello" + str(self.counter_))
        self.counter_ +=1

def main(args=None):
    rclpy.init(args=args) #initialize
    node = Mynode()  ##Mynode is am instance of the class Mynode(), Create node.
    rclpy.spin(node) #Spinning     
    rclpy.shutdown() #Shutdown

if __name__ == "__main__": #Corrected here
    main()