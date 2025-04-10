#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):   # Modify name
    def __init__(self): #creating a python constructor
        super().__init__("node_name")  # Modify name
       
def main(args=None):
    rclpy.init(args=args) #initialize
    node = MyCustomNode()  ##Mynode is am instance of the class Mynode(), Create node.
    rclpy.spin(node) #Spinning     
    rclpy.shutdown() #Shutdown

if __name__ == "__main__": #Corrected here
    main()