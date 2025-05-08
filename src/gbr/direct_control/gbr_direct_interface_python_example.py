#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from packages.gbr_direct_interface import GBRDirectInterface

def main():
    rclpy.init()
    node = Node('direct_interface_python')    
    rov = GBRDirectInterface(node)

    # Forward movement
    print("\n--- Moving forward ---")
    rov.set_thrusters([40.0, 40.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(20):
        rov.print_state()
        time.sleep(0.1)
        rclpy.spin_once(node)
    
    # Rotate right
    print("\n--- Rotating right ---")
    rov.set_thrusters([-40.0, 40.0, -40.0, 40.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(20):
        rov.print_state()
        time.sleep(0.1)
        rclpy.spin_once(node)
    
    # Move backward
    print("\n--- Moving backward ---")
    rov.set_thrusters([-40.0, -40.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(20):
        rov.print_state()
        time.sleep(0.1)
        rclpy.spin_once(node)
    
    # Rotate left
    print("\n--- Rotating left ---")
    rov.set_thrusters([40.0, -40.0, 40.0, -40.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(20):
        rov.print_state()
        time.sleep(0.1)
        rclpy.spin_once(node)

    rov.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()