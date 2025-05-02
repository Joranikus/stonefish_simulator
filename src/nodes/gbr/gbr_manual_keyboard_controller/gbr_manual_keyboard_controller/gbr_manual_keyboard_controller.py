#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pynput import keyboard
import threading
import time

class GBRManualKeyboardController(Node):
    def __init__(self):
        super().__init__('gbr_manual_keyboard_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/gbr/thrusters', 10)
        
        self.thrust_values = [0.0] * 8
        self.active_keys = set()
        self.lock = threading.Lock()
        self.emergency_stop = False

        self.thrust_map = {
            'w': [20.0, 20.0, -20.0, -20.0, 0.0, 0.0, 0.0, 0.0], 
            's': [-20.0, -20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0],
            'a': [20.0, -20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0], 
            'd': [-20.0, 20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0], 
            'q': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, -20.0, -20.0], 
            'e': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 20.0, 20.0],    
            '<up': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, 20.0, 20.0],
            '<down': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, -20.0, -20.0], 
            '<right': [0.0, 0.0, 0.0, 0.0, -20.0, 20.0, -20.0, 20.0], 
            '<left': [0.0, 0.0, 0.0, 0.0, 20.0, -20.0, 20.0, -20.0], 
            '<page_up': [20.0, -20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],  
            '<page_down': [-20.0, 20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0],  
            '<space': self.emergency_stop_handler
        }

        self.timer = self.create_timer(0.05, self.publish_thrusters)
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.keyboard_listener.start()

    def on_press(self, key):
        try:
            with self.lock:
                key_str = key.char if hasattr(key, 'char') else str(key).replace('Key.', '<')
                self.active_keys.add(key_str)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            with self.lock:
                key_str = key.char if hasattr(key, 'char') else str(key).replace('Key.', '<')
                self.active_keys.discard(key_str)
        except AttributeError:
            pass

    def emergency_stop_handler(self):
        self.emergency_stop = True
        self.thrust_values = [0.0] * 8
        self.get_logger().error("EMERGENCY STOP ACTIVATED!")

    def publish_thrusters(self):
        with self.lock:
            if self.emergency_stop:
                self.thrust_values = [0.0] * 8
            else:
                self.thrust_values = [0.0] * 8
                for key in self.active_keys:
                    if key in self.thrust_map:
                        if callable(self.thrust_map[key]):
                            self.thrust_map[key]()
                        else:
                            self.thrust_values = [sum(pair) for pair in zip(
                                self.thrust_values, 
                                self.thrust_map[key]
                            )]

            msg = Float64MultiArray()
            msg.data = self.thrust_values
            self.publisher_.publish(msg)
            self.display_status()

    def display_status(self):
        status = f"""
        Thrusters: {self.thrust_values}
        Active Keys: {self.active_keys}
        Emergency Stop: {'ACTIVE' if self.emergency_stop else 'inactive'}
        """
        print("\033[2J\033[H")
        print(status)

    def __del__(self):
        self.keyboard_listener.stop()

def main(args=None):
    rclpy.init(args=args)
    node = GBRManualKeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()