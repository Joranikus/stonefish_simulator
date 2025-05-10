#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from pynput import keyboard
import threading
from packages.gbr_direct_interface import GBRDirectInterface

# Control state
active_keys = set()
lock = threading.Lock()
emergency_stop = False

# Thrust mappings
thrust_map = {
    'w': [20.0, 20.0, -20.0, -20.0, 0.0, 0.0, 0.0, 0.0],    # Forward
    's': [-20.0, -20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0],    # Backward
    'a': [20.0, -20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0],    # Left turn
    'd': [-20.0, 20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],    # Right turn
    'q': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, -20.0, -20.0],  # Down
    'e': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 20.0, 20.0],      # Up
    'Key.up': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, 20.0, 20.0],      # Pitch up
    'Key.down': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, -20.0, -20.0],    # Pitch down
    'Key.left': [0.0, 0.0, 0.0, 0.0, 20.0, -20.0, 20.0, -20.0],    # Roll left
    'Key.right': [0.0, 0.0, 0.0, 0.0, -20.0, 20.0, -20.0, 20.0],   # Roll right
    'Key.page_up': [20.0, -20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],     # Strafe right
    'Key.page_down': [-20.0, 20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0],   # Strafe left
}

def on_press(key):
    global active_keys
    try:
        with lock:
            key_str = key.char if hasattr(key, 'char') else str(key)
            active_keys.add(key_str)
            if key_str == 'Key.space':
                global emergency_stop
                emergency_stop = True
    except AttributeError:
        pass

def on_release(key):
    global active_keys
    try:
        with lock:
            key_str = key.char if hasattr(key, 'char') else str(key)
            active_keys.discard(key_str)
    except AttributeError:
        pass

def display_status():
    print("\033[2J\033[H")  # Clear screen and move cursor to top
    print(f"Active Keys: {active_keys}")
    print(f"Emergency Stop: {'ACTIVE' if emergency_stop else 'inactive'}")

def main():
    rclpy.init()
    node = Node('gbr_keyboard_controller')    
    rov = GBRDirectInterface(node)
    
    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while rclpy.ok():
            with lock:
                if emergency_stop:
                    rov.stop()
                    node.get_logger().error("EMERGENCY STOP ACTIVATED!")
                else:
                    # Combine thrust values from all active keys
                    thrust_values = [0.0] * 8
                    for key in active_keys:
                        if key in thrust_map:
                            thrust_values = [sum(pair) for pair in zip(
                                thrust_values, 
                                thrust_map[key]
                            )]
                    rov.set_thrusters(thrust_values)

            rov.print_state()
            display_status()
            time.sleep(0.05)  # 20Hz control loop
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        listener.stop()
        rov.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()