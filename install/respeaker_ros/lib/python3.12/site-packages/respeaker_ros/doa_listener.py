#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import serial
import time

def quaternion_to_yaw(x, y, z, w):
    # Convert quaternion to yaw (rotation around z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)

class DOAListener(Node):
    def __init__(self):
        super().__init__('doa_listener')
        
        # Declare parameters for serial communication
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        # Initialize serial connection
        try:
            self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/doa',
            self.listener_callback,
            10
        )
        
        self.get_logger().info("DOA Listener started with serial control")

    def listener_callback(self, msg):
        q = msg.pose.orientation
        angle = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        print(f"DOA angle: {angle:.2f}°")  # Print directly to terminal
        self.get_logger().info(f"DOA angle: {angle:.2f}°")  # Also log to ROS2
        angle = angle + 180.0
        print(f"DOA angle: {angle:.2f}°")  # Print directly to terminal
        self.get_logger().info(f"DOA angle: {angle:.2f}°")  # Also log to ROS2
        # 270 degree is the front of the robot
        # 90 degree is the back of the robot
        # 180 degree is the left of the robot
        # 0 degree is the right of the robot
        
        # Control logic based on DOA angle
        # if 270 ± 45 he will not move (225° to 315° = no move zone - front facing)
        # if more than 315 will send 'R' then 'Z' (rotate right)
        # if less than 225 will send 'L' then 'Z' (rotate left)
        
        if angle > 315 or angle < 45:  # Right side detection (0° area)
            self.send_command("R")
            time.sleep(0.1)  # Brief delay
            self.send_command("Z")
            self.get_logger().info("Rotating RIGHT to face sound source")
            
        elif angle > 135 and angle < 225:  # Left side detection (180° area)
            self.send_command("L") 
            time.sleep(0.1)  # Brief delay
            self.send_command("Z")
            self.get_logger().info("Rotating LEFT to face sound source")
            
        else:  # No move zone (225° to 315° - roughly front facing)
            self.get_logger().info("Sound source in front - staying still")
    
    def send_command(self, cmd):
        """Send command to Arduino via serial"""
        if self.arduino is not None:
            try:
                command = f"{cmd}\n"
                self.arduino.write(command.encode("utf-8"))
                self.arduino.flush()
                self.get_logger().info(f"Sent command: {cmd}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command {cmd}: {e}")
        else:
            self.get_logger().warn(f"Arduino not connected - would send: {cmd}")
    
    def destroy_node(self):
        """Clean up serial connection when node is destroyed"""
        if self.arduino is not None:
            self.send_command("Z")  # Stop motors
            self.arduino.close()
            self.get_logger().info("Serial connection closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DOAListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
