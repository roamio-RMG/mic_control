#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math
import serial
import time
from respeaker_ros.interface import RespeakerInterface


# new logic he lesten once and he start turning until he is in the right angle

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
        self.declare_parameter('vad_threshold', 5)  # Voice activity detection threshold in dB
        self.declare_parameter('doa_interval', 0)  # DOA processing interval in seconds
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.doa_interval = self.get_parameter('doa_interval').value
        
        # Initialize timing control for DOA processing
        self.last_doa_time = time.time() - self.doa_interval  # Allow first processing immediately
        
        # Initialize ReSpeaker interface for voice detection and noise suppression
        try:
            self.respeaker = RespeakerInterface()
            self.setup_noise_suppression()
            self.get_logger().info("ReSpeaker interface initialized with noise suppression")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ReSpeaker: {e}")
            self.respeaker = None
        
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
        
        # Subscribe to IMU angle topic
        self.imu_angle_subscription = self.create_subscription(
            Float32,
            '/imu/angle',
            self.imu_angle_callback,
            10
        )
        
        self.get_logger().info(f"DOA Listener started with voice detection, noise suppression, and {self.doa_interval}s processing interval")
        self.get_logger().info("Subscribed to /imu/angle topic")

    def setup_noise_suppression(self):
        """Configure ReSpeaker for optimal voice detection and noise suppression"""
        if self.respeaker is None:
            return
            
        try:
            # Enable stationary noise suppression
            self.respeaker.write('STATNOISEONOFF', 1)
            self.get_logger().info("Enabled stationary noise suppression")
            
            # Enable non-stationary noise suppression  
            self.respeaker.write('NONSTATNOISEONOFF', 1)
            self.get_logger().info("Enabled non-stationary noise suppression")
            
            # Enable high-pass filter to remove low frequency noise (125 Hz cutoff)
            self.respeaker.write('HPFONOFF', 2)
            self.get_logger().info("Enabled high-pass filter (125 Hz)")
            
            # Set voice activity detection threshold
            self.respeaker.set_vad_threshold(self.vad_threshold)
            self.get_logger().info(f"Set VAD threshold to {self.vad_threshold} dB")
            
            # Enable automatic gain control for consistent voice levels
            self.respeaker.write('AGCONOFF', 1)
            self.get_logger().info("Enabled automatic gain control")
            
        except Exception as e:
            self.get_logger().error(f"Failed to configure noise suppression: {e}")

    def listener_callback(self, msg):
        # Check timing - only process DOA at specified interval
        current_time = time.time()
        if current_time - self.last_doa_time < self.doa_interval:
            # Too soon since last processing, ignore this DOA reading
            return
        
        # Check if voice is detected before processing DOA
        if self.respeaker is not None:
            try:
                voice_detected = self.respeaker.is_voice()
                if not voice_detected:
                    # No voice detected, ignore this DOA reading
                    return
            except Exception as e:
                self.get_logger().warn(f"Failed to check voice activity: {e}")
                # Continue processing if voice check fails
        
        # Update last processing time (voice was detected and timing is good)
        self.last_doa_time = current_time
        
        q = msg.pose.orientation
        angle = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        print(f"Voice detected! DOA angle: {angle:.2f}° (processing after {self.doa_interval}s interval)")  # Print directly to terminal
        self.get_logger().info(f"Voice detected! DOA angle: {angle:.2f}° (processing after {self.doa_interval}s interval)")  # Also log to ROS2
        angle = angle + 180.0
        print(f"Adjusted DOA angle: {angle:.2f}°")  # Print directly to terminal
        self.get_logger().info(f"Adjusted DOA angle: {angle:.2f}°")  # Also log to ROS2
        
        # 270 degree is the front of the robot
        # 90 degree is the back of the robot
        # 180 degree is the left of the robot
        # 0 degree is the right of the robot
        
        # Control logic based on DOA angle (only when voice is detected)
        # if 270 ± 45 he will not move (225° to 315° = no move zone - front facing)
        # if more than 315 will send 'L' then 'Z' (rotate left to face voice)
        # if less than 225 will send 'R' then 'Z' (rotate right to face voice)
        
        angle_to_turn = 0
        
        if angle > 295:
            angle_to_turn = 270 - angle
            self.get_logger().info(f"Voice detected on left - rotating LEFT to face speaker: {angle_to_turn}")
        elif angle > 0 and angle <= 90:
            angle_to_turn = angle + 90
            angle_to_turn *= -1
            self.get_logger().info(f"Voice detected on left - rotating LEFT to face speaker: {angle_to_turn}")
        elif angle > 90 and angle <= 247:
            angle_to_turn = 270 - angle
            self.get_logger().info(f"Voice detected on right - rotating RIGHT to face speaker: {angle_to_turn}")
        else:
            self.get_logger().info(f"Voice detected in front - staying still: {angle_to_turn}")
            
        
        

        
        # if angle > 293 or angle <= 90:  # Right side detection (0° area)
        #     self.send_command("L")
        #     time.sleep(0.15)  # Brief delay
        #     self.send_command("Z")
        #     self.get_logger().info("Voice detected on right - rotating LEFT to face speaker")
            
        # elif angle > 90 and angle < 247:  # Left side detection (180° area)
        #     self.send_command("R") 
        #     time.sleep(0.15)  # Brief delay
        #     self.send_command("Z")
        #     self.get_logger().info("Voice detected on left - rotating RIGHT to face speaker")
    
    def imu_angle_callback(self, msg):
        """Callback function for IMU angle messages"""
        angle = msg.data
        print(f"IMU Angle: {angle:.2f}°")
        self.get_logger().info(f"Received IMU angle: {angle:.2f}°")
    
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
