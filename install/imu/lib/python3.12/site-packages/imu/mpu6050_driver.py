#!/usr/bin/env python3

# Required imports for ROS 2, I2C communication, and sensor messages
import rclpy.time
import smbus
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Empty

# MPU6050 register addresses and configuration values
# These are hardware-specific addresses used to communicate with the MPU6050 sensor
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68


class MPU6050Driver(Node):
    """
    Main driver class for the MPU6050 IMU sensor.
    Handles I2C communication and publishes IMU data as ROS 2 messages.
    """

    def __init__(self):
        # Initialize ROS 2 node and set up publishers and timers
        super().__init__("mpu6050_driver")
        
        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.angle_pub_ = self.create_publisher(Float32, "/imu/angle", qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.frequency_ = 0.01  # 100Hz update rate
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        
        # Reset angle service
        self.reset_angle_service_ = self.create_service(Empty, "/imu/reset_angle", self.reset_angle_callback)
        
        # Angle integration variables
        self.current_angle_z_ = 0.0  # Current Z angle in degrees
        self.last_time_ = None  # For calculating dt
        
        # Calibration and filtering parameters
        self.calibration_samples_ = 100
        self.is_calibrated_ = False
        self.accel_bias_ = np.array([0.0, 0.0, 0.0])
        self.gyro_bias_ = np.array([0.0, 0.0, 0.0])
        
        # Simple moving average filter
        self.filter_size_ = 5
        self.accel_buffer_ = np.zeros((self.filter_size_, 3))
        self.gyro_buffer_ = np.zeros((self.filter_size_, 3))
        self.buffer_index_ = 0
        
        # Noise thresholds (values below this are considered noise)
        self.gyro_noise_threshold_ = 0.01  # rad/s
        self.accel_noise_threshold_ = 0.1   # m/s^2
        
        # Start calibration
        self.get_logger().info("Starting IMU calibration. Keep sensor still for 5 seconds...")
        self.calibrate_sensor()

    def timerCallback(self):
        # Main callback function that reads sensor data and publishes IMU messages
        try:
            if not self.is_connected_:
                self.init_i2c()
            
            if not self.is_calibrated_:
                return
            
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Convert raw values to proper units and apply bias correction
            accel_raw = np.array([acc_x / 1670.13, acc_y / 1670.13, acc_z / 1670.13])
            gyro_raw = np.array([gyro_x / 939.65, gyro_y / 939.65, gyro_z / 939.65])
            
            # Apply bias correction
            accel_corrected = accel_raw - self.accel_bias_
            gyro_corrected = gyro_raw - self.gyro_bias_
            
            # Apply noise filtering
            accel_filtered = self.apply_filter(accel_corrected, self.accel_buffer_)
            gyro_filtered = self.apply_filter(gyro_corrected, self.gyro_buffer_)
            
            # Apply noise thresholding (zero out small values)
            gyro_filtered = self.apply_noise_threshold(gyro_filtered, self.gyro_noise_threshold_)
            accel_filtered = self.apply_noise_threshold_accel(accel_filtered, self.accel_noise_threshold_)
            
            # Populate IMU message
            self.imu_msg_.linear_acceleration.x = float(accel_filtered[0])
            self.imu_msg_.linear_acceleration.y = float(accel_filtered[1])
            self.imu_msg_.linear_acceleration.z = float(accel_filtered[2])
            self.imu_msg_.angular_velocity.x = float(gyro_filtered[0])
            self.imu_msg_.angular_velocity.y = float(gyro_filtered[1])
            self.imu_msg_.angular_velocity.z = float(gyro_filtered[2])

            # Integrate Z angular velocity to get angle
            current_time = self.get_clock().now()
            if self.last_time_ is not None:
                dt = (current_time - self.last_time_).nanoseconds / 1e9  # Convert to seconds
                # Integrate angular velocity (rad/s) to get angle change
                angle_change_rad = gyro_filtered[2] * dt
                angle_change_deg = np.degrees(angle_change_rad)  # Convert to degrees
                
                # Update current angle
                self.current_angle_z_ += angle_change_deg
                
                # Wrap angle to 0-360 degrees
                self.current_angle_z_ = self.current_angle_z_ % 360.0
                
                # Publish angle
                angle_msg = Float32()
                angle_msg.data = self.current_angle_z_
                self.angle_pub_.publish(angle_msg)
                
                # Print current angle to console
                print(f"{self.current_angle_z_:.2f}")
                
            self.last_time_ = current_time

            self.imu_msg_.header.stamp = current_time.to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        # Initialize I2C communication and configure MPU6050 sensor
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False
        
    def read_raw_data(self, addr):
        # Read 16-bit data from MPU6050 registers and convert to signed values
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
        
        # Concatenate higher and lower value
        value = ((high << 8) | low)
            
        # Convert to signed value
        if(value > 32768):
            value = value - 65536
        return value
    
    def calibrate_sensor(self):
        """Calibrate the sensor by collecting bias values when stationary"""
        if not self.is_connected_:
            self.get_logger().error("Cannot calibrate: sensor not connected")
            return
            
        accel_sum = np.array([0.0, 0.0, 0.0])
        gyro_sum = np.array([0.0, 0.0, 0.0])
        
        self.get_logger().info("Collecting calibration data...")
        
        for i in range(self.calibration_samples_):
            try:
                # Read raw sensor data
                acc_x = self.read_raw_data(ACCEL_XOUT_H) / 1670.13
                acc_y = self.read_raw_data(ACCEL_YOUT_H) / 1670.13
                acc_z = self.read_raw_data(ACCEL_ZOUT_H) / 1670.13
                
                gyro_x = self.read_raw_data(GYRO_XOUT_H) / 939.65
                gyro_y = self.read_raw_data(GYRO_YOUT_H) / 939.65
                gyro_z = self.read_raw_data(GYRO_ZOUT_H) / 939.65
                
                accel_sum += np.array([acc_x, acc_y, acc_z])
                gyro_sum += np.array([gyro_x, gyro_y, gyro_z])
                
                time.sleep(0.05)  # 50ms between samples
                
            except OSError:
                self.get_logger().error("I2C error during calibration")
                return
        
        # Calculate bias (average of all samples)
        self.gyro_bias_ = gyro_sum / self.calibration_samples_
        
        # For accelerometer, we expect gravity on Z-axis, so only bias X and Y
        accel_avg = accel_sum / self.calibration_samples_
        self.accel_bias_[0] = accel_avg[0]  # X should be 0, so bias is the average
        self.accel_bias_[1] = accel_avg[1]  # Y should be 0, so bias is the average  
        self.accel_bias_[2] = accel_avg[2] - (-9.81)  # Z should be -9.81 (gravity), so bias is avg - (-9.81)
        
        self.is_calibrated_ = True
        self.get_logger().info(f"Calibration complete!")
        self.get_logger().info(f"Gyro bias: [{self.gyro_bias_[0]:.4f}, {self.gyro_bias_[1]:.4f}, {self.gyro_bias_[2]:.4f}]")
        self.get_logger().info(f"Accel bias: [{self.accel_bias_[0]:.4f}, {self.accel_bias_[1]:.4f}, {self.accel_bias_[2]:.4f}]")
    
    def apply_filter(self, data, buffer):
        """Apply simple moving average filter"""
        # Add new data to buffer
        buffer[self.buffer_index_] = data
        self.buffer_index_ = (self.buffer_index_ + 1) % self.filter_size_
        
        # Return average of buffer
        return np.mean(buffer, axis=0)
    
    def apply_noise_threshold(self, data, threshold):
        """Zero out values below noise threshold"""
        result = data.copy()
        result[np.abs(result) < threshold] = 0.0
        return result
    
    def apply_noise_threshold_accel(self, data, threshold):
        """Zero out X and Y accelerometer values below threshold, preserve Z"""
        result = data.copy()
        # Only apply threshold to X and Y axes, preserve Z (gravity)
        if np.abs(result[0]) < threshold:
            result[0] = 0.0
        if np.abs(result[1]) < threshold:
            result[1] = 0.0
        # Don't threshold Z-axis as it should maintain gravity reading
        return result
    
    def reset_angle_callback(self, request, response):
        """Service callback to reset the Z angle to 0 degrees"""
        self.current_angle_z_ = 0.0
        self.get_logger().info("Z angle reset to 0 degrees")
        return response


def main():
    # Main function to initialize and run the MPU6050 driver node
    rclpy.init()
    mpu6050_driver = MPU6050Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 
    
    # ros2 topic echo /imu/out
    # ros2 run sensor_fusion mpu6050_driver
    
#     source install/setup.bash
# ros2 run sensor_fusion mpu6050_driver

# source install/setup.bash
# ros2 run sensor_fusion mpu6050_driver



# pip install smbus

# colcon build --parallel-workers 2