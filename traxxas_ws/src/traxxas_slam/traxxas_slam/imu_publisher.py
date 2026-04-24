import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
)
from adafruit_extended_bus import ExtendedI2C
import adafruit_bus_device.i2c_device as i2c_device_module
import time

def _patched_probe(self):
    try:
        self.i2c.readfrom_into(self.device_address, bytearray(1))
    except OSError:
        raise ValueError("No I2C device at address: 0x%x" % self.device_address)

i2c_device_module.I2CDevice._I2CDevice__probe_for_device = _patched_probe

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        i2c = ExtendedI2C(1)
        self.sensor = BNO08X_I2C(i2c, address=0x4B)
        time.sleep(1.0)  # wait for sensor to fully boot

        # Enable one feature at a time with a small delay between each
        self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.1)
        self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.1)
        self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        time.sleep(0.1)

        # 20Hz instead of 50Hz — gives the sensor time to respond
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("IMU ONLINE")

    def loop(self):
        try:
            quat = self.sensor.quaternion
            gyro = self.sensor.gyro
            accel = self.sensor.acceleration
        except (OSError, KeyError, Exception) as e:
            self.get_logger().warn(f"Sensor read error (skipping): {e}")
            return

        if quat is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]

        # Real covariance values so Cartographer accepts the data
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        if gyro is not None:
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        if accel is not None:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
