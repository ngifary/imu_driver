import os
import sys
import time
import smbus
import math

from imusensor.MPU9250 import MPU9250, config

import numpy as np

from typing import Type, Union, List

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.cfg = config.getConfigVals()

        calib_file = self.declare_parameter("calib_file", "").value

        address = 0x68
        bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(bus, address)
        self.imu.begin()

        if calib_file == "":
            self.imu.caliberateGyro()
            self.imu.caliberateAccelerometer()
        else:
            # load your calibration file
            self.imu.loadCalibDataFromFile(calib_file)

        self.imu_pub = self.create_publisher(Imu, "imu", 10)

        hz = self.declare_parameter("hz", 30).value
        self.timer_period = 1.0 / hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.imu.readSensor()
        self.imu.computeOrientation()

        imu_msg = Imu()

        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        # RPY to convert:
        roll = self.cfg.Degree2Radian * self.imu.roll
        pitch = self.cfg.Degree2Radian * self.imu.pitch
        yaw = self.cfg.Degree2Radian * self.imu.yaw
        q = quaternion_from_euler(roll, pitch, yaw)

        quat_msg = Quaternion()
        quat_msg.x = q[0]
        quat_msg.y = q[1]
        quat_msg.z = q[2]
        quat_msg.w = q[3]

        imu_msg.orientation = quat_msg
        imu_msg.orientation_covariance[0] = -1

        gyro_msg = Vector3()
        gyro_msg.x = self.imu.GyroVals[0]
        gyro_msg.y = self.imu.GyroVals[1]
        gyro_msg.z = self.imu.GyroVals[2]

        imu_msg.angular_velocity = gyro_msg
        imu_msg.angular_velocity_covariance[0] = -1

        acc_msg = Vector3()
        acc_msg.x = self.imu.AccelVals[0]
        acc_msg.y = self.imu.AccelVals[1]
        acc_msg.z = self.imu.AccelVals[2]

        imu_msg.linear_acceleration = acc_msg
        imu_msg.linear_acceleration_covariance[0] = -1

        self.imu_pub.publish(imu_msg)

        # print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
        # print ("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
        # print ("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
        # print(
        #     "roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw)
        # )


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info(
            "Starting " + node.get_name() + ", shut down with CTRL-C"
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
