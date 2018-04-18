#!/usr/bin/env python
from __future__ import division
import math
import serial
import struct
import rospy
from sensor_msgs.msg import Imu

# -32768 to 32767 for a 16 bit int
# The next two lines specify the default ranges for the MPU 6050.

# print(ser.name)         # check which port was really used

def imuSender():
    aRange = 2  # +/- g
    gRange = 250  # +/- degrees/second
    GACCEL = 9.80655  # m/s^2
    DEG_TO_RAD = math.pi / 180
    ser = serial.Serial('/dev/backup_imu', 57600)  # open serial port
    pub = rospy.Publisher('imu_backup', Imu, queue_size=10)
    rospy.init_node('imub')
    
    msg = Imu()
    msg.header.frame_id = 'imu_link'
    inRaw = 0
    print('begin')
    while not rospy.is_shutdown():
        ser.readline()
        # next comes the quaternions
        floats = [0.0, 0.0, 0.0, 0.0]  # wxyz
        for x in range(0, 4):
            inRaw = ser.read(4)
            floats[x] = struct.unpack('f', inRaw)[0]

        msg.orientation.w = floats[0]
        msg.orientation.x = floats[1]
        msg.orientation.y = floats[2]
        msg.orientation.z = floats[3]

        # accelerations are next
        accels = [0.0, 0.0, 0.0]
        for x in range(0, 3):
            inRaw = ser.read(2);  # read two bytes
            accels[x] = struct.unpack('h', inRaw)[0]
            if accels[x] > 0:
                accels[x] = (accels[x] / (32767 / aRange)) * GACCEL
            else:
                accels[x] = (accels[x] / (32768 / aRange)) * GACCEL

        msg.linear_acceleration.x = accels[0]
        msg.linear_acceleration.y = accels[1]
        msg.linear_acceleration.z = accels[2]

        # next comes gyro data
        accels = [0.0, 0.0, 0.0]
        for x in range(0, 3):
            inRaw = ser.read(2);
            accels[x] = struct.unpack('h', inRaw)[0]
            if accels[x] > 0:
                accels[x] = (accels[x] / (32767 / gRange)) * DEG_TO_RAD
            else:
                accels[x] = (accels[x] / (32768 / gRange)) * DEG_TO_RAD

        msg.angular_velocity.x = accels[0]
        msg.angular_velocity.y = accels[1]
        msg.angular_velocity.z = accels[2]

        pub.publish(msg)
        


if __name__ == '__main__':
    try:
        imuSender()
    except rospy.ROSInterruptException:
        pass
