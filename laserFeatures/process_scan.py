#! /usr/bin/env python
"""
Preprocessing to extract laser scans from bag file to dataframe
"""
__author__ = "Adel"


import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan


class ScanSaver(object):
    def __init__(self):
        rospy.Subscriber('scan', LaserScan, self._scan_cb)
        
    def _scan_cb(self, msg):
        rospy.logdebug("angle_min %f" %msg.angle_min)
        rospy.logdebug("angle_max %f" %msg.angle_max)
        rospy.logdebug("angle_increment %f" %msg.angle_increment)
        rospy.logdebug("range_max %f" %msg.range_max)
        rospy.logdebug("range_min %f" %msg.range_min)
        rospy.logdebug("size %f" %len(msg.ranges))

        x = []
        y = []
        for i in range(len(msg.ranges)):
            x.append(msg.ranges[i] * np.cos(msg.angle_min + (i*msg.angle_increment)))
            y.append(msg.ranges[i] * np.sin(msg.angle_min + (i*msg.angle_increment))) 
     


def main():
    plt.ion()
    rospy.init_node('scan_saver', anonymous=True, log_level=rospy.DEBUG)
    s = ScanSaver()
    rospy.spin()

if __name__ == '__main__':
    main()
