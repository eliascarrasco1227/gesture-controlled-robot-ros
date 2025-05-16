#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('obstacle_avoidance')

        # Subscriber for the point cloud of obstacles captured by lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Subscriber for Ackermann control commands
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publisher for modified Ackermann commands
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)

        # Store the last Ackermann message received
        self.last_ackermann_cmd = AckermannDrive()

    def obstacle_callback(self, msg):
        # TODO Process the point cloud with the obstacles taking into account the last received movement message to avoid collisions.
        stop = False
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            
            if  x < 1.5 and 0 < x and abs(y) < 0.5:
                stop = True
                break

 
        cmd = self.last_ackermann_cmd

        # TODO Modify ackermann's message if necessary
        if stop:
            cmd.speed = 0.0

        self.cmd_pub.publish(cmd)
        
        return

    def ackermann_callback(self, msg):
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self):
        cmd = self.last_ackermann_cmd
        cmd.drive.speed = 0.0  
        cmd.drive.steering_angle = 0.0 
        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
