#!/usr/bin/env python
#
# ROS node to move the UR5 arm robot to the goal position using MoveIt and
# open/close the gripper. It comunicates with the other nodes through the topic
# /ur5_goal, the student should add the appropiate topics for coordinate the tasks.
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf
from tf.transformations import quaternion_from_euler

from math import pi, dist, cos, fabs
from moveit_commander.conversions import pose_to_list

#TODO define the global position of the robot.
ROBOT_POSITION = geometry_msgs.msg.Point(x=4.5, y=3.5, z=0) 

#TODO define joint positions of the arm (home position) and of the gripper (open and close)
#They must be in radians.
HOME_JOINT_STATE = [0, -1.50, -1.50, -1.50, 1.50, -0.75]

GR2RAD = pi/180

OPEN_JOINT_STATE = [3*GR2RAD, 0, -34*GR2RAD, 3*GR2RAD, 0, -33*GR2RAD, 3*GR2RAD, 0, -33*GR2RAD]

CLOSE_JOINT_STATE = [39*GR2RAD, 23*GR2RAD, -3*GR2RAD, 35*GR2RAD, 7*GR2RAD, -3*GR2RAD, 23*GR2RAD, 13*GR2RAD, -19*GR2RAD] 

class MoveUR5Node(object):

    def __init__(self):
        super(MoveUR5Node, self).__init__()

        ## Initialization of `moveit_commander`_ and the ROS node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_UR5_node", anonymous=True)
        
        self.state=1
        ## Initialize "RobotCommander". It gives information about the robot, such as its cinematic and the joint state.
        self.robot = moveit_commander.RobotCommander()

        ## Initizalize "PlanningSceneInterface". It offers a remote interface to get and set the scene where the robot works.
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        ## Initialize "MoveGroupCommander". It is an interface for planning the position of a joint group.
        ## The robot UR5 has three groups:
        ## "arm": The joints of the robotic arm.
        ## "gripper": The joints of the gripper to open and close it.
        ## "gripper_mode": The gripper has differents modes, that allosw to move the fingers in other directions.
        ## This last group doesn't need to  be used in this assignment.

        ## The interface is used to plan and execute the movements:
        self.move_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Variables
        self.box_name = ""
        self.object_position=geometry_msgs.msg.Point()

        #Remove all the objects in the scene, if there are.
        self.scene.remove_attached_object(self.move_arm.get_end_effector_link())
        self.scene.remove_world_object()
        #Go to home position
        self.go_to_joint_gripper_state(OPEN_JOINT_STATE)
        self.go_to_joint_arm_state(HOME_JOINT_STATE)

        #TODO consider additional publishers and subscribers for the communication between both robots.

        ## Publishers definition

        # Subscribers definition
        self.position_subscriber = rospy.Subscriber("/pose_array",
            geometry_msgs.msg.PoseArray, self.detection_callback, queue_size=1)
        
    def detection_callback(self, pose_array:geometry_msgs.msg.PoseArray):
        # Check if the object has been detected. It is communicated through the position in z.
        object_position=pose_array.poses[0].position
        if object_position.z<0: return

        self.object_position=object_position
    

    def go_to_joint_arm_state(self, joint_goal):
        ## Movement to a joint position of the arm.
        ## The order of the joints is the following: shoulder_pan_joint, shoulder_lift_join,
        ## elbow_joint, wrist1_joint, wrist2_joint, wrist3:joint
        print("Target joint position of the robotic arm:")
        print(joint_goal)

        if len(joint_goal)!=6:
            print("Joint position not valid, UR5 has 6 joints.")
            return False

        # The command go() can be used with joint values, poses or without parameters,
        # if the target has been defined for the group.
        self.move_arm.go(joint_goal, wait=True)

        # The stop() function ensures that there isn't residual movement.
        self.move_arm.stop()

        # Check if the robot has reached the target.
        current_joints = self.move_arm.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.015)
    
    def go_to_joint_gripper_state(self, joint_goal):
        ## Movement to a joint position of the gripper.
        # The gripper has 9 joints, 3 for each finger. 
        print("Target joint position of the gripper:")
        print(joint_goal)

        if len(joint_goal)!=9:
            print("Joint position not valid, the gripper has 9 joints.")
            return False

        # The command go() can be used with joint values, poses or without parameters,
        # if the target has been defined for the group.
        self.move_gripper.go(joint_goal, wait=True)

        # The stop() function ensures that there isn't residual movement.
        self.move_gripper.stop()

        # It isn't checked if the gripper reachs the target, because when it is closed
        # it will fail as it doesn't close completly when it graps the object, though
        # it achieves the task objective.

    def go_to_pose_arm_goal(self, pose_goal:geometry_msgs.msg.Pose):
        ## Movement to a cartesian pose, defined by the target position and orientation of the UR5 wrist. 
        print("Target cartesian pose:")
        print(pose_goal)
        # We set the current position and using the command go() we plan and
        # execute the movement. It returns if it has been able to perform it.
        self.move_arm.set_pose_target(pose_goal)
        success = self.move_arm.go(wait=True)

        # The stop() function ensures that there isn't residual movement.
        self.move_arm.stop()

        # It is good to always clean the target pose.
        self.move_arm.clear_pose_targets()

        # Check if the robot has reached the target.
        current_pose = self.move_arm.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.1)

    def add_object(self, object_position:geometry_msgs.msg.Point):
        ## Add the object to the scene. The object position (x,y)
        ## should be respect the robot's base.
        print("Adding the object at the position x={}, y={}",object_position.x, object_position.y)
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = "world"
        object_pose.pose.position = object_position
        object_pose.pose.orientation.w = 1.0
        object_pose.pose.position.z = 0.1  
        self.box_name = "object"
        self.scene.add_box(self.box_name, object_pose, size=(0.1, 0.1, 0.2))


    def attach_object(self):
        ## Attach the object to the gripper. Manipulate objects requires that the robot be able to touch them
        ## while the planner doesn't consider the contact as a collision. 
        touch_links = self.robot.get_link_names(group="gripper")
        self.scene.attach_box(self.move_arm.get_end_effector_link(), self.box_name, touch_links=touch_links)


    def detach_object(self):
        ## Detach the object.
        self.scene.remove_attached_object(self.move_arm.get_end_effector_link(), name=self.box_name)

    def remove_object(self):
        ## Remove the object, it should be detached previously.
        self.scene.remove_world_object(self.box_name)


    #TODO define functions to perform the task of grap the object and leave it in the mobile robot. Consider the use of a state machine to coordinate the process.
    #Note: The current pose of the arm (position and orientation) can be obtained through "self.move_arm.get_current_pose()"


    def execute_task_sequence(self):
        """
        Execute a sequence of tasks to grab and place an object.
        """
        punto= geometry_msgs.msg.Pose()

        punto.position.x=self.object_position.x-ROBOT_POSITION.x + 0.08
        punto.position.y=self.object_position.y-ROBOT_POSITION.y
        punto.position.z=0.5

        punto.orientation=self.move_arm.get_current_pose().pose.orientation

        if self.state==1:
            if(self.go_to_pose_arm_goal(punto)):
                self.state=2
        elif self.state==2:
            punto.position.z=0.35
            if(self.go_to_pose_arm_goal(punto)):
                self.go_to_joint_gripper_state(CLOSE_JOINT_STATE)
                self.attach_object()  # Grab the object
                self.state=3
        elif self.state==3:
            #print("raise the arm")
            self.go_to_joint_arm_state(HOME_JOINT_STATE)
            print('arm raised')
            self.state=4

    def run(self):
        """
        Main control loop of the node.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.execute_task_sequence()  # Execute the sequence of tasks
            rate.sleep()  # Maintain the frequency

    
def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
    
    elif type(goal) is geometry_msgs.msg.Point:
        x0, y0, z0 = actual.x, actual.y, actual.z
        x1, y1, z1 = goal.x, goal.y, goal.z
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        return d <= tolerance

    return True


def main():
    try:
        print("Init move_UR5_node")
        node = MoveUR5Node()
        node.run()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()