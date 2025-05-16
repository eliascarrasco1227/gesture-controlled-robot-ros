#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def video_publisher():
    # Initialise a ROS node
    rospy.init_node('video_publisher', anonymous=True)

    # TODO Create a publisher in the /operator/image topic.
    pub = rospy.Publisher('/operator/image', Image, queue_size=10)

    # TODO Set up video capture from webcam (or from video)
    cap = cv2.VideoCapture(0) #Webcam
    #video_path = "/home/docker/catkin_ws/src/robotica_inteligente/scripts/gestos.mp4"
    #cap = cv2.VideoCapture(video_path, cv2.CAP_FFMPEG) #video

    if not cap.isOpened():
        #rospy.logerr(f"Failed to open video file: {video_path}")
        rospy.logerr("Failed to open video file: webcam")
        return
    # Create an instance of CvBridge to convert OpenCV images to ROS messages.
    bridge = CvBridge()

    # Define the publication rate (e.g., 10 Hz).
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # TODO Capture a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("End of video file reached")
            break    
        # TODO Convert OpenCV frame to ROS message
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

        # TODO Post the message in the topic
        pub.publish(ros_image)

        # Waiting to meet the publication rate
        rate.sleep()

    # When you're done, release the catch
    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass