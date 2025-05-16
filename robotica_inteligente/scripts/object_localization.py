#!/usr/bin/env python

# This node (implemented in Python) computes the position (xyz) of the objects detected by an RGBD camera. 
# The centroid of each individual detected object is obtained using the RGB to HSV color filtering.
# The xyz objects coordinates (with respect to the camera) are calculated considering the camera-object distance and the insitric parameters of the camera are previously known.

# A template with "TODO" statements is provided to ease the node implementation. 


import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from math import sqrt, pow

# Define global variables (publishers and flags)
filtered_obj_pub = None
filtered_blue_pub = None
pose_array_pub = None
obt_detec = False
robot_detec = False

centroide_obj = 0.0  
centroide_blue = 0.0  
depth_obj = 0.0  
depth_blue = 0.0  

def filter_img_objects(color_image, lower, upper):   
    # Function to filter an RGB image to a given color in HSV range.
    # This function ouputs the filtered image, the object centroid in image coordinates, and a flag 
    # indicating if an object was detected or not.

    # Convert the RGB image to HSV channel for the filtering
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Filter the image to get the mask
    mask = cv2.inRange(hsv_image, lower, upper)
    filtered_image = cv2.bitwise_and(color_image, color_image, mask=mask)

    # Obtain the mask contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Save the coordinates of the object centroid in pixels, and a detected/not detected flag for the object.
     
    centroid = None #(px,py)
    detection = True #detected/not detected object flag

    #TODO
    # Calculate the mask centroid
    # Obtain the contour with the biggest area (cv2.contourArea) given the contours of the filtered object.
    # Obtaing the object centroid (px,py) using the image moments (cv2.moments)
    
    if contours:

        # Find the contour with the biggest area to obtain its contour
        max_contour = max(contours, key=cv2.contourArea)

        # Obtain the image moments
        moment = cv2.moments(max_contour)

        # Calculate the centroid coordinates
        centroid = (int(moment['m10'] / (moment['m00'] + 1e-5)), int(moment['m01'] / (moment['m00'] + 1e-5)))
    else:
        # bandera para decir que no existio objeto en la mascara
        detection = False

    return filtered_image, centroid, detection

def img_xyz(centroid, depth, camera_matrix):

    # This function transforms a pixel (centroid) from a depth image to a XYZ coordinate using the intrinsic parameters of the camera (camera_matrix).
    # This function outputs the x,y,z of the detected object.

   
    # TODO 
    # Extract the intrinsic parameters of the camera_matrix variable given the following order:
    
    # fx 0  cx 
    # 0  fy cy
    # 0  0  1 

    fx = camera_matrix[0]
    cx = camera_matrix[2]
    fy = camera_matrix[4]
    cy = camera_matrix[5]

    # TODO 
    # Calculate the x,y,z coordinates of the object given the intrinsic parameters of the camera (fx,fy,cx,cy) and 
    # the object centroid (px,py).

    x = (centroid[0]-cx)/fx*depth
    y = (centroid[1]-cy)/fy*depth
    z = depth

    return x,y,z


def color_image_callback(color_image_msg):

    # Callback function of the RGB image.
    # This function filters the RGB image and publish two messages with the filtered images of the 
    # detected objects (red object and Blue robot)

    # Definition of global variables
    global filtered_obj_pub, filtered_blue_pub, centroide_obj, centroide_blue, obt_detec , robot_detec

    # Convert the image message from ROS type to OpenCV
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding="bgr8")

    # Filter robot Blue
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([120, 255, 255])
    filtered_blue, centroide_blue, robot_detec = filter_img_objects(color_image,lower_blue,upper_blue)


    # Publish the filtered image of the robot
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_blue, encoding="bgr8")
    filtered_blue_pub.publish(filtered_image_msg)

    #TODO
    # Filter the red color objects corresponding to the objects to manipulate in the scene, 
    # based on the previous example of blue color filtering.

    # Filter red color object (HSV limits)
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])

    filtered_obj, centroide_obj, obt_detec  = filter_img_objects(color_image,lower_red,upper_red)

    # Publish the filtered image of the object
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_obj, encoding="bgr8")
    filtered_obj_pub.publish(filtered_image_msg)



def depth_image_callback(depth_image_msg):

    # Callback function of the depth image
    # This callback function obtains the depth of the pixel centroid (px,py) given the depth image.
    # If the centroid does not exist because an object is not detected, the depth is not calculated.

    # Definition of global variables
    global centroide_obj, centroide_blue, depth_obj, depth_blue, obt_detec, robot_detec 

    # Transform the depth image message to a OpenCV image
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg)

    # Obtain the data from the depth image given the centroid.
    # If the detected object or robot variables are False, then there is no depth.

    # TODO 
    # Obtain the camera-object distance (depth) of the object and robot in the scene, given the previous description.
    # Note: The depth image data must be converted from millimeters to meters.

    if (obt_detec):
        depth_obj  = depth_image[centroide_obj[1]][centroide_obj[0]]/1000
    
    if (robot_detec):
        depth_blue = depth_image[centroide_blue[1]][centroide_blue[0]]/1000


def camera_info_callback(camera_info_msg):

    # Callback function of the camera information.
    # This callback obtains the intrinsic parameters of the camera.
    # Once the RGB and depth image data are obtained, the object is located with 
    # respect to the camera.

    # Definition of global variables
    global centroide_obj, centroide_blue, depth_obj, depth_blue, pose_array_pub, obt_detec, robot_detec

    # Obtain the object localization
    # Initialize xyz variables of each object.
    # If an object is not detected in the image, z=-1, x=0, y=0
    x_obj, y_obj, z_obj    = 0,0,-1 
    x_blue, y_blue, z_blue = 0,0,-1 

    if (obt_detec):
        x_obj, y_obj, z_obj    = img_xyz(centroide_obj, depth_obj, camera_info_msg.K)
        distancia_euclidiana = sqrt(x_obj**2 + y_obj**2 + z_obj**2)  
        print(f"Distancia euclidiana al objeto: {distancia_euclidiana:.3f} metros")

    if (robot_detec):
        x_blue, y_blue, z_blue = img_xyz(centroide_blue, depth_blue, camera_info_msg.K) 

    # Invert y axis so that it matches the global axis.
    y_obj, y_blue = -y_obj, -y_blue

    # Print the object localization
    print("XYZ object: {:.3f}, {:.3f}, {:.3f}".format(x_obj, y_obj, z_obj))
    print("XYZ robot : {:.3f}, {:.3f}, {:.3f}".format(x_blue, y_blue, z_blue))


    # Define a PoseArray message
    pose_array_msg = PoseArray()

        # Define the message header (timestamp and frame)
    pose_array_msg.header.stamp = rospy.Time.now()
    pose_array_msg.header.frame_id = "camera_link"  

    # Object pose
    pose_obj = Pose()
    pose_obj.position.x = x_obj
    pose_obj.position.y = y_obj
    pose_obj.position.z = z_obj
    pose_obj.orientation.w = 1.0  

    # Robot pose
    pose_blue = Pose()
    pose_blue.position.x = x_blue
    pose_blue.position.y = y_blue
    pose_blue.position.z = z_blue
    pose_blue.orientation.w = 1.0  

    # Add the object pose to the PoseArray message
    pose_array_msg.poses.append(pose_obj)
    # Add the robot pose to the PoseArray message
    pose_array_msg.poses.append(pose_blue)

    # Publish the PoseArray message
    pose_array_pub.publish(pose_array_msg)

   
def ros_node():
    rospy.init_node('Object_localization', anonymous=True)

    # Subscribes to the following topics:
    # - RGB image from the camera
    # - Depth image from the camera
    # - Intrinsic parameters of the camera (fx,fy,cx,cy)
    
    # TODO
    # Add the topics to which the program needs to be subscribed. Follow the example of how to 
    # subscribe to the "/camera/color/image_raw" topic to obtaing the RGB image.
    # The subscribers to obtain the depth image and the intrinsic parameters of the camera must be implemented.
    
    color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback)
    depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback) 
    camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)
    

    # global variables
    global filtered_obj_pub, filtered_blue_pub,pose_array_pub

    # Define publishers (filtered images and objects poses)
    filtered_obj_pub  = rospy.Publisher('/filtered_image/object', Image, queue_size=1)
    filtered_blue_pub = rospy.Publisher('/filtered_image/robot', Image, queue_size=1)
    pose_array_pub = rospy.Publisher('/pose_array', PoseArray, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        ros_node()
    except rospy.ROSInterruptException:
        pass