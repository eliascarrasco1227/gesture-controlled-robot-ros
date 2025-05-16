#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declare the mediapipe pose detector to be used
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)
mp_drawing = mp.solutions.drawing_utils

bridge = CvBridge()

# Control message publisher
ackermann_command_publisher = None

#Operator image processing
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Processing the image with MediaPipe
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_image)

    # TODO Recognise the gesture by means of some classification from the landmarks.
    gesture = "none"
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            index_tip = hand_landmarks.landmark[8]
            thumb_tip = hand_landmarks.landmark[4]

            if index_tip.y < thumb_tip.y: # señalar arriba
                gesture = "recto"
            elif index_tip.x < thumb_tip.x: # señalar derecha
                gesture = "izquierda"
            elif index_tip.x > thumb_tip.x:
                gesture = "derecha" # señalar izquierda

    # TODO Draw landsmarks on the image
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS
            )
    
    # Display image with detected landmarks/gestures
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

    # TODO Interpret the obtained gesture and send the ackermann control command.
    drive_msg = ackermann_msgs.msg.AckermannDrive()
    drive_msg.speed = 0.5

    if gesture == "izquierda":
        drive_msg.steering_angle = 0.75
    elif gesture == "derecha":
        drive_msg.steering_angle = -0.75
    elif gesture == "recto":
        drive_msg.steering_angle = 0.0
    else:
        drive_msg.speed = 0.0

    ackermann_command_publisher.publish(drive_msg)

def main():
    global ackermann_command_publisher
    
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()