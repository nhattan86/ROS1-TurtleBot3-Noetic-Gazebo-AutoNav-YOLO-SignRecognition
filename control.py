#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ultralytics import YOLO

# Load a model
model = YOLO("/home/thesun/dacn/src/yolov8/ultralytics/best.pt") 

# Global variables to store current speed values
current_speed = 0.0
speed_1 = 0.05
speed_2 = 0.15
speed_3 = 0.25


def callback(image_msg):
    """This function is called to handle the subscribed messages

    Args:
        image_msg (Image): message type Image from sensor_msgs
    """
    global current_speed
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        
        # Convert ROS image color format (RGB) to OpenCV color format (BGR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Resize the image to a smaller size
        scale_percent = 50  # adjust as needed
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        small_image = cv2.resize(cv_image, (width, height))

        # Perform YOLO detection with confidence threshold of 0.8
        results = model(small_image, conf=0.8, show=True)  # predict on the resized image with confidence threshold

        # Display the small image with bounding boxes
        cv2.imshow('YOLO Detection', results)
        #cv2.waitKey(10)

        # Adjust speed based on detection results
        if results == "speed_1":
            # Modify speed based on detection results
            # Example: Increase speed to speed_2 if any object is detected
            current_speed = speed_1
        if results == "speed_2":

            # Set default speed if no object is detected
            current_speed = speed_2
        if results == "speed_3":

            # Set default speed if no object is detected
            current_speed = speed_3    

    except CvBridgeError as error:
        print(error)

def detection_result(results):
    """Function to determine if any object is detected"""
    # Your logic to determine if any object is detected
    return True  # Placeholder logic, replace with actual detection result

def control_turtlebot():
    """Function to publish twist messages to control TurtleBot3"""
    global current_speed
    rospy.init_node("turtlebot_controller", anonymous=True)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = current_speed
        twist_msg.angular.z = 0.0  # No angular velocity

        cmd_vel_pub.publish(twist_msg)
        rate.sleep()

if __name__=="__main__":
    bridge = CvBridge()

    # Start the turtlebot controller
    try:
        control_turtlebot()
    except KeyboardInterrupt:
        print("Shutting down!")
