#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from dynamic_reconfigure.client import Client

# Define speed values for different traffic sign classes
speed_1 = 0.15
speed_2 = 0.20
speed_3 = 0.25

# Load the YOLOv8 model for object detection
model = YOLO("best.pt")
dynamic_param_client = None 

def callback(image_msg):
    """This function is called to handle the subscribed messages

    Args:
    image_msg (Image): message type Image from sensor_msgs
    """
    global dynamic_param_client

    try:
        # Convert ROS image format to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        # Convert ROS image color format (RGB) to OpenCV color format (BGR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Resize the image to a smaller size
        scale_percent = 50  # adjust as needed
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        small_image = cv2.resize(cv_image, (width, height))

        # Perform YOLO detection with confidence threshold of 0.8
        results = model(small_image, conf=0.8, verbose=False)
        
        cv2.imshow('YOLO Detection', results)
        cv2.waitKey(10)
        
        # Check if there are any results
        if results:
            # Get the list of detected objects
            detections = results[0].boxes.data.tolist()

            # Iterate through each detection
            for detection in detections:
                class_id = int(detection[5])
                if class_id == 0:
                    new_speed = speed_1
                if class_id == 1:
                    new_speed = speed_2
                if class_id == 2:
                    new_speed = speed_3
                               
                # Update the max_vel_x parameter of DWAPlannerROS
                if dynamic_param_client:
                    config = {'max_vel_x': new_speed}
                    dynamic_param_client.update_configuration(config)

    except CvBridgeError as error:
        print(error)

if __name__ == "__main__":
    bridge = CvBridge() 
    rospy.init_node("image_subscriber", anonymous=True)
    print("Subscribe images from topic /camera/rgb/image_raw ...")

    # Create a dynamic_reconfigure client for DWAPlannerROS
    dynamic_param_client = Client("/move_base/DWAPlannerROS")
    # Subscribe to the camera topic and call the 'callback' function for each image message
    image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, callback)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Finished!")