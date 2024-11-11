#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO

# Load a model
model = YOLO("best.pt") 

def callback(image_msg):
    """This function is called to handle the subscribed messages

    Args:
        image_msg (Image): message type Image from sensor_msgs
    """
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        
        # Convert ROS image color format (RGB) to OpenCV color format (BGR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Resize the image to a smaller size
        scale_percent = 50  # adjust as needed
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        small_image = cv2.resize(cv_image, (width, height))

        # Perform YOLO detection with confidence threshold of 0.79
        results = model(small_image, conf=0.79, show=True)  # predict on the resized image with confidence threshold

        # Display the small image with bounding boxes
        cv2.imshow('YOLO Detection', results)
        cv2.waitKey(10)
      
    except CvBridgeError as error:
        print(error)

if __name__=="__main__":
    bridge = CvBridge()
    rospy.init_node("image_subscriber", anonymous=True)
    print("Subscribe images from topic /camera/rgb/image_raw ...")

    image_subcriber = rospy.Subscriber("/camera/rgb/image_raw", Image, callback)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Finished!")
