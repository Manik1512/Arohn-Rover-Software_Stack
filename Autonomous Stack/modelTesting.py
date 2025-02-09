#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image
        cv2.imshow("Kinect HD Image", frame)
        cv2.waitKey(1)  # Add a small delay for display
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('kinect_hd_image_viewer', anonymous=True)
    
    # Initialize CvBridge
    bridge = CvBridge()

    # Subscribe to the Kinect2 topic

    rospy.Subscriber('/kinect2/hd/image_color_rect', Image, image_callback)

    # Keep the node running
    rospy.spin()

    # Clean up OpenCV windows
    cv2.destroyAllWindows()
