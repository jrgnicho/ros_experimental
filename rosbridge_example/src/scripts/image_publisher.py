#!/usr/bin/env python
import rospy
import roslib.packages
import sys
import cv2
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__=="__main__":
    
    # initialization
    rospy.init_node("image_publisher")
    
    # load resources
    img_path = roslib.packages.get_pkg_dir("rosbridge_example") + "/resources/ros_industrial_logo.png"    
    img = cv.LoadImage(img_path,1)
#     img = cv2.imread(img_path,cv2.CV_LOAD_IMAGE_COLOR)
#     cv2.imshow("Image",img)
#     cv2.waitKey(0)
#     cv2.destroyWindow("Image")

    # publisher
    img_pub = rospy.Publisher("image_topic",Image)
    
    # convert image to ros message
    cv_bridge = CvBridge()
    img_msg = cv_bridge.cv_to_imgmsg(img, "bgr8")
    
    # publishing image
    while not rospy.is_shutdown():
        
        rospy.sleep(0.4)
        
        try:
            img_pub.publish(img_msg)
        except CvBridgeError, e:
            print e 
        
        
    
    


