from __future__ import division
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraFeedReader():
    """
    ### Class which provides an interface to read the current frame seen by the Turtlebot camera.
    Provides access to the realtime image data in different CV2 colour spaces.
    """
    
    def __init__(self):
        
        # Available feed data in CV2 format
        self.cv_image  = None    # Note: When set, this image is in the BGR (Blue, Green, Red) colour space
        self.hsv_image = None
        
        # Subscribers
        self.camera_subscriber = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)
    
    def callback(self, data):
        try:
            cv_bridge = CvBridge()
            
            # Convert ROS Image Message data to CV2 image data and set member
            self.cv_image = cv_bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Set HSV space image
            self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            
        except CvBridgeError as err:
            print(f"Error occured while bridging ROS image to CV2: {err}")
            