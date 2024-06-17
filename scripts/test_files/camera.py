import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class CameraViewer:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()  # Bridge to convert ROS image messages to OpenCV format
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
    
    def image_callback(self, msg):
        """Callback function to handle image frames received from the camera."""
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display the image using OpenCV
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)  # Refresh display
        except cv_bridge.CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node("camera_viewer", anonymous=True)
    cv = CameraViewer()
    rospy.spin()  # Keep the script alive until the node is shutdown