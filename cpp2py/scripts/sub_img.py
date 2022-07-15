import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2

class CameraReader(object):
    def __init__(self, topic_name):
        """
        接收kinect的信息
        """
        self.image = None
        self.cv_bridge = CvBridge()
        self.topic = topic_name
        self.image_sub = rospy.Subscriber(self.topic, Image, self.callbackRGB)

    def callbackRGB(self, msg):
        # rospy.loginfo('Image has received...')
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, 'rgba8')[:,:,:3]
        print(self.image.shape)

    def __iter__(self):
        return self

    def __next__(self):

        if self.image is None:
            raise StopIteration

        return self.image
        
        
if __name__ == "__main__":
    rospy.init_node('human_pose', anonymous=True)
    camera_topic = "kinectSDK/color"
    frame_provider = CameraReader(camera_topic)
    
    while not rospy.is_shutdown():
        for img in frame_provider:
        
            cv2.imshow('img_py', img)
            cv2.waitKey(1)