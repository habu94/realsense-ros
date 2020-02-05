#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float64
import sys
import os
import re

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.pub = rospy.Publisher('depth', Float64, queue_size=10)

    def imageDepthCallback(self, data):
        pub = rospy.Publisher('depth', Float64, queue_size=10)
        #pub.publish(cv_image[pix[1], pix[0]])
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            num = cv_image[pix[1], pix[0]]
            num = 0.3 - 0.001 * num
            #num = float(num)
            pub.publish(num)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
 
def main():
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
