#!/usr/bin/env python
import roslib, sys, rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String



class Follower:

	def __init__(self):
		self.image_pub = rospy.Publisher("image_capture",Image,  queue_size=1)
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('/videoray/camera/image_raw', Image, self.image_callback)

	def image_callback(self, msg):
    		lower_yellow = numpy.array([ 21, 200, 102])
    		upper_yellow = numpy.array([30, 255, 255])
		#image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
		try:
			image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		except CvBridgeError as e:
			print(e)	
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		masked = cv2.bitwise_and(image, image, mask=mask)
		cv2.imshow("window", masked )
		cv2.waitKey(3)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image,"bgr8"))
		except CvBridgeError as e:
			print(e)


def main(args):
    ic = Follower()
    rospy.init_node('Follower', anonymous = True)
#rospy.init_node('follower')
    print('hellow from follower')
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
