#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
  def __init__(self):
      self.videoCapture = cv2.VideoCapture('/dev/video1')

      self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                         Twist, queue_size=1)
      self.twist = Twist()

      while not rospy.is_shutdown():
          success, image = self.videoCapture.read()
          hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
          hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
          lower_yellow = numpy.array([ 0,  43,  46])
          upper_yellow = numpy.array([10, 255, 250])
          mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

          h, w, d = image.shape
          search_top = 3*h/4
          search_bot = 3*h/4 + 20
          mask[0:search_top, 0:w] = 0
          mask[search_bot:h, 0:w] = 0
          M = cv2.moments(mask)
          if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # BEGIN CONTROL
            err = cx - w/2
            self.twist.linear.x = 0.26
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
          cv2.imshow("mask",mask)
          cv2.imshow("output", image)
          cv2.waitKey(3)


if __name__ == '__main__':
  rospy.init_node('follower')
  follower = Follower()

# END ALL
