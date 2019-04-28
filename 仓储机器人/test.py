import rospy
import time

from geometry_msgs.msg import Twist


if __name__=="__main__":

	rospy.init_node('test_stop', anonymous=False)
	cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	while not rospy.is_shutdown():

		str = raw_input()
		print str
	
		if str == '000001':

			for i in range(1000):
					move_cmd = Twist()
					move_cmd.linear.x = 0.0
					move_cmd.angular.z = 0.0
					cmd_vel.publish(move_cmd)
					






	



