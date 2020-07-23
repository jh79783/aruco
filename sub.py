#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class test_sub():
	def __init__(self):

		self.tw_sub = rospy.Subscriber('/id_msg', String, self.callback)

	def callback(self,d):
		
		print(d.data)
		


		
def main():
	rospy.init_node('test_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
