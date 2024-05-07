#!/usr/bin/env python  

import numpy as np
import rospy
import os

class calibrateRobot:
	def __init__ (self):
		Tpose = rospy.get_param("/Tpose")
		print(Tpose)
	def stop(self):
		os.system("rosparam dump tPoseConfig.yaml")
		print("stopping")

	def main(self):
		pass

if __name__=='__main__':
	# inicializacion 
	rospy.init_node("calibrate_robot")
	bogo3 = calibrateRobot();
	rate = rospy.Rate(10)
	#rospy.on_shutdown(robotLoc.stop())

	try:
		while not rospy.is_shutdown():
			bogo3.main()
			rate.sleep()
		bogo3.stop()

	except rospy.ROSInterruptException:
		bogo3.stop()
