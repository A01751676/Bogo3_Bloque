#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder 

class Bogo3App (MDApp):
	def __init__ (self, **kwargs):
		super().__init__(**kwargs)
		self.screen = Builder.load_file("ros_gui.kv")

	def build(self):
		return self.screen

if __name__=='__main__':
	bogo3_gui = Bogo3App()
	bogo3_gui.run()
	rospy.init_node("test_kivy")
