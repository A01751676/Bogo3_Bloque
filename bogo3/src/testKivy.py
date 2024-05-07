#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder 
from std_msgs.msg import Bool
from std_msgs.msg import Int16

class Bogo3App (MDApp):
	def __init__ (self, **kwargs):
		super().__init__(**kwargs)
		self.screen = Builder.load_file("ros_gui.kv")
		self.pub = rospy.Publisher("/button", Bool, queue_size= 1)
		self.pub_slider = rospy.Publisher("/slider", Int16, queue_size= 1)

	def my_func(self, *args):
		# ejemplo de publicador y de cambio de label en app
		print("boton")
		self.pub.publish(True)
		self.screen.ids.label1.text = "presionado"


	def slider1(self, value):
		# ejemplo de publicador para un slider
		print(int(value))
		self.pub_slider.publish(int(value))
		rospy.set_param("slider", int(value))
		

	def build(self):
		return self.screen

if __name__=='__main__':
	rospy.init_node("test_kivy")
	bogo3_gui = Bogo3App()
	bogo3_gui.run()
