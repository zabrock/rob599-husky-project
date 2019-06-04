#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np

class FootprintPublisher:
	def __init__(self,l,a,b,robot_width,trailer_width):
		self.l = l
		self.a = a
		self.b = b
		self.v = 0.0
		self.w = 0.0
		self.rob_width = robot_width
		self.tr_width = trailer_width
		self.width_diff = self.tr_width - self.rob_width
		self.angle_sub = rospy.Subscriber('articulation_angle',Float64,self.simple_angle_callback,queue_size=1)
		self.odom_sub = rospy.Subscriber("odometry/filtered",Odometry,self.odom_callback,queue_size=1)
		self.loc_pub = rospy.Publisher('move_base/local_costmap/footprint',Polygon,queue_size=1)
		self.glb_pub = rospy.Publisher('move_base/global_costmap/footprint',Polygon,queue_size=1)

	def angle_callback(self,msg):
		gamma = msg.data
		if abs(gamma) < 0.1:
			offtrack = 0
		else:
			v_wa = np.divide(self.v,self.w*self.a)
			offtrack = np.divide(self.v,self.w) + self.b*np.divide(np.tan(gamma)+v_wa,1-v_wa*np.tan(gamma))
		self.update_footprint(offtrack)

	def simple_angle_callback(self,msg):
		gamma = msg.data
		offtrack = self.b*np.sin(gamma)
		self.update_footprint(offtrack)

	def odom_callback(self,msg):
		self.v = msg.twist.twist.linear.x
		self.w = msg.twist.twist.angular.z

	def update_footprint(self,offtrack):
		width = self.rob_width + self.width_diff + abs(offtrack)
		if width < self.rob_width:
			width = self.rob_width

		if offtrack > 0:
			left_width = width
			right_width = self.rob_width
		else:
			right_width = width
			left_width = self.rob_width

		msg = Polygon()
		p1 = Point32()
		val = 0.5
		p1.x = -self.l/2; p1.y =-left_width/2; p1.z = 0;
		p2 = Point32()
		p2.x = -self.l/2; p2.y = right_width/2; p2.z = 0;
		p3 = Point32()
		p3.x = self.l/2; p3.y = right_width/2; p3.z = 0;
		p4 = Point32()
		p4.x = self.l/2; p4.y = -left_width/2; p4.z = 0;
		msg.points = [p1,p2,p3,p4]
		self.loc_pub.publish(msg)
		self.glb_pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('footprint_adjuster')
	ft_pub = FootprintPublisher(0.99,0.99/2,(15+28)*0.0254,0.66,(24+4)*0.0254)
	
	rospy.spin()
