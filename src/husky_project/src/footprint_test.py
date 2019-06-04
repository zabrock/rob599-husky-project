#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Polygon

def footprint_msg():
	msg = Polygon()
	p1 = Point32()
	val = 0.5
	p1.x = -val; p1.y =-val; p1.z = 0;
	p2 = Point32()
	p2.x = -val; p2.y = val; p2.z = 0;
	p3 = Point32()
	p3.x = val; p3.y = val; p3.z = 0;
	p4 = Point32()
	p4.x = val; p4.y = -val; p4.z = 0;
	msg.points = [p1,p2,p3,p4]
	return msg

if __name__ == '__main__':
	rospy.init_node('test_footprint_msg')
	loc_pub = rospy.Publisher('move_base/local_costmap/footprint',Polygon,queue_size=1)
	glb_pub = rospy.Publisher('move_base/global_costmap/footprint',Polygon,queue_size=1)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		loc_pub.publish(footprint_msg())
		glb_pub.publish(footprint_msg())
		rate.sleep()
