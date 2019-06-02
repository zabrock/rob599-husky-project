#!/usr/bin/env python

# Every python controller needs these lines
import rospy

# The goal command message
from move_base_msgs.msg import MoveBaseActionGoal

if __name__ == '__main__':
	rospy.init_node('goal_sender')
	pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)

	keep_going = True
	while keep_going and not rospy.is_shutdown():
		# A publisher for the move data
		
		x, y = raw_input("Enter goal coordinates: ").split()
		try:
			x = float(x); y = float(y)
		except ValueError:
			print("User did not enter numbers!")

		msg = MoveBaseActionGoal()
		msg.goal.target_pose.header.stamp = rospy.Time.now()
		msg.goal.target_pose.header.frame_id = 'odom'
		msg.goal.target_pose.pose.position.x = x
		msg.goal.target_pose.pose.position.y = y
		msg.goal.target_pose.pose.orientation.w = 1.0
		pub.publish(msg)
		print("Sent goal to robot")
		cmd = raw_input("Send another goal (y/n)? ")
		if cmd == 'n':
			keep_going = False
			
