#!/usr/bin/env python

# Every python controller needs these lines
import rospy

# The goal command message
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry

class Robot:
	def __init__(self):
		# Initialize empty inventory
		self.inventory = []
		self.x = 0.0
		self.y = 0.0

	def test_inventory(self):
		# Set inventory to a test case
		self.inventory = ['cheese','milk','screwdriver','toolbox','puppy']

	def print_inventory(self):
		# Print everything in the inventory as a numbered list
		if len(self.inventory) < 1:
			print("Inventory is empty!")
		else:
			print("Items currently in inventory:")
			for idx in range(0,len(self.inventory)):
				print('{}. '.format(idx+1) + self.inventory[idx] + '\n')

	def take_from_inventory(self,item_idx):
		# Remove item from inventory
		if item_idx >= len(self.inventory) or item_idx < 0:
			print("Error: item index incorrect in take_from_inventory!")
		else:
			removed_item = self.inventory.pop(item_idx)
			print("Successfully removed " + removed_item + " from inventory")
	def add_to_inventory(self,item_name):
		self.inventory.append(item_name)
		print(item_name + " successfully added to inventory")
	
	def odom_callback(msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

def help_dialog():
	print("\nCommand:\tAction:")
	print("help\t\tLists possible commands")
	print("inventory\tLists items currently in robot inventory")
	print("delivery\tInitiates delivery of item from inventory")
	print("add_item\tInitiates addition of item to inventory")
	print("quit\t\tExits program")
	print("\n")

def go_to_goal():
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
	return msg
	
	
def main():
	rospy.init_node('goal_sender')
	robot = Robot()
	# A subscriber for the laser scan data
	sub = rospy.Subscriber('odom', Odometry, robot.odom_callback)
	pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
	
	robot.test_inventory()
	accept_commands = True
	while accept_commands and not rospy.is_shutdown():
		# Ask the user for input
		cmd = raw_input('Enter command (enter help for list of commands): ')
		if cmd == 'help':
			help_dialog()
		elif cmd == 'inventory':
			robot.print_inventory()
		elif cmd == 'delivery':
			if len(robot.inventory) > 0:
				delivery_in_progress = True
				while delivery_in_progress:
					robot.print_inventory()
					idx = raw_input('Enter index of item you would like to have delivered: ')
					try:
						idx = int(idx)
					except ValueError:
						print("User didn't enter a number!")
					if idx > 0 and idx <= len(robot.inventory):
						pub.publish(go_to_goal())
						print('Delivering item')
						robot.take_from_inventory(idx-1)
						delivery_in_progress = False
						
					else:
						print('Entered index doesn\'t correlate to an item in inventory!')
			else:
				print("Can't deliver - nothing in inventory!")
		elif cmd == 'add_item':	
			item_name = raw_input('Enter name of item to be added to inventory: ')	
			robot.add_to_inventory(item_name)
		elif cmd == 'quit':
			print("Goodbye!")
			accept_commands = False
		else:
			print('Command not recognized!')
	
	

if __name__ == '__main__':
	main()
