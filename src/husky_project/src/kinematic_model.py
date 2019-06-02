#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def model(x,t,deltadot,v,l,a,b):
		delta = x[2]
		th1 = x[3]
		th2 = x[4]

		dxdt = np.cos(th1)*v
		dydt = np.sin(th1)*v
		dth1dt = np.tan(delta)*v/l
		dth2dt = (-(a/l)*np.tan(delta)*np.cos(th1-th2)+np.sin(th1-th2))*v/b
		return np.array([dxdt,dydt,deltadot,dth1dt,dth2dt])

class ODESolver:
	def __init__(self,l,a,b):
		self.l = l
		self.a = a
		self.b = b
		self.init = True
		self.x0 = np.zeros(5)
	

	def solve_ode(self,x0,t,delta,v):
		# Unpack vehicle lengths for brevity
		l = self.l; a = self.a; b = self.b
		n = len(v)
		# State storage for solution
		states = np.zeros((n,len(x0)),dtype=np.float32)
		# Record initial conditions
		states[0] = x0
		for i in range(1,n):
			# Span for next time step
			tspan = [t[i-1],t[i]]
			# Take forward difference of steering angle
			deltadot = (delta[i]-delta[i-1])/(t[i]-t[i-1])
			# Solve for next step
			x = odeint(model,x0,tspan,args=(deltadot,v[i],l,a,b))
			states[i] = x[1]
			# Next initial condition
			x0 = x[1]

		return states
	
	def test_model(self):
		# Speed of 15 m/s with steering angle of pi/6 after 1 second for
		# total time ten seconds
		n = 1001
		t_end = 10
		t = np.linspace(0,t_end,n)
		v = 15*np.ones(n)
		u = np.zeros(n)
		u[int(2*n/t_end):] = np.pi/12
		for i in range(int(n/t_end),int(2*n/t_end)):
			u[i] = (np.pi/12)*((i-int(n/t_end))/(int(n/t_end)))
		x0 = np.array([0,0,0,0,0])
		states = self.solve_ode(x0,t,u,v)
		return states

	def solve_ode_step(self,t,delta,v):
		if self.init:
			self.t_d1 = t
			self.delta_d1 = delta
			self.init = False
			return self.x0
		tspan = [self.t_d1,t]
		deltadot = (delta-self.delta_d1)/(t-self.t_d1)
		x = odeint(model,self.x0,tspan,args=(deltadot,v,self.l,self.a,self.b))
		self.x0 = x[1]
		self.t_d1 = t
		self.delta_d1 = delta
		print self.x0
		return x[1]
	
	def euler_solve_ode_step(self,t,delta,v):
		if self.init:
			self.t_d1 = t
			self.init = False
			return self.x0
		self.x0[2] = delta
		dx = model(self.x0,t,0,v,self.l,self.a,self.b)
		print(t-self.t_d1)
		self.x0 = dx*(t-self.t_d1)
		self.t_d1 = t
		self.x0[2] = delta
		print self.x0
		return self.x0

def test_script():
	ode = ODESolver(226*.0254,9*.0254,16.104-0.914-3.083-1.2446/2)
	states = ode.test_model()
	x = states[:,0]
	y = states[:,1]
	gamma = states[:,3]-states[:,4]
	t = np.linspace(0,10,1001)
	print(x)
	print(y)
	plt.plot(t,gamma)
	plt.show()

class ArticulationAngleEstimator:
	def __init__(self,l,a,b):
		self.l = l
		self.a = a
		self.b = b
		self.joint_name = "trailer_joint"
		self.joint_state = JointState()
		self.joint_state.name = [self.joint_name,"left_trailer_wheel_joint","right_trailer_wheel_joint"]
		self.joint_state.position = [0.0,0.0,0.0]
		self.joint_state.velocity = [0.0,0.0,0.0]
		self.vel_sub = rospy.Subscriber("cmd_vel",Twist,self.vel_callback,queue_size=1)
		self.pub = rospy.Publisher("joint_states",JointState,queue_size=1)
		self.solver = ODESolver(l,a,b)

	def vel_callback(self,msg):
		[v,delta] = self.twist_to_ackerman(msg)
		if abs(v) > 0.01:
			t = rospy.get_time()
			state = self.solver.solve_ode_step(t,delta,v)
			articulation_angle = state[3]-state[4]
			self.joint_state.position = [-articulation_angle,0.0,0.0]
			self.joint_state.name = [self.joint_name,"left_trailer_wheel_joint","right_trailer_wheel_joint"]
			self.joint_state.velocity+=[0.0,0.0,0.0]
			self.joint_state.header.stamp = rospy.Time.now()
			self.pub.publish(self.joint_state)

	def twist_to_ackerman(self,twist):
		w = twist.angular.z
		v = twist.linear.x
		if abs(v) < 0.01:
			delta = 0
		else:
			delta = np.arctan2(self.l*w,v)
		print(v,delta)
		return [v,delta]
		


if __name__ == "__main__":
	rospy.init_node('articulation_angle_est')
	estimator = ArticulationAngleEstimator(0.99,0.99/2,(15+36)*0.0254)
	rospy.spin()

