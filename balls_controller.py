from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import matplotlib.pyplot as plt
import numpy as np

#additional imported modules
import time
import math

"""
function to decide whether ball is in contact with fingers
"""
def contact_gripper(sim, controller):
	"""
	reading the finger sensor data
	"""
	f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
	contact_sensors = f1_proximal_takktile_sensors + f1_distal_takktile_sensors + f2_proximal_takktile_sensors + f2_distal_takktile_sensors + f3_proximal_takktile_sensors + f3_distal_takktile_sensors		
	f1_contact = [s.getMeasurements()[0] for s in f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f1_distal_takktile_sensors]
	f2_contact = [s.getMeasurements()[0] for s in f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f2_distal_takktile_sensors]
	f3_contact = [s.getMeasurements()[0] for s in f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f3_distal_takktile_sensors]
	print "Contact sensors"
	print "  finger 1:",[int(v) for v in f1_contact]
	print "  finger 2:",[int(v) for v in f2_contact]
	print "  finger 3:",[int(v) for v in f3_contact]
	# print '\n\n'

	"""
	check whether ball is held
	"""
	if f1_contact[5] == 1 or f2_contact[5] == 1 or f3_contact[5] == 1:
		return True
	elif f1_contact[4] == 1 or f2_contact[4] == 1 or f3_contact[4] == 1:
		return True
	else:
		return False




class StateMachineController(ReflexController):
	"""A more sophisticated controller that uses a state machine."""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.dt = dt
		self.sim.updateWorld()
		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
		self.state = 'idle'
		self.counter = 11
		self.ball_count = 0
		self.dx = 0.0
		self.dy1 = -0.0125
		self.dy2 = +0.0350
		self.X_loc = []
		self.Y_loc = []
		self.done = True
		self.resize_data = np.zeros(360)
		self.offset = 0 
		self.average = 0 

	def __call__(self,controller):
		sim = self.sim
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = True
		ReflexController.__call__(self,controller)

		"""
		decide the position of the ball
		"""

		print 'ball count: ', self.ball_count

		"""
		new method to locate the position of balls
		"""
		box_dims = (0.5,0.5,0.3)
		ballsperlayer = 12
		w = int(math.ceil(math.sqrt(ballsperlayer)))
		h = int(math.ceil(float(ballsperlayer)/float(w)))

		
		if self.done == True:
			for i in xrange(ballsperlayer):
				x = i % w
				y = i / w
				x = (x - (w-1)*0.5)*box_dims[0]*0.7/(w-1)
				y = (y - (h-1)*0.5)*box_dims[1]*0.7/(h-1)

				if i == 0 or i == 1 or i == 2 or i == 3:
					y = y + self.dy2

				if i == 0 or i == 4 or i == 8:
					x = x - 0.015 
				# print 'balls # ', i
				# print 'x: ', x
				# print 'y: ', y
				# print '\n\n'
				self.X_loc.append(x)
				self.Y_loc.append(y)
			self.done = False


		# print 'x_loc: ', self.X_loc
		# print '\n\n'
		# print 'y_lco: ', self.Y_loc
		# time.sleep(5)
		try:
			x = self.X_loc[self.ball_count]
			y = self.Y_loc[self.ball_count]
			# if self.ball_count == 0 or self.ball_count == 3 or self.ball_count == 6:
			# 	x = x + 0.015
			# # 	y = y
			# # elif self.ball_count == 1:
			# # 	x = x + 0.06
		except IndexError:
			x = self.X_loc[ballsperlayer-1]
			y = self.Y_loc[ballsperlayer-1]



		if contact_gripper(sim, controller):
			print 'ball is held'
		else:
			print 'ball is NOT held'

		"""
		navigate and pick the ball
		"""
		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil((sim.getTime()+self.offset)%self.counter)) + \
				"\t\tTime: " + str(sim.getTime())
		print " Offset : ", self.offset
		print " Average : ", self.average
		if self.ball_count == ballsperlayer :
			self.ball_count = 0

		if self.state == 'idle':
			self.hand.setCommand([math.radians(90), math.radians(90), math.radians(90), 0])
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 1 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 3:
				print "Going above the desired point.........................."
				desired = se3.mul((so3.identity(),[x+self.dx, y, 0.1]), xform)
				#desired = se3.mul((so3.identity(),[0,0 , 0.1]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'translate_in_plane'

		elif self.state == 'translate_in_plane':
			lidar_data = controller.sensor(33).getMeasurements()
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 4 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 5:	
				self.resize_data = lidar_data[0:180]
				x_coordinate = [ i for i in range(len(self.resize_data)) ]
				plt.plot(x_coordinate,self.resize_data)
				plt.savefig("PS_1_3.png")
				sum = 0 
				for i in range(88,92) :
					sum = sum + self.resize_data[i]
				avg = sum / 4
				self.average = avg
				#plt.show()
				self.hand.setCommand([math.radians(25), math.radians(25), math.radians(25), 0])
				desired = se3.mul((so3.rotation((0, 0, 1), math.radians(90)),[x+self.dx, y, 0.1]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.9)
				self.state = 'finger_reposition'
		elif self.state == 'finger_reposition':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 5 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 6:
				# exit(9)
				if(self.average > 0.53) :
					# No ball below
					self.state = 'idle'
					self.offset = -math.floor(sim.getTime())
					self.ball_count += 1 
				else : 
					# Balls below
					self.hand.setCommand([math.radians(25), math.radians(25), math.radians(25), 0])
					desired = se3.mul((so3.rotation((0, 0, 1), math.radians(90)),[x+self.dx, y, -0.285]), xform)
					send_moving_base_xform_linear(controller,desired[0],desired[1], 0.9)
					self.state = 'lowering'

		elif self.state == 'lowering':
			# x_coordinate = [ i for i in range(len(lidar_data)) ]
			# plt.plot(x_coordinate,lidar_data)
			# plt.show()
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 6 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 7:
				#this is needed to stop at the current position in case there's some residual velocity
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				#the controller sends a command to the hand: f1,f2,f3,preshape

				self.hand.setCommand([0.2, 0.2, 0.2, 0])
				self.state = 'closing'
		elif self.state == 'closing':
			#start = time.time()
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 7 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 8:
				#the controller sends a command to the base after 1 s to lift the object
				#[0,0,0.10]
				desired = se3.mul((so3.rotation((0, 0, 1), math.radians(90)),[x+self.dx, y, 0.1]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 1.0)
				self.ball_count += 1
				# if self.ball_count == 1:
				# 	self.ball_count += 1
				self.state = 'raising'
				#print 'time for raising: ', time.time()-start
		elif self.state == 'raising':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 8 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 9:
				desired = se3.mul((so3.rotation((0, 0, 1), math.radians(90)), [+0.5, 0,0.1]), xform)
				send_moving_base_xform_linear(controller, desired[0], desired[1], 1.0)
				self.state = 'translate_pos_x'
		elif self.state == 'translate_pos_x':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 9 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 10:
				#release the ball if there is any
				self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])

				self.state = 'release'
		elif self.state == 'release':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 10 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 11:
				desired = se3.mul((so3.rotation((0, 0, 1), math.radians(90)), [0, 0, 0.1]), xform)
				send_moving_base_xform_linear(controller, desired[0], desired[1], 0.5)
				self.state = 'translate_neg_x'
		elif self.state == 'translate_neg_x':
			#print " Time function : " + str(math.ceil((sim.getTime()+self.offset)%self.counter))
			if math.ceil((sim.getTime()+self.offset)%self.counter) == 1.0:
				self.state = 'idle'

	

def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	#print 'make is called...'
	return StateMachineController(sim,hand,dt)


