from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *


#additional modules
import time
import numpy as np


class StateMachineController(ReflexController):
	"""A more sophisticated controller that uses a state machine."""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.dt = dt
		self.sim.updateWorld()
		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
		self.state = 'idle'
		self.dest = [[0, 0]]
		self.counter = 7

		#hard coded for 12 balls
		self.delta_x = 0.125
		self.delta_y = 0.166666667
		self.origin_x, self.origin_y = (0.0, 0.0)
		self.half_maxX = +0.25
		self.half_minX = -0.25
		self.half_maxY = +0.25
		self.half_minY = -0.25

		#locations of all 12 balls
		self.ball_locations = [[self.origin_x+self.half_minX + 0.5 * self.delta_x, self.origin_y+self.half_minY + 0.5 * self.delta_y],
						  [self.origin_x+self.half_minX + 1.5 * self.delta_x, self.origin_y+self.half_minY + 0.5 * self.delta_y],
						  [self.origin_x+self.half_minX + 2.5 * self.delta_x, self.origin_y+self.half_minY + 0.5 * self.delta_y],
					  	  [self.origin_x+self.half_minX + 3.5 * self.delta_x, self.origin_y+self.half_minY + 0.5 * self.delta_y],
					  	  [self.origin_x+self.half_minX + 0.5 * self.delta_x, self.origin_y+self.half_minY + 1.5 * self.delta_y],
					  	  [self.origin_x+self.half_minX + 1.5 * self.delta_x, self.origin_y+self.half_minY + 1.5 * self.delta_y],
					 	  [self.origin_x+self.half_minX + 2.5 * self.delta_x, self.origin_y+self.half_minY + 1.5 * self.delta_y],
					 	  [self.origin_x+self.half_minX + 3.5 * self.delta_x, self.origin_y+self.half_minY + 1.5 * self.delta_y],
					  	  [self.origin_x+self.half_minX + 0.5 * self.delta_x, self.origin_y+self.half_minY + 2.5 * self.delta_y],
					  	  [self.origin_x+self.half_minX + 1.5 * self.delta_x, self.origin_y+self.half_minY + 2.5 * self.delta_y],
						  [self.origin_x+self.half_minX + 2.5 * self.delta_x, self.origin_y+self.half_minY + 2.5 * self.delta_y],
						  [self.origin_x+self.half_minX + 3.5 * self.delta_x, self.origin_y+self.half_minY + 2.5 * self.delta_y]
				    																	          ] 

		print 'in the init method: ', self.ball_locations[0]
		# print 'shape of ball_locations: ', len(self.ball_locations)


	def __call__(self,controller):
		sim = self.sim
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = True
		ReflexController.__call__(self,controller)

		#collect the data from lidar
		# lidar_data = np.zeros((1, 180))
		# lidar_data = controller.sensor(33).getMeasurements()
		# lidar_data = np.array(lidar_data)
		# lidar_data = lidar_data[None, :] #shape (1, 180)


		#print 'start data: ', lidar_data[0, 0:10]
		# start_data = lidar_data[0, 0:10];

		
		"""
		decide the co-ordinates of the ball
		"""
		try:
			self.dest = self.ball_locations[0]
			print 'Target location for the gripper: ', self.dest
			self.ball_locations = self.ball_locations[1:]
		except IndexError:
			print 'No more balls in the box'


		"""
		now go to that particular location and lift the ball and drop it in the other box
		"""
		dest_x, dest_y = self.dest[0], self.dest[1]

		if math.ceil(sim.getTime()%self.counter) > 1 and math.ceil(sim.getTime()%self.counter) <= 2:
				desired = se3.mul((so3.identity(),[dest_x, dest_y, 0]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'lowering'
		elif self.state == 'lowering':
			if math.ceil(sim.getTime()%self.counter) > 2 and math.ceil(sim.getTime()%self.counter) <= 3:
				#this is needed to stop at the current position in case there's some residual velocity
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				#the controller sends a command to the hand: f1,f2,f3,preshape
				self.hand.setCommand([0.2, 0.2, 0.2, 0])
				self.state = 'closing'
		elif self.state == 'closing':
			if math.ceil(sim.getTime()%self.counter) > 3 and math.ceil(sim.getTime()%self.counter) <= 4:
				#the controller sends a command to the base after 1 s to lift the object
				desired = se3.mul((so3.identity(),[dest_x, dest_y, 0]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'raising'
		elif self.state == 'raising':
			if math.ceil(sim.getTime()%self.counter) > 4 and math.ceil(sim.getTime()%self.counter) <= 5:
				desired = se3.mul((so3.identity(),[dest_x, dest_y, 0]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'translate_pos_x'
		elif self.state == 'translate_pos_x':
			if math.ceil(sim.getTime()%self.counter) > 5 and math.ceil(sim.getTime()%self.counter) <= 6:
				#release the ball if there is any
				self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])
				self.state = 'release'
		elif self.state == 'release':
			if math.ceil(sim.getTime()%self.counter) > 6 and math.ceil(sim.getTime()%self.counter) <= 7:
				desired = se3.mul((so3.identity(),[dest_x, dest_y, 0]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'translate_neg_x'
		elif self.state == 'translate_neg_x':
			if math.ceil(sim.getTime()%self.counter) == 1.0:
				self.state = 'idle'

#navigate and find the location of the balls
	# def navigate():
	# 	try:
	# 		print 'shape of ball_locations: ', len(self.ball_locations)
	# 		self.dest = self.ball_locations[0]
	# 		self.ball_locations = self.ball_locations[1:]
	# 	except IndexError:
	# 		print 'No more balls are present!'
	# 	finally:
	# 		return 


		
def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""							
	return StateMachineController(sim,hand,dt)