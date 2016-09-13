from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *

#additional imported modules
import time
import math


class StateMachineController(ReflexController):
	"""A more sophisticated controller that uses a state machine."""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.dt = dt
		self.sim.updateWorld()
		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
		self.state = 'idle'
		self.counter = 7
		self.ball_count = 0
		self.dx = 0.025
		self.dy1 = -0.0125
		self.dy2 = +0.0125

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
		if self.ball_count == 0:
			x = -0.1875
			y = -0.1666666665+self.dy1
		elif self.ball_count == 1:
			x = -0.0625
			y = -0.1666666665+self.dy1
		elif self.ball_count == 2:
			x = 0.0625
			y = -0.1666666665+self.dy1
		elif self.ball_count == 3:
			x = +0.1875
			y = -0.1666666665+self.dy1
		elif self.ball_count == 4:
			x = -0.1875
			y = 4.999999858590343e-10
		elif self.ball_count == 5:
			x = -0.0625
			y = 4.999999858590343e-10
		elif self.ball_count == 6:
			x = 0.0625
			y = 4.999999858590343e-10
		elif self.ball_count == 7:
			x = 0.1875
			y = 4.999999858590343e-10
		elif self.ball_count == 8:
			x = -0.1875
			y = 0.16666666749999998+self.dy2
		elif self.ball_count == 9:
			x = -0.0625
			y = 0.16666666749999998+self.dy2
		elif self.ball_count == 10:
			x = 0.0625
			y = 0.16666666749999998+self.dy2
		elif self.ball_count == 11:
			x = -0.1875
			y = 0.16666666749999998+self.dy2
		else:
			print 'done!'



		"""
		navigate and pick the ball
		"""
		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil(sim.getTime()%self.counter)) + \
				"\t\tTime: " + str(sim.getTime())
		if self.state == 'idle':
			self.hand.setCommand([math.radians(25), math.radians(25), math.radians(25), 0])
			if math.ceil(sim.getTime()%self.counter) > 1 and math.ceil(sim.getTime()%self.counter) <= 2:
				#print 'xform: ', xform 
				#[0,0,-0.10]
				desired = se3.mul((so3.identity(),[x+self.dx, y, -0.235]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.ball_count += 1
				self.state = 'lowering'
		elif self.state == 'lowering':
			if math.ceil(sim.getTime()%self.counter) > 2 and math.ceil(sim.getTime()%self.counter) <= 3:
				#this is needed to stop at the current position in case there's some residual velocity
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				#the controller sends a command to the hand: f1,f2,f3,preshape
				self.hand.setCommand([0.2, 0.2, 0.2, 0])
				self.state = 'closing'
		elif self.state == 'closing':
			#start = time.time()
			if math.ceil(sim.getTime()%self.counter) > 3 and math.ceil(sim.getTime()%self.counter) <= 4:
				#the controller sends a command to the base after 1 s to lift the object
				#[0,0,0.10]
				desired = se3.mul((so3.identity(),[x+self.dx, y, +0.2]),xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'raising'
				#print 'time for raising: ', time.time()-start
		elif self.state == 'raising':
			if math.ceil(sim.getTime()%self.counter) > 4 and math.ceil(sim.getTime()%self.counter) <= 5:
				desired = se3.mul((so3.identity(), [+0.6, 0, +0.2]), xform)
				send_moving_base_xform_linear(controller, desired[0], desired[1], 0.5)
				self.state = 'translate_pos_x'
		elif self.state == 'translate_pos_x':
			if math.ceil(sim.getTime()%self.counter) > 5 and math.ceil(sim.getTime()%self.counter) <= 6:
				#release the ball if there is any
				self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])
				self.state = 'release'
		elif self.state == 'release':
			if math.ceil(sim.getTime()%self.counter) > 6 and math.ceil(sim.getTime()%self.counter) <= 7:
				desired = se3.mul((so3.identity(), [0, 0, +0.2]), xform)
				send_moving_base_xform_linear(controller, desired[0], desired[1], 0.5)
				self.state = 'translate_neg_x'
		elif self.state == 'translate_neg_x':
			if math.ceil(sim.getTime()%self.counter) == 1.0:
				self.state = 'idle'

		
def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	#print 'make is called...'
	return StateMachineController(sim,hand,dt)