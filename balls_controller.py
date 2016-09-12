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
		self.counter = 7;

	def __call__(self,controller):
		sim = self.sim
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = True
		ReflexController.__call__(self,controller)

		#print 'Time :', sim.getTime()
		#controller state machine

		#print 'List of all sensors available are: \n'
		#for idx in xrange(40):
			#print controller.sensor(idx).name()
		print 'length of lidar sensor data: ', len(controller.sensor(33).getMeasurements())
		#time.sleep(0.5)
		print '\n\n\n'
		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil(sim.getTime()%self.counter)) + \
				"\t\tTime: " + str(sim.getTime())
		if self.state == 'idle':
			#print controller.sensor(33).getMeasurements()
			#print 'sim.getTime()=', sim.getTime()
			if math.ceil(sim.getTime()%self.counter) > 1 and math.ceil(sim.getTime()%self.counter) <= 2:
				#print 'xform: ', xform 
				#[0,0,-0.10]
				desired = se3.mul((so3.identity(),[0, 0, -0.2]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'lowering'
				#print controller.sensor(33).getMeasurements()
				#print controller.sensor(34).getMeasurements()
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
				desired = se3.mul((so3.identity(),[0, 0, +0.2]),xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'raising'
				#print 'time for raising: ', time.time()-start
		elif self.state == 'raising':
			if math.ceil(sim.getTime()%self.counter) > 4 and math.ceil(sim.getTime()%self.counter) <= 5:
				desired = se3.mul((so3.identity(), [+0.7, 0, +0.2]), xform)
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