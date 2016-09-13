from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *

#additional imported modules
import time
import math
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
		self.counter = 20;
		self.delta = 0.07
		self.flag = 1

		#analyse the data from lidar data
		self._list = np.zeros((1, 180))
		temp_list = np.zeros((1, 180))

	def __call__(self,controller):
		sim = self.sim
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = True
		ReflexController.__call__(self,controller)

		# if self.flag == 1:
		# 	_list = np.zeros((180, 1))
		# 	self.flag = 0

		# print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil(sim.getTime()%self.counter)) + \
		#  		"\t\tTime: " + str(sim.getTime())

		#print the data from the lidar sensor
		lidar_data = np.zeros((1, 180))
		lidar_data = controller.sensor(33).getMeasurements()
		lidar_data = np.array(lidar_data)
		lidar_data = lidar_data[None, :]

		#print 'shape: ', lidar_data.shape
		# print 'new shape: ', ((lidar_data[None, :])).shape
		#lidar_data = lidar_data[0:180]
		# print 'lidar_data: ', (lidar_data[None, :]).shape
		#print lidar_data[140:180]
		#print 'self._list: ', self._list.shape
		#print 'type of lidar data: ', type(lidar_data)
		#print 'size and shape :', lidar_data.size, lidar_data.shape
		#print 'subset of lidar data: ', lidar_data
		#print 'self._list: ', self._list.shape
		#print 'lidar_data: ', lidar_data.shape


		# file = open('/home/kv/Desktop/lidar_data.txt', 'a')
		# file.write(str(controller.sensor(33).getMeasurements()))
		# file.write('\n')
		# if sim.getTime() > 7:
		# 	print 'done!!'
		# 	file.close()
		
		print 'mid data: ', lidar_data[0, 85:95]
		print 'start data: ', lidar_data[0, 0:10]
		print 'last data : ', lidar_data[0, 170:179]
		print "\n"
		#self._list = np.vstack((self._list, lidar_data))
		# if sim.getTime() >= 6:
		# 	self._list = self._list[1:, :]
		# 	np.savez('/home/kv/Desktop/lidar_data.npz', dist=self._list) 

		if self.state == 'idle':
			if math.ceil(sim.getTime()%self.counter) > 0 and math.ceil(sim.getTime()%self.counter) <= 1:
				desired = se3.mul((so3.identity(),[0.25-self.delta, -0.25+self.delta, 0]),xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1],0.5)
				self.state = 'BR'
		elif self.state == 'BR':
			#print 'data[85 to 90]: ', lidar_data[0, 85:91]
			if math.ceil(sim.getTime()%self.counter) > 5 and math.ceil(sim.getTime()%self.counter) <= 10:
				desired = se3.mul((so3.identity(),[-0.25+self.delta, -0.25+self.delta, 0]),xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1],0.5)
				self.state = 'BL'
		elif self.state == 'BL':
			if math.ceil(sim.getTime()%self.counter) > 10 and math.ceil(sim.getTime()%self.counter) <= 15:
				desired = se3.mul((so3.identity(),[-0.25+self.delta, 0.25-self.delta, 0]),xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1],0.5)
				self.state = 'TL'
		elif self.state == 'TL':
			if math.ceil(sim.getTime()%self.counter) > 15 and math.ceil(sim.getTime()%self.counter) <= 20:
				desired = se3.mul((so3.identity(),[0.25-self.delta, 0.25-self.delta, 0]),xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1],0.5)
				self.state = 'TR'
		elif self.state == 'TR':
			if math.ceil(sim.getTime()%self.counter) == 1.0:
				self.state = 'idle'


		#print 'Time :', sim.getTime()
		#controller state machine

		#print 'List of all sensors available are: \n'
		#for idx in xrange(40):
			#print controller.sensor(idx).name()
		#print 'lidar data: ', controller.sensor(33).getMeasurements()
		# temp_list = controller.sensor(33).getMeasurements()
		# np.vstack((_list, temp_list))
		
		# if sim.getTime() > 15:
		# 	_list = _list[1:]
		# 	np.savez('/home/kv/Desktop/lidar_data.npz', dist=_list)
		# #time.sleep(1)
		# print '\n\n\n'
		# print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil(sim.getTime()%self.counter)) + \
		# 		"\t\tTime: " + str(sim.getTime())
		# if self.state == 'idle':
		# 	#print controller.sensor(33).getMeasurements()
		# 	#print 'sim.getTime()=', sim.getTime()
		# 	if math.ceil(sim.getTime()%self.counter) > 1 and math.ceil(sim.getTime()%self.counter) <= 2:
		# 		#print 'xform: ', xform 
		# 		#[0,0,-0.10]
		# 		desired = se3.mul((so3.identity(),[0, 0, -0.2]), xform)
		# 		send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
		# 		self.state = 'lowering'
		# 		#print controller.sensor(33).getMeasurements()
		# 		#print controller.sensor(34).getMeasurements()
		# elif self.state == 'lowering':
		# 	if math.ceil(sim.getTime()%self.counter) > 2 and math.ceil(sim.getTime()%self.counter) <= 3:
		# 		#this is needed to stop at the current position in case there's some residual velocity
		# 		controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
		# 		#the controller sends a command to the hand: f1,f2,f3,preshape
		# 		self.hand.setCommand([0.2, 0.2, 0.2, 0])
		# 		self.state = 'closing'
		# elif self.state == 'closing':
		# 	#start = time.time()
		# 	if math.ceil(sim.getTime()%self.counter) > 3 and math.ceil(sim.getTime()%self.counter) <= 4:
		# 		#the controller sends a command to the base after 1 s to lift the object
		# 		#[0,0,0.10]
		# 		desired = se3.mul((so3.identity(),[0, 0, +0.2]),xform)
		# 		send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
		# 		self.state = 'raising'
		# 		#print 'time for raising: ', time.time()-start
		# elif self.state == 'raising':
		# 	if math.ceil(sim.getTime()%self.counter) > 4 and math.ceil(sim.getTime()%self.counter) <= 5:
		# 		desired = se3.mul((so3.identity(), [+0.7, 0, +0.2]), xform)
		# 		send_moving_base_xform_linear(controller, desired[0], desired[1], 0.5)
		# 		self.state = 'translate_pos_x'
		# elif self.state == 'translate_pos_x':
		# 	if math.ceil(sim.getTime()%self.counter) > 5 and math.ceil(sim.getTime()%self.counter) <= 6:
		# 		#release the ball if there is any
		# 		self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])
		# 		self.state = 'release'
		# elif self.state == 'release':
		# 	if math.ceil(sim.getTime()%self.counter) > 6 and math.ceil(sim.getTime()%self.counter) <= 7:
		# 		desired = se3.mul((so3.identity(), [0, 0, +0.2]), xform)
		# 		send_moving_base_xform_linear(controller, desired[0], desired[1], 0.5)
		# 		self.state = 'translate_neg_x'
		# elif self.state == 'translate_neg_x':
		# 	if math.ceil(sim.getTime()%self.counter) == 1.0:
		# 		self.state = 'idle'

		
def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	#print 'make is called...'
	return StateMachineController(sim,hand,dt)