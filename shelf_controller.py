from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import matplotlib.pyplot as plt
import numpy as np
#additinal imported libraries
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
		self.counter = 8
		self.ball_count = 0
		self.dx = 0.025
		self.dy1 = -0.0125
		self.dy2 = +0.0125
		self.mypos = [0,0,0]
		self.resize_data = np.zeros(360)

	def __call__(self,controller):

		sim = self.sim
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = True
		ReflexController.__call__(self,controller)

		# f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		# f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
		# f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		# f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
		# f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		# f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
		# contact_sensors = f1_proximal_takktile_sensors + f1_distal_takktile_sensors + f2_proximal_takktile_sensors + f2_distal_takktile_sensors + f3_proximal_takktile_sensors + f3_distal_takktile_sensors
		# sim.updateWorld()
		# xform = get_moving_base_xform(sim.controller(0).model())

		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil(sim.getTime()%self.counter)) + \
			  "\t\tTime: " + str(sim.getTime())

		rotate_angle = so3.rotation((-1,0,0),math.radians(90))

		#print the contact sensors... you can safely take this out if you don't want to use it
		# try:
		# 	f1_contact = [s.getMeasurements()[0] for s in f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f1_distal_takktile_sensors]
		# 	f2_contact = [s.getMeasurements()[0] for s in f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f2_distal_takktile_sensors]
		# 	f3_contact = [s.getMeasurements()[0] for s in f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f3_distal_takktile_sensors]
		# 	# print "Contact sensors"
		# 	# print "  finger 1:",[int(v) for v in f1_contact]
		# 	# print "  finger 2:",[int(v) for v in f2_contact]
		# 	# print "  finger 3:",[int(v) for v in f3_contact]
		# except:
		# 	pass
		
		
		if self.state == 'idle' :
			self.hand.setCommand([0.2, 0.2, 0.2, 0])
			rotate_base(controller,rotate_angle,0.1)
			if math.ceil(sim.getTime()%self.counter) > 1 and math.ceil(sim.getTime()%self.counter) <= 2:
				desired = se3.mul((so3.identity(),[0,0,0.05]),xform)
				send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
				self.mypos = [0,0,0.05]
				self.state = 'get_lidar_data'
		elif self.state == 'get_lidar_data':
			lidar_data = controller.sensor(33).getMeasurements()
			self.resize_data = lidar_data
			# self.resize_data = resize_data
			x_coordinate = [ i for i in range(len(self.resize_data)) ]
			if math.ceil(sim.getTime()%self.counter) > 4:
				x_coordinate = [ i for i in range(len(self.resize_data)) ]
				plt.plot(x_coordinate,self.resize_data)
				# # plt.show()
				plt.savefig("./PS_2_1.png")
				self.state = 'fetch'
		elif self.state == 'fetch':
			if math.ceil(sim.getTime()%self.counter) > 4:
				xPos = -self.resize_data[130]*math.sin(math.radians(40))
				yPos = self.resize_data[130]*math.cos(math.radians(40))
				print "\n"
				print "Destination xPos= ",self.mypos[0] - xPos," ,yPos= ",self.mypos[1] + yPos, " current= ", so3.identity()
				#self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])	
				desired = se3.mul((so3.identity(),[self.mypos[0] - xPos , 0.3,0.05]),xform)
				send_moving_base_xform_linear(controller,rotate_angle,desired[1],1.0)
				# self.mypos = [self.mypos[0] + xPos,0.3,0.05]

		# 	controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
		# 	hand.setCommand([0.2, 0.2, 0.2, 0])
		# 	print "Closing"

		# if sim.getTime() > 3 and sim.getTime() < 4:
			
		# 	desired = se3.mul((so3.identity(),[0,-0.3,0.1]),xform)
		# 	send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
		# 	print "Moving backward"

		#need to manually call the hand emulator
		self.hand.process({},self.dt)

def make(sim,hand,dt):
	#get references to the robot's sensors (not properly functioning in 0.6.x)
	return StateMachineController(sim,hand,dt)

