from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import matplotlib.pyplot as plt

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
		self.counter = 8
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
		prevPos = [0,0,0.0] 
		nextPos = [0,0,0.0]

		"""
		decide the position of the ball
		"""
		print 'ball count: ', self.ball_count
		if self.ball_count == 0:
			x = -0.1950
			y = -0.1666666665+self.dy1
		elif self.ball_count == 1:
			x = -0.0625
			y = -0.1666666665+self.dy1
		elif self.ball_count == 2:
			x = 0.0625
			y = -0.1666666665+self.dy1
		elif self.ball_count == 3:
			x = +0.1700
			y = -0.1666666665+self.dy1
		elif self.ball_count == 4:
			x = -0.1950
			y = 4.999999858590343e-10
		elif self.ball_count == 5:
			x = -0.0625
			y = 4.999999858590343e-10
		elif self.ball_count == 6:
			x = 0.0625
			y = 4.999999858590343e-10
		elif self.ball_count == 7:
			x = +0.1700
			y = 4.999999858590343e-10
		elif self.ball_count == 8:
			x = -0.1950
			y = 0.16666666749999998+self.dy2
		elif self.ball_count == 9:
			x = -0.0625
			y = 0.16666666749999998+self.dy2
		elif self.ball_count == 10:
			x = 0.0625
			y = 0.16666666749999998+self.dy2
		elif self.ball_count == 11:
			x = +0.1875
			y = 0.16666666749999998+self.dy2
		else:
			x = +0.1700
			y = 0.16666666749999998+self.dy2
			print 'done!'



		"""
		navigate and pick the ball
		"""
		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil(sim.getTime()%self.counter)) + \
				"\t\tTime: " + str(sim.getTime())
		lidar_data = controller.sensor(33).getMeasurements()
		print controller.sensor(33).getMeasurements(), len(controller.sensor(33).getMeasurements())
		print "|Exiting"
		# exit()
		if self.state == 'idle':
			lidar_data = controller.sensor(33).getMeasurements()
			x_coordinate = [ i for i in range(len(lidar_data)) ]
			# print "Going above the desired point"
			desired = se3.mul((so3.identity(),[0, 0, 0.1]), xform)
			send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)

			# if math.ceil(sim.getTime()%self.counter) > 2 and math.ceil(sim.getTime()%self.counter) <= 4:
				# rotate_angle = so3.rotation((-1,0,0),math.radians(90))
				# rotate_base(controller,rotate_angle,0.1)

			if math.ceil(sim.getTime()%self.counter) > 4 and math.ceil(sim.getTime()%self.counter) <= 6:
				self.state = 'lowering'

		elif self.state == 'lowering':
			x_coordinate = [ i for i in range(len(lidar_data)) ]
			plt.plot(x_coordinate,lidar_data)
			plt.show()
			path = "./" + str(2.0) + ".png"
			plt.savefig("./z=2_with_rotation.png")


		
def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	#print 'make is called...'
	return StateMachineController(sim,hand,dt)