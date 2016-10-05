from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal 
#additinal imported libraries
import math

c_hand=0.3
o_hand=0.2
pre_hand=0.2
shelf_height = 0.3
close_hand=[c_hand,c_hand,c_hand,pre_hand]

def minimum(a):
	min = 4.0
	for i in range (70,100):
		if a[i] < min :
			min = a[i]
			index = i 

	return index,a[index]

class StateMachineController(ReflexController):
	"""A more sophisticated controller that uses a state machine."""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.dt = dt
		self.sim.updateWorld()
		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
		self.state = 'idle'
		self.counter = 10
		self.num_objects=sim.world.numRigidObjects()
		self.waiting_list=range(self.num_objects)
		self.default_list = range(self.num_objects)
		self.target=[0,0,0]
		self.current_target=0
		self.score=0
		self.everything_done = False
		self.offset = 0
		self.lidar_height = shelf_height/3. 
		self.resize_data = np.zeros(180)

	def __call__(self,controller):

		sim = self.sim
		sim.updateWorld()
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = True
		ReflexController.__call__(self,controller)

		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil((sim.getTime()+self.offset)%self.counter)) + \
			  "\t\tTime: " + str((sim.getTime()+self.offset))

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
			print len(self.waiting_list)
			if len(self.waiting_list) < 1 and self.everything_done == False:
				print "Finished! Total score is",self.score,"/",self.num_objects
				self.everything_done = True

			elif len(self.waiting_list)>0:
				self.hand.setCommand([math.radians(200), math.radians(200), math.radians(200), 0])
				rotate_base(controller,rotate_angle,0.1)
				if math.ceil((sim.getTime()+self.offset)%self.counter) > 1 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 2:
					desired = se3.mul((so3.identity(),[0.0,0.0,self.lidar_height]),xform)
					send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
					self.state = 'set_pos'

		elif self.state == 'set_pos':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 3 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 4:
				lidar_data = controller.sensor(33).getMeasurements()
				self.resize_data = lidar_data[290:360] + lidar_data[0:290]

			if math.ceil((sim.getTime()+self.offset)%self.counter) > 4 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 5:
				self.hand.setCommand([math.radians(25), math.radians(25), math.radians(25), 0])
				rotate_base(controller,rotate_angle,0.1)

			if math.ceil((sim.getTime()+self.offset)%self.counter) > 5 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 7:
				self.plot_data = self.resize_data[169:200]	
				x_coordinate = [ i for i in range(len(self.plot_data))]
				filtered_data = signal.savgol_filter(self.plot_data, 11, 3)
				self.plot_data = filtered_data				
				# plt.plot(x_coordinate,self.plot_data,'g')
				# plt.savefig("./old.png")
				# plt.show()
				# plt.cla()
				# plt.clf()

				local_min = filtered_data.max()
				local_min_idx = -1
				for idx in range(0, len(filtered_data), 3):
					if filtered_data[idx] < local_min :
						local_min = filtered_data[idx]
						local_min_idx = idx

				angle = (163 + local_min_idx)/2.0 
				depth = self.resize_data[169 + local_min_idx]
				dx = depth * math.cos(math.radians(angle))
				dy = depth * math.sin(math.radians(angle))
				print depth 
				print angle
				print dy 
				print local_min_idx
				if local_min_idx != -1 and dy < 0.65:
					print 'detected local min is at angle: ', (169 + local_min_idx)/2.0 
 
					print 'Angle: ', angle, ' Depth: ', depth
					print "\n New Pos X: ", dx, " Y: ", dy
					self.target = self.find_nearest(dx)
					desired = se3.mul((so3.identity(),[self.target[0] , self.target[1] - 0.15 , 0.06]),xform)
					send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
					self.state = 'fetch'
				else:
					print 'no local minima detected...'
					if self.lidar_height == shelf_height/3.  :
						self.lidar_height = 0.07
						self.state = "idle"
						self.offset = -math.floor(sim.getTime())

					elif self.lidar_height == 0.07 :
						self.target = self.find_target()
						desired = se3.mul((so3.identity(),[self.target[0] , self.target[1] - 0.15 , 0.06]),xform)
						send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
						self.state = 'fetch'

						#Mantri waala part use hoga yahan
					

		elif self.state == 'fetch':
			print self.num_objects
			print self.target
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 7 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 8:
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				self.hand.setCommand(close_hand)
				#self.hand.setCommand([0.2, 0.2, 0.2, 0.5])
				self.state = 'grasp'
		elif self.state == 'grasp':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 8 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 9:
				if self.contact_gripper(sim, controller):
					desired = se3.mul((so3.identity(),[0, -0.15, 0.07]), xform)
					send_moving_base_xform_linear(controller,rotate_angle,desired[1], 1.0)
					self.state = 'backward'
				else :
					print 'Ball is not in contact with gripper'
					self.state = 'idle'
					self.offset = -math.floor(sim.getTime())
		elif self.state =='backward':
			if math.ceil((sim.getTime()+self.offset)%self.counter) > 9 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 10:
				self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])
				self.state = 'release'
		elif self.state == 'release':
			#print " Time function : " + str(math.ceil((sim.getTime()+self.offset)%self.counter))
			if math.ceil((sim.getTime()+self.offset)%self.counter) == 1.0:
				self.state = 'idle'
				self.check_target()


		#need to manually call the hand emulator
		self.hand.process({},self.dt)

	def find_nearest(self , x_pos):
		object_id = 0 ;
		error = 10
		for i in self.waiting_list :
			p=self.sim.world.rigidObject(i).getTransform()[1]
			if math.fabs(p[0] - x_pos) < error :
				error = math.fabs(p[0] - x_pos)
				object_id = i 
		return self.sim.world.rigidObject(object_id).getTransform()[1]


	def find_target(self):
		self.current_target=self.waiting_list[0]
		best_p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		for i in self.waiting_list:
			p=self.sim.world.rigidObject(i).getTransform()[1]
			print " Index : " , i , p[0] , p[1] , p[2]
			if math.fabs(p[0]) < math.fabs(best_p[0]) :
				self.current_target=i
				best_p=p
		return best_p

	def check_target(self):
		#this value gives the current co-ordinates of the ball
		#p is list of 3 co-ordinates of mentioned ball number
		p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		if p[1] < 0.2 :
			self.waiting_list.remove(self.current_target)
		if p[0]<0.2 and p[0]>-0.25 and p[1]<0.2 and p[1]>-0.2:
			self.score=self.score+1
			print 'Ball #', self.current_target, ' is placed in the target box!\n\n'
		else:
			print 'Ball #', self.current_target, ' is not placed in the target box\n\n'
		print 'Going over to next cycle...'

	def contact_gripper(self, sim, controller):
		"""
		reading finger sensor data
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
		# print "Contact sensors"
		# print "  finger 1:",[int(v) for v in f1_contact]
		# print "  finger 2:",[int(v) for v in f2_contact]
		# print "  finger 3:",[int(v) for v in f3_contact]
		# # print '\n\n'

		"""
		check whether ball is held
		"""
		if f1_contact[5] == 1 or f2_contact[5] == 1 or f3_contact[5] == 1:
			return True
		elif f1_contact[4] == 1 or f2_contact[4] == 1 or f3_contact[4] == 1:
			return True
		else:
			return False

def make(sim,hand,dt):
	#get references to the robot's sensors (not properly functioning in 0.6.x)
	return StateMachineController(sim,hand,dt)


#---------------------------------------------------------------------------------------------------------------------------------------------------------------#
#........................................  Old Code ............................................................................................................#
#---------------------------------------------------------------------------------------------------------------------------------------------------------------#


# from klampt import *
# from klampt.math import vectorops,so3,se3
# from moving_base_control import *
# from reflex_control import *
# import matplotlib.pyplot as plt
# import numpy as np
# #additinal imported libraries
# import math

# def minimum(a):
# 	min = 4.0
# 	for i in range (70,100):
# 		if a[i] < min :
# 			min = a[i]
# 			index = i 

# 	return index,a[index]

# class StateMachineController(ReflexController):
# 	"""A more sophisticated controller that uses a state machine."""
# 	def __init__(self,sim,hand,dt):
# 		self.sim = sim
# 		self.hand = hand
# 		self.dt = dt
# 		self.sim.updateWorld()
# 		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
# 		self.state = 'idle'
# 		self.counter = 8
# 		self.ball_count = 0
# 		self.dx = 0.025
# 		self.dy1 = -0.0125
# 		self.dy2 = +0.0125
# 		self.mypos = [0,0,0]
# 		self.resize_data = np.zeros(180)

# 	def __call__(self,controller):

# 		sim = self.sim
# 		xform = self.base_xform
# 		#turn this to false to turn off print statements
# 		ReflexController.verbose = True
# 		ReflexController.__call__(self,controller)

# 		# f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
# 		# f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
# 		# f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
# 		# f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
# 		# f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
# 		# f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
# 		# contact_sensors = f1_proximal_takktile_sensors + f1_distal_takktile_sensors + f2_proximal_takktile_sensors + f2_distal_takktile_sensors + f3_proximal_takktile_sensors + f3_distal_takktile_sensors
# 		# sim.updateWorld()
# 		# xform = get_moving_base_xform(sim.controller(0).model())

# 		print "State: " + str(self.state) + "\t\tSplit: " + str(math.ceil((sim.getTime()+self.offset)%self.counter)) + \
# 			  "\t\tTime: " + str((sim.getTime()+self.offset))

# 		rotate_angle = so3.rotation((-1,0,0),math.radians(90))

# 		#print the contact sensors... you can safely take this out if you don't want to use it
# 		# try:
# 		# 	f1_contact = [s.getMeasurements()[0] for s in f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f1_distal_takktile_sensors]
# 		# 	f2_contact = [s.getMeasurements()[0] for s in f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f2_distal_takktile_sensors]
# 		# 	f3_contact = [s.getMeasurements()[0] for s in f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f3_distal_takktile_sensors]
# 		# 	# print "Contact sensors"
# 		# 	# print "  finger 1:",[int(v) for v in f1_contact]
# 		# 	# print "  finger 2:",[int(v) for v in f2_contact]
# 		# 	# print "  finger 3:",[int(v) for v in f3_contact]
# 		# except:
# 		# 	pass
		
		
# 		if self.state == 'idle' :
# 			self.hand.setCommand([math.radians(200), math.radians(200), math.radians(200), 0])
# 			rotate_base(controller,rotate_angle,0.1)
# 			if math.ceil((sim.getTime()+self.offset)%self.counter) > 1 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 2:
# 				desired = se3.mul((so3.identity(),[0.0,-0.2,0.15]),xform)
# 				send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
# 				self.newPosX = -0.1
# 				self.state = 'set_pos'
# 		elif self.state == 'set_pos':
# 			lidar_data = controller.sensor(33).getMeasurements()
# 			self.resize_data = lidar_data[290:360] + lidar_data[0:290]


# 			if math.ceil((sim.getTime()+self.offset)%self.counter) > 3:
# 				x_coordinate = [ i for i in range(len(self.resize_data)) ]
# 				plt.plot(x_coordinate,self.resize_data,'r')
# 				plt.savefig("./old.png")
# 				plt.show()

# 		# 		angle , depth = minimum(self.resize_data)
# 		# 		print "Angle , Depth ",angle,depth
# 		# 		plt.show()
# 		# 		dx = depth*math.cos(math.radians(angle))
# 		# 		dy = depth*math.sin(math.radians(angle))
# 		# 		print "\n New Pos :",-0.1+dx + 0.05
# 		# 		if angle <= 70 and angle > 95 :
# 		# 			desired = se3.mul((so3.identity(),[-0.1+dx,0.2,0.07]),xform)
# 		# 		else :	
# 		# 			desired = se3.mul((so3.identity(),[-0.1+dx + 0.05,0.2,0.07]),xform)
# 		# 		send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
# 		# 		self.newPosX = -0.1+dx+0.05
# 		# 		#Calculate the point and go to that point ahead of obstacle
# 		# 		self.state = 'get_lidar_data'
# 		# elif self.state == 'get_lidar_data':
# 		# 	if math.ceil((sim.getTime()+self.offset)%self.counter) > 4 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 5:
# 		# 		#self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])	
# 		# 		#Go Towards the object
# 		# 		desired = se3.mul((so3.identity(),[self.newPosX, 0.55,0.07]),xform)
# 		# 		send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
# 		# 		self.state = 'fetch'
# 		# elif self.state == 'fetch':
# 		# 	if math.ceil((sim.getTime()+self.offset)%self.counter) > 5 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 6:
# 		# 		controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
# 		# 		self.hand.setCommand([0.2, 0.2, 0.2, 0])
# 		# 		self.state = 'grasp'
# 		# elif self.state == 'grasp':
# 		# 	if math.ceil((sim.getTime()+self.offset)%self.counter) > 6 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 7:
# 		# 		desired = se3.mul((so3.identity(),[0, -0.15, 0.07]), xform)
# 		# 		send_moving_base_xform_linear(controller,rotate_angle,desired[1], 1.0)
# 		# 		self.state = 'backward'
# 		# elif self.state =='backward':
# 		# 	if math.ceil((sim.getTime()+self.offset)%self.counter) > 7 and math.ceil((sim.getTime()+self.offset)%self.counter) <= 8:
# 		# 		self.hand.setCommand([math.radians(30), math.radians(30), math.radians(30), 0])
# 		# 		self.state = 'release'
# 		# elif self.state == 'release':
# 		# 	#print " Time function : " + str(math.ceil((sim.getTime()+self.offset)%self.counter))
# 		# 	if math.ceil((sim.getTime()+self.offset)%self.counter) == 1.0:
# 		# 		self.state = 'idle'


# 		#need to manually call the hand emulator
# 		self.hand.process({},self.dt)

# def make(sim,hand,dt):
# 	#get references to the robot's sensors (not properly functioning in 0.6.x)
# 	return StateMachineController(sim,hand,dt)

