from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import time as T
import math
import numpy as np

# c_hand = 0.25 # for 5cm ball
c_hand_10 = 0.38
c_hand_inter = 0.275
c_hand_5 = 0.22
o_hand=0.4
pre_hand=0.7
close_hand_10 = [c_hand_10,c_hand_10,c_hand_10,pre_hand]
close_hand_inter = [c_hand_inter, c_hand_inter, c_hand_inter, pre_hand]
close_hand_5 = [c_hand_5-0.05, c_hand_5-0.05, c_hand_5-0.05, pre_hand]
open_hand=[o_hand,o_hand,o_hand+0.1,pre_hand]
move_speed=0.5;
face_down=[1,0,0, 0,1,0, 0,0,1]
# start_pos=(face_down, [0, 0, 0.6])
# drop_pos=(face_down, [0.65, 0, 0.6])



class StateMachineController(ReflexController):
	"""A more sophisticated controller that uses a state machine."""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.robot=sim.world.robot(0)
		self.dt = dt
		self.sim.updateWorld()
		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
		self.state = 'idle'
		self.last_state_end_t=sim.getTime()
		self.face_down=[1,0,0, 0,-1,0, 0,0,-1]
		self.target=(self.face_down, [0,0,0])
		self.num_ball=sim.world.numRigidObjects()
		self.current_target=0
		self.waiting_list=range(self.num_ball)
		self.default_list_counter=np.zeros(self.num_ball)
		self.ntries=6
		self.score=0
		self.print_flag=1
		self.everything_done = False
		self.init_time = sim.getTime()
		self.init_time2 = T.time()
		self.inital_flag = True
		self.rotate_angle = so3.identity()
		self.flag = True


	def __call__(self,controller):
		sim = self.sim
		sim.updateWorld()
		xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = False
		ReflexController.__call__(self,controller)
		time=sim.getTime();
		current_pos=get_moving_base_xform(self.sim.world.robot(0))
		#controller state machine
		if self.print_flag==1:
			print "State:",self.state
			self.print_flag=0

		self.update_waiting_list()

		# start_pos=(self.face_down, [0, 0, 0.6])
		# self.go_to(controller,current_pos,start_pos)
		if self.flag == True:
			desired = se3.mul((self.rotate_angle,[0, 0, 0.1]), xform)
			print 'xform[0]: \n', xform[0]
			print 'desired[0]: \n', desired[0]
			send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)		
			self.flag = False

		if self.state == 'idle':
			if time > self.last_state_end_t + 1.5 and len(self.waiting_list) > 0:
				desired = se3.mul((self.rotate_angle,[0, 0, 0.1]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)		
				self.open_hand()
				self.last_state_end_t = time
				self.print_flag = 1
				self.state = 'find_target'
			elif len(self.waiting_list) < 1 and self.everything_done == False:
				print "Finished! Total score is",self.score,"/",self.num_ball
				print 'Total time taken: ', sim.getTime() - self.init_time
				print 'total actual time: ', T.time() - self.init_time2
				self.everything_done = True				

		elif self.state=='find_target':
			if time > self.last_state_end_t + 0.5:
				#self.target is the postion co-ordinates of the ball along with angular measures
				#give 0.5 sec to locate the target of the ball
				self.target=self.find_target()
				self.state='pick_target'
				self.last_state_end_t=time
				self.print_flag=1

		elif self.state == 'pick_target':
			# self.go_to(controller,current_pos,(self.target[0], vectorops.add(self.target[1],[0, 0, 0.2])))
			desired = se3.mul((self.target[0],[self.target[1][0], self.target[1][1], 0.1]), xform)
			send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
			self.last_state_end_t = time
			self.state = 'go_down_along_z'
			self.print_flag = 1

		elif self.state == 'go_down_along_z':
			if time > self.last_state_end_t + 1:
				# self.target[]
				# self.go_to(controller,current_pos, self.target)			
				desired = se3.mul((self.target[0],[self.target[1][0], self.target[1][1], self.target[1][2]]), xform)
				send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
				self.state = 'close'
				self.last_state_end_t = time
				self.print_flag = 1

		elif self.state == 'close':
			if time > self.last_state_end_t + 1:
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				self.hand.setCommand(close_hand_10)
				self.state='closing'
				self.last_state_end_t=time
				self.print_flag=1

		# elif self.state == 'checking':
		# 	if time > self.last_state_end_t + 1:
		# 		contact_se = self.contact_gripper(sim, controller)
		# 		print 'Contact Values: ', self.hand.getCommand(), '\tIn contact? ', contact_se 
		# 		if contact_se:
		# 			if time > self.last_state_end_t + 1.5:	
		# 				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
		# 				self.hand.setCommand(self.hand.getCommand())
		# 				print 'Inner condition...'
		# 				self.state='closing'
		# 				self.last_state_end_t=time
		# 				self.print_flag=1
		# 		elif time > self.last_state_end_t + 2:
		# 			print 'Outer condition...'
		# 			self.state='closing'
		# 			self.last_state_end_t=time
		# 			self.print_flag=1


		elif self.state == 'closing':
			if time > self.last_state_end_t + 0.5:
				self.state='contact_1'
				self.last_state_end_t=time
				self.print_flag=1

		elif self.state == 'contact_1':
			if self.contact_gripper(sim, controller, 10):
				print 'Ball is of radius 10!'
				self.state = 'verify_contact_1'
				self.last_state_end_t = time
				self.print_flag = 1
			else:
				print 'Ball is not of radius 10'
				self.state = 'contact_2'
				self.last_state_end_t = time
				self.print_flag = 1

		elif self.state == 'verify_contact_1':
			if time > self.last_state_end_t + 0.3:
				if self.contact_gripper(sim, controller, 10):
					print 'Ball is still in contact with the gripper...'
					desired = se3.mul((self.target[0],[self.target[1][0], self.target[1][1], 0.1]), xform)
					send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
					self.state = 'raising'
					self.last_state_end_t = time
					self.print_flag = 1
				else:
					self.state = 'contact_2'
					self.last_state_end_t = time
					self.print_flag = 1

		elif self.state == 'contact_2':
			self.hand.setCommand(close_hand_inter)
			self.state = 'verify_contact_2'
			self.last_state_end_t = time
			self.print_flag = 1

		elif self.state == 'verify_contact_2':
			if time > self.last_state_end_t + 0.3:
				if self.contact_gripper(sim, controller, 7):
					print 'Ball is in contact with the gripper...'
					self.state = 'raising'
					desired = se3.mul((self.target[0],[self.target[1][0], self.target[1][1], 0.1]), xform)
					send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
					self.last_state_end_t = time
					self.print_flag = 1
				else:
					self.state = 'contact_3'
					self.last_state_end_t = time
					self.print_flag = 1

		elif self.state == 'contact_3':
			self.hand.setCommand(close_hand_5)
			self.state = 'verify_contact_3'
			self.last_state_end_t = time
			self.print_flag = 1

		elif self.state == 'verify_contact_3':
			if time > self.last_state_end_t + 0.3:
				if self.contact_gripper(sim, controller, 5):
					print 'Ball is in contact with the gripper...'
					self.state = 'raising'
					desired = se3.mul((self.target[0],[self.target[1][0], self.target[1][1], 0.1]), xform)
					send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)
					self.last_state_end_t = time
					self.print_flag = 1
				else:
					print 'Ball is too small to be held...'
					desired = se3.mul((self.target[0],[self.target[1][0], self.target[1][1], 0.1]), xform)
					send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)					
					self.state = 'raising'
					self.last_state_end_t = time
					self.print_flag = 1

		elif self.state=='raising':
			if time > self.last_state_end_t+1:
				if self.contact_gripper(sim, controller, 0):
					print 'Ball is in contact with gripper!'
					self.state='move_to_drop_position'
					self.last_state_end_t=time
					self.print_flag=1

					desired = se3.mul((self.target[0],[0.65, 0, 0.1]), xform)
					send_moving_base_xform_linear(controller,desired[0],desired[1], 0.5)

				else:
					print 'Ball is not in contact with gripper'
					print 'Finding the target...'
					self.state = 'idle'
					self.last_state_end_t = time

		elif self.state == 'move_to_drop_position':
			if time > self.last_state_end_t + 1:
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				self.hand.setCommand(open_hand)
				self.state='drop'				
				self.last_state_end_t=time
				self.print_flag=1

		elif self.state=='drop':
			if time > self.last_state_end_t+0.3:
				self.state='idle'
				self.last_state_end_t=time
				self.print_flag=1
				self.check_target()


		self.hand.process({},self.dt)

	def at_destination(self,current_pos,goal_pos):
		if se3.distance(current_pos,goal_pos)<0.05:
			return True
		else:
			return False

	def find_target(self):
		lob = list()
		# self.current_target=self.waiting_list[0]
		# best_p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		for i in self.waiting_list:
			if self.default_list_counter[i] < self.ntries:
				p=self.sim.world.rigidObject(i).getTransform()[1]
				lob.append(( i, math.sqrt((p[0]*p[0]) + (p[1]*p[1]) + (p[2]*p[2]))))
		if len(lob)==0:
			tmp = np.argmin(self.default_list_counter)
			p=self.sim.world.rigidObject(tmp).getTransform()[1]
			lob.append(( tmp, math.sqrt((p[0]*p[0]) + (p[1]*p[1]) + (p[2]*p[2]))))
		lob.sort(key=lambda x: x[1])
		for bestlob in lob:
			bestlobp = self.sim.world.rigidObject(bestlob[0]).getTransform()[1]
			upflag = 0
			for i in self.waiting_list:
				if i == bestlob[0]:
					continue
				p=self.sim.world.rigidObject(i).getTransform()[1]
				if math.sqrt(pow(p[0]-bestlobp[0],2) + pow(p[1]-bestlobp[1],2)) < 0.11 and p[2] > bestlobp[2]:
					upflag = 1
					break
			if upflag == 0:
				self.current_target = bestlob[0]
				best_p = bestlobp
				print "best lob", best_p
				break

		self.default_list_counter[self.current_target] = self.default_list_counter[self.current_target]+1

		#edge conditions...rotate about z-axis by 90 degrees
		d_x = 0
		d_y = 0
		# best_p[0] = 0.25 - 0.064
		# best_p[1] = 0.25  - 0.055
		target=(self.rotate_angle, vectorops.add(best_p, [d_x, d_y, -0.342]))
	
		if best_p[0] <= (-0.25 + 0.095) or best_p[0] > (+0.25 - 0.065):
			face_down = so3.rotation((0, 0, -1), math.radians(90))
			target=(face_down, vectorops.add(best_p, [d_x, d_y, -0.342]))

			if target[1][1] <= (-0.25 + 0.065):
				target[1][1] = -0.25 + 0.065
			elif target[1][1] > (+0.25 - 0.095):
				target[1][1] = 0.25 - 0.095
			if target[1][0] >= (0.25 - 0.055):
				target[1][0] = 0.25 - 0.055
			elif target[1][0] <= (-0.25 + 0.055):
				target[1][0] = -0.25 + 0.055

		elif best_p[1] >= (0.25 - 0.055):
			best_p[1] = 0.25 - 0.055
			target[1][1] = 0.25 - 0.055

		elif best_p[1] <= (-0.25 + 0.055):
			best_p[1] = -0.25 + 0.055 
			target[1][1] = -0.25 + 0.055

		return target

	def go_to(self,controller,current_pos,goal_pos):
		#t is the time calculated for smooth transition
		# t=vectorops.distance(current_pos[1],goal_pos[1])/move_speed
		#goal_pos[1] is the co-ordinate of the ball 
		#goal_pos[0] is the angle specification of the ball
		# print 'Angle Specs: ', goal_pos[0]
		send_moving_base_xform_linear(controller, self.rotate_angle,goal_pos[1], 1.0);

	def close_hand(self):
		self.hand.setCommand(close_hand)

	def open_hand(self):
		self.hand.setCommand(open_hand)

	def update_waiting_list(self):
		# print "Updating Waiting List", len(self.waiting_list)
		for i in self.waiting_list:
			p = self.sim.world.rigidObject(i).getTransform()[1]
			if p[0]>0.25 or p[0]<-0.25 or p[1]>0.25 or p[1]<-0.25:
				print "Removing an element from waiting list"
				self.waiting_list.remove(i)
				self.default_list_counter[i] = 100
		# print "Final Length..", len(self.waiting_list)

	def check_target(self):
		#this value gives the current co-ordinates of the ball
		#p is list of 3 co-ordinates of mentioned ball number
		p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		if p[0]<0.95 and p[0]>0.45 and p[1]<0.25 and p[1]>-0.25:
			self.score=self.score+1
			print 'Ball #', self.current_target, ' is placed in the target box!\n\n'
		else:
			print 'Ball #', self.current_target, ' is not placed in the target box\n\n'
		print 'Going over to next cycle...'

	def contact_gripper(self, sim, controller, radius):
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
		# if f1_contact[5] == 1 or f2_contact[5] == 1 or f3_contact[5] == 1:
		# 	return True
		# elif f1_contact[4] == 1 or f2_contact[4] == 1 or f3_contact[4] == 1:
		# 	return True
		# else:
		# 	return False

		if radius == 10:
			if (f1_contact[5] == 1 and f2_contact[5] == 1 and f3_contact[5] == 1)  or (f1_contact[4] == 1 or f2_contact[4] == 1 or f3_contact[4] == 1):
				return True
			else:
				return False

		elif radius == 7 or radius == 5 or radius == 0:
			if f1_contact[5] == 1 or f2_contact[5] == 1 or f3_contact[5] == 1:
				return True
			elif f1_contact[4] == 1 or f2_contact[4] == 1 or f3_contact[4] == 1:
				return True
			else:
				return False


		
def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	return StateMachineController(sim,hand,dt)