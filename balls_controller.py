from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import time
import math

c_hand=0.33
o_hand=0.4
pre_hand=0.7
close_hand=[c_hand,c_hand,c_hand,pre_hand]
open_hand=[o_hand,o_hand,o_hand+0.1,pre_hand]
move_speed=0.5;
face_down=[1,0,0,0,-1,0,0,0,-1]
start_pos=(face_down,[0,0,0.5])
drop_pos=(face_down,[0.65,0,0.6])


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
		self.target=(so3.identity(),[0,0,0])
		self.num_ball=sim.world.numRigidObjects()
		self.current_target=0
		self.waiting_list=range(self.num_ball)
		self.score=0
		self.print_flag=1
		self.everything_done = False

	def __call__(self,controller):
		sim = self.sim
		sim.updateWorld()
		# xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = False
		ReflexController.__call__(self,controller)
		time=sim.getTime();
		current_pos=get_moving_base_xform(self.sim.world.robot(0))
		#controller state machine
		if self.print_flag==1:
			print "State:",self.state
			self.print_flag=0
		if self.state == 'idle':
			self.go_to(controller,current_pos,start_pos)
			self.open_hand()
			#if 2 sec of time has passed and there are balls then:
			if time>self.last_state_end_t+1 and len(self.waiting_list)>0:
				self.state='find_target'
				self.last_state_end_t=time
				self.print_flag=1
				#if there are no balls in the basket then finish the job
			elif len(self.waiting_list) < 1 and self.everything_done == False:
				print "Finished! Total score is",self.score,"/",self.num_ball
				self.everything_done = True

		elif self.state=='find_target':
			#self.target is the postion co-ordinates of the ball along with angular measures
			self.target=self.find_target();
			#give 0.5 sec to locate the target of the ball
			if time>self.last_state_end_t+0.5:
				self.state='pick_target'
				self.last_state_end_t=time
				self.print_flag=1

		elif self.state == 'pick_target':
			if time<self.last_state_end_t+1:
				#we move in the plane of the gripper and catch the ball
				self.go_to(controller,current_pos,(self.target[0],vectorops.add(self.target[1],[0,0,0.2])))
			elif time<self.last_state_end_t+1.5:
				#move down along z axis
				self.go_to(controller,current_pos,self.target)
			elif time<self.last_state_end_t+2:
				#this is needed to stop at the current position in case there's some residual velocity
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				self.close_hand()
			else:
				#having picked the ball, raise up
				self.state='closing'
				self.last_state_end_t=time
				self.print_flag=1
		elif self.state == 'closing':
			if self.contact_gripper(sim, controller) == True or time> self.last_state_end_t+0.5:
				#having picked the ball, raise up
				self.state='raising'
				self.last_state_end_t=time
				self.print_flag=1
		elif self.state=='raising':
			raise_pos=(current_pos[0],[current_pos[1][0],current_pos[1][1],0.6])
			if time<self.last_state_end_t+1 :
				#raise along the z-co-ordinate of the picked ball
				self.go_to(controller,current_pos,raise_pos)
			else:
				"""
				check if the ball is still held by the gripper
				"""
				if self.contact_gripper(sim, controller):
					print 'Ball is in contact with gripper!'
					self.state='move_to_drop_position'
					self.last_state_end_t=time
					self.print_flag=1
				else:
					print 'Ball is not in contact with gripper'
					print 'Finding the target...'
					self.state = 'idle'
					self.last_state_end_t = time

		elif self.state == 'move_to_drop_position':
			if time<self.last_state_end_t+1.7 :
				self.go_to(controller,current_pos,drop_pos)
			else:
				self.state='drop'
				self.last_state_end_t=time
				self.print_flag=1

		elif self.state=='drop':
			if time<self.last_state_end_t+0.5:
				self.open_hand()
			else:
				self.state='idle'
				self.last_state_end_t=time
				self.print_flag=1
				self.check_target()

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
			p=self.sim.world.rigidObject(i).getTransform()[1]
			lob.append(( i, math.sqrt((p[0]*p[0]) + (p[1]*p[1]) + (p[2]*p[2]))))
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
				break

		d_x=0
		if best_p[0]>0.15:
			d_x =- 0.030
			# print 'too close to the wall!!'
		elif best_p[0]<-0.15:
			d_x = +0.025
			# print 'too close to the wall!!'
		target=(face_down,vectorops.add(best_p,[d_x,0,0.16]))
		return target

	def go_to(self,controller,current_pos,goal_pos):
		#t is the time calculated for smooth transition
		t=vectorops.distance(current_pos[1],goal_pos[1])/move_speed
		#goal_pos[1] is the co-ordinate of the ball 
		#goal_pos[0] is the angle specification of the ball
		send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);

	def close_hand(self):
		self.hand.setCommand(close_hand)

	def open_hand(self):
		self.hand.setCommand(open_hand)

	def check_target(self):
		#this value gives the current co-ordinates of the ball
		#p is list of 3 co-ordinates of mentioned ball number
		p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		if p[0]>0.25 or p[0]<-0.25 or p[1]>0.25 or p[1]<-0.25:
			self.waiting_list.remove(self.current_target)
		if p[0]<0.95 and p[0]>0.45 and p[1]<0.25 and p[1]>-0.25:
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
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	return StateMachineController(sim,hand,dt)