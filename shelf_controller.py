from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
import matplotlib.pyplot as plt

#additinal imported libraries
import math

def make(sim,hand,dt):
	#get references to the robot's sensors (not properly functioning in 0.6.x)
	f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
	contact_sensors = f1_proximal_takktile_sensors + f1_distal_takktile_sensors + f2_proximal_takktile_sensors + f2_distal_takktile_sensors + f3_proximal_takktile_sensors + f3_distal_takktile_sensors
	sim.updateWorld()
	xform = get_moving_base_xform(sim.controller(0).model())

	def controlfunc(controller):
		"""Place your code here... for a more sophisticated controller you could also create a class where the control loop goes in the __call__ method."""
		rotate_angle = so3.rotation((-1,0,0),math.radians(90))
		#print the contact sensors... you can safely take this out if you don't want to use it
		try:
			f1_contact = [s.getMeasurements()[0] for s in f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f1_distal_takktile_sensors]
			f2_contact = [s.getMeasurements()[0] for s in f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f2_distal_takktile_sensors]
			f3_contact = [s.getMeasurements()[0] for s in f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f3_distal_takktile_sensors]
			# print "Contact sensors"
			# print "  finger 1:",[int(v) for v in f1_contact]
			# print "  finger 2:",[int(v) for v in f2_contact]
			# print "  finger 3:",[int(v) for v in f3_contact]
		except:
			pass
		if sim.getTime() < 0.05 :
			hand.setCommand([0.2, 0.2, 0.2, 0])
			#hand.setCommand([math.radians(25), math.radians(25), math.radians(25), 0])
		
		if sim.getTime() > 0.05 and sim.getTime() < 1:
			#Setting the hand agle to face the shelf
			rotate_base(controller,rotate_angle,0.1)

		if sim.getTime() > 1 and sim.getTime() < 2:
			#the controller sends a command to the base after 1 s to lift the object
			desired = se3.mul((so3.identity(),[0,-0.2,-0.4]),xform)
			send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
			print "Moving forward"


		if sim.getTime() > 2:
			#getting lidar data
			lidar_data = controller.sensor(33).getMeasurements()

			if sim.getTime() > 4 :
				x_coordinate = [ i for i in range(len(lidar_data)) ]
				plt.plot(x_coordinate,lidar_data)
				plt.show()

		# 	controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
		# 	hand.setCommand([0.2, 0.2, 0.2, 0])
		# 	print "Closing"

		# if sim.getTime() > 3 and sim.getTime() < 4:
			
		# 	desired = se3.mul((so3.identity(),[0,-0.3,0.1]),xform)
		# 	send_moving_base_xform_linear(controller,rotate_angle,desired[1],0.5)
		# 	print "Moving backward"

		#need to manually call the hand emulator
		hand.process({},dt)
	return controlfunc

