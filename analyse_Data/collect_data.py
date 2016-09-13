import numpy as np
import ast

file = open('/home/kv/Desktop/lidar_data.txt', 'r')
empty = file.readline()

list_vals = np.zeros((1, 180))
line = True

while line:
	new_line = file.readline()
	if new_line == '':
		print 'done!'
		line = False
	else:
		list = ast.literal_eval(new_line)
		ndlist = np.array(list)
		list_vals = np.vstack((list_vals, ndlist))
		
np.savez('/home/kv/Desktop/lidar.npz',dist=list_vals)
print 'saved!'
