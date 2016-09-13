from matplotlib import pyplot as plt
import numpy as np
import time

data = np.load('/home/kv/Desktop/lidar.npz')
raw_data = data['dist']

print 'raw_data shape: ', raw_data.shape

m, n = raw_data.shape
Xp = []
Yp = []
plt.axis([0, 200, 0, 60])
plt.ion()
plt.show()

for j in range(n):
	Xp.append(j)

for i in range(1, m):
	for j in range(n):
		Yp.append(raw_data[i][j])

	plt.plot(Xp, Yp)
	plt.draw()
	plt.pause(0.3)
	print '# of plot: ', i
	#time.sleep(10)
	Yp = []
