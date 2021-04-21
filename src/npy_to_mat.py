import numpy as np
import scipy.io
test_set = np.array([1, 4, None, None, 5, 8, None])
a = np.load('/home/denis/human-robot-collider-devel/src/controlled_collision.npy', allow_pickle = True)
b = []
for index in range(len(a)):
    if np.size(a[index])>1:
        b.append(a[index])
#c = np.load('C:/Semester_Project_HRCA_python/controlled_collision(2).npy', allow_pickle = True)
#d = []
#for index in range(len(c)):
#    if np.size(c[index])>1:
 #       d.append(c[index])
scipy.io.savemat('result_phase_1.mat', {'mydata': b})
from matplotlib import pyplot as plt
fig, (ax1, ax2) = plt.subplots(1, 2)
fig.suptitle('Horizontally stacked subplots')
ax1.plot(b[4])
ax2.plot(d[0])
#plt.plot(b[0])
#plt.subplot(d[0])
