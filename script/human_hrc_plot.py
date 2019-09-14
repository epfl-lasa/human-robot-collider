import pybullet as p
from walk.walker import Man
import numpy as np
import scipy.io as sio
import time

res = sio.loadmat("qolo_contact_points_case_4_with_velocities.mat")
global_scaling = res.result[0, 19]

physics_client_id = p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
m = Man(physics_client_id, partitioned = True, scaling = global_scaling)
m.setGaitPhase(0.7)
m.setColorForPartitionedCase4()
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
time.sleep(2000)
p.disconnect()
return
for i in range(np.size(res.result,0)):
	link_index = res.result[i, 1]
	cp_local = res.result[i, 20:23]