import pybullet as p
from walk.walker import Man
import numpy as np
import scipy.io as sio
import time

#res = sio.loadmat("qolo_contact_points_case_4_with_velocities.mat")
global_scaling =1.0# res.result[0, 19]

physics_client_id = p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
m = Man(physics_client_id, partitioned = True, scaling = global_scaling)
m.setGaitPhase(0.45)
m.setColorForPartitionedCase4()

mc = Man(physics_client_id, partitioned = True, scaling =1/1.75)
mc.resetGlobalTransformation([0.7,0.35,0.94/1.75-0.94],[0,0,0])
mc.setGaitPhase(0.45)
mc.setColorForPartitionedCase4()

qolo_id = p.loadURDF('../data/qolo_and_user_rotated.urdf')
p.resetBasePositionAndOrientation(qolo_id, [1.4,0.7,-0.94],
	p.getQuaternionFromEuler([0,0,np.pi]))

colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [50,50,50])
box_id = p.createMultiBody(0, colBoxId, -1, [0, 0, -50-0.94],
	p.getQuaternionFromEuler([0,0,np.arctan2(0.35,0.7)]))
shape_data = p.getVisualShapeData(box_id)
p.changeVisualShape(box_id, shape_data[0][1], 
					rgbaColor=[0.7,0.7,0.7, 1])

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(1.5,(np.arctan2(0.35,0.7))/np.pi/2.0*360+180,0,[0.7,0.35,0])


time.sleep(2000)
p.disconnect()
quit()#return
for i in range(np.size(res.result,0)):
	link_index = res.result[i, 1]
	cp_local = res.result[i, 20:23]
