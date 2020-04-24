import pybullet as p
from walk.walker import Man
import numpy as np
import scipy.io as sio
import time

def rotate_by_quaternion(vec3, q):
	R = p.getMatrixFromQuaternion(q)
	result = [
		R[0]*vec3[0] + R[1]*vec3[1] + R[2]*vec3[2],
		R[3]*vec3[0] + R[4]*vec3[1] + R[5]*vec3[2],
		R[6]*vec3[0] + R[7]*vec3[1] + R[8]*vec3[2]]
	return result

def rotate_by_inverse_of_quaternion(vec3, q):
	R = p.getMatrixFromQuaternion(q)
	result = [
		R[0]*vec3[0] + R[3]*vec3[1] + R[6]*vec3[2],
		R[1]*vec3[0] + R[4]*vec3[1] + R[7]*vec3[2],
		R[2]*vec3[0] + R[5]*vec3[1] + R[8]*vec3[2]]
	return result

def getLinkOrBaseState(body_id, link_index):
	if link_index > -1:
		link_state = p.getLinkState(body_id, link_index,
			computeForwardKinematics = 1)
	else:
		base_position, base_orientation = p.getBasePositionAndOrientation(body_id)
		link_state = [None, None, None, None, base_position, base_orientation]
	return link_state

result_child = np.load("qolo_contact_points_case_4_with_velocities_child.npy")
phase_2_res_child = sio.loadmat("qolo_case_4_force_peaks_phase_2_child.mat")
result_adult = np.load("qolo_contact_points_case_4_with_velocities_adult.npy")
phase_2_res_adult = sio.loadmat("qolo_case_4_force_peaks_phase_2_adult.mat")
results = [result_child, result_adult]
phase_2_results = [phase_2_res_child, phase_2_res_adult]
x_positions = [-0.3, 0.3]

physics_client_id = p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
for i in [0,1]:
	result = results[i]
	phase_2_res = phase_2_results[i]

	color_coding = phase_2_res['color_coding']
	f_normalized = phase_2_res['F_normalize_per_iteration']
	global_scaling = result[0, 19]

	m = Man(physics_client_id, partitioned = True, scaling = global_scaling)
	m.setGaitPhase(0.45)
	m.resetGlobalTransformation([x_positions[i],0,0.94*m.scaling],[0,0,0])

	sdl = p.getVisualShapeData(m.body_id)
	for i in range(len(sdl)):
		p.changeVisualShape(m.body_id, sdl[i][1], 
			rgbaColor=[0.5, 0.5, 0.9, 1])
	#qolo_id = p.loadURDF('../data/qolo_and_user_rotated.urdf')
	#p.resetBasePositionAndOrientation(qolo_id, [1.4,0.7,-0.94],
	#	p.getQuaternionFromEuler([0,0,np.pi]))

	#colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [50,50,50])
	#box_id = p.createMultiBody(0, colBoxId, -1, [0, 0, -50-0.94],
	#	p.getQuaternionFromEuler([0,0,np.arctan2(0.35,0.7)]))
	#shape_data = p.getVisualShapeData(box_id)
	#p.changeVisualShape(box_id, shape_data[0][1], 
	#					rgbaColor=[0.7,0.7,0.7, 1])

	p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
	#p.resetDebugVisualizerCamera(1.5,(np.arctan2(0.35,0.7))/np.pi/2.0*360+180,0,[0.7,0.35,0])

	colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius = 0.005)
	#visSphereId = p.createVisualShape(p.GEOM_SPHERE, radius = 0.005)
	for i in range(np.size(result,0)):
		if f_normalized[i] < 0.001:
			continue

		link_index = int(result[i, 1])
		cp_local = result[i, 20:23]

		link_state = getLinkOrBaseState(m.body_id, link_index)
		cp_global = np.array(link_state[4]) + np.array(rotate_by_quaternion(cp_local, link_state[5]))
		visSphereId = p.createVisualShape(p.GEOM_SPHERE, radius = 0.009/1.75)#f_normalized[i]/4*0.015)
		sphere_id = p.createMultiBody(0, colSphereId, visSphereId, cp_global)
		sdl = p.getVisualShapeData(sphere_id)
		p.changeVisualShape(sphere_id, sdl[0][1], 
			rgbaColor=[color_coding[i,0], color_coding[i,1], color_coding[i,2], 1])
		#p.addUserDebugLine(cp_global,cp_global+np.ones([3])*0.0001,[1,0,0],0.2)

print ("Done.")

time.sleep(2000)
p.disconnect()