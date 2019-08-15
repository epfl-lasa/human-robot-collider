#PYHTON 3!

import xml.etree.ElementTree as ET
import numpy as np
import pybullet as p
import math
import time
import re
import matplotlib.pyplot as plt

# def applyMMMJointPositionsToURDF(joint_positions, urdf_body_id):
# 	#BT to chest_to_belly

# 	#pelvis_to_right_leg
# 	applyMMMRotationToURDFJoint(urdf_body_id, 3,
# 		joint_positions[14], joint_positions[15], joint_positions[16])

def generateQuaternionFromMMMRxRyRz(rx, ry, rz):
	q_intermediate = p.getQuaternionFromEuler([math.pi/2, 0, math.pi/2])
	t, q_intermediate_inv = p.invertTransform([0,0,0], q_intermediate)
	q_tmp = p.getQuaternionFromEuler([ry, rz, rx])
	t, q_tmp = p.multiplyTransforms([0,0,0], q_intermediate, [0,0,0], q_tmp)
	t, q = p.multiplyTransforms([0,0,0], q_tmp, [0,0,0], q_intermediate_inv)
	return q

# def generateQuaternionFromMMMGlobalRxRyRz(rx, ry, rz):
# 	q_intermediate = p.getQuaternionFromEuler([0, -math.pi/2, math.pi])
# 	t, q_intermediate_inv = p.invertTransform([0,0,0], q_intermediate)
# 	q_tmp = p.getQuaternionFromEuler([rz, -ry, rx])
# 	t, q_tmp = p.multiplyTransforms([0,0,0], q_intermediate, [0,0,0], q_tmp)
# 	t, q = p.multiplyTransforms([0,0,0], q_tmp, [0,0,0], q_intermediate_inv)
# 	return q

def applyMMMRotationToURDFJoint(urdf_body_id, joint_index, rx, ry, rz, inverse=False):
	q = generateQuaternionFromMMMRxRyRz(rx, ry, rz)
	quat_tf_urdf = p.getQuaternionFromEuler([-math.pi/2, math.pi, 0])
	translation, quat_tf_urdf_inv = p.invertTransform([0,0,0], quat_tf_urdf)
	t, q = p.multiplyTransforms([0,0,0], quat_tf_urdf, [0,0,0], q)
	t, q = p.multiplyTransforms([0,0,0], q, [0,0,0], quat_tf_urdf_inv)

	if inverse:
		t, q = p.invertTransform([0,0,0], q)

	p.resetJointStateMultiDof(urdf_body_id, joint_index, q)

	# quat_0 = p.getQuaternionFromEuler([rx,rz,-ry])

	# quat_tf = p.getQuaternionFromEuler([math.pi/2,0,0])
	# translation, quat_tf_inv = p.invertTransform([0,0,0], quat_tf)

	# quat_tf_urdf = p.getQuaternionFromEuler([-math.pi/2,0,-math.pi])
	# translation, quat_tf_urdf_inv = p.invertTransform([0,0,0], quat_tf_urdf)

	# translation, quat_tmp = p.multiplyTransforms([0,0,0], quat_tf_urdf,
	# 	[0,0,0], quat_tf)

	# translation, quat_tmp = p.multiplyTransforms([0,0,0], quat_tmp,
	# 	[0,0,0], quat_0)

	# translation, quat_tmp = p.multiplyTransforms([0,0,0], quat_tmp,
	# 	[0,0,0], quat_tf_inv)

	# translation, quat_final = p.multiplyTransforms([0,0,0], quat_tmp,
	# 	[0,0,0], quat_tf_urdf_inv)

	# if inverse:
	# 	translation, quat_final = p.invertTransform([0,0,0], quat_final)

	# p.resetJointStateMultiDof(urdf_body_id, joint_index, quat_final)

# call this function AFTER applying the BT- and BP-joint angles for the urdf (using the function above) 
def applyMMMRotationAndZeroTranslationToURDFBody(urdf_body_id, rx, ry, rz):
	# get the rotation from the (hypothetical) world to the pelvis
	tpcom, rpcom, tplcom, rplcom, tpf, rotation_pelvis_frame, v, omega = p.getLinkState(
		urdf_body_id, 1, True, True)

	# get the rotation from the (hypothetical) world to the chest
	t_com, r_com = p.getBasePositionAndOrientation(urdf_body_id)
	#m, lfr, lI, lIt, lIr, rest, rfr, sfr, cd, cs = p.getDynamicsInfo(bodyUniqueId,-1)
	#t_tmp, r_tmp = p.invertTransform([0,0,0], lIr)
	#t_tmp, rotation_chest_frame = p.multiplyTransforms([0,0,0], r_com, 
	#	[0,0,0], r_tmp)
	rotation_chest_frame = r_com

	# get the rotation from the pelvis to the chest
	t_tmp, r_tmp = p.invertTransform([0,0,0], rotation_pelvis_frame)
	t_tmp, r_pelvis_to_chest = p.multiplyTransforms([0,0,0], r_tmp, 
		[0,0,0], rotation_chest_frame)

	# pre-multiply with rotation from MMM to my axis convention
	t_tmp, r_mmm_pelvis_to_chest = p.multiplyTransforms([0,0,0],
		p.getQuaternionFromEuler([-math.pi/2, math.pi, 0]), 
		[0,0,0], r_pelvis_to_chest)

	# generate rotation from world to pelvis MMM frame
	#r_world_to_mmm_pelvis = generateQuaternionFromMMMRxRyRz(rx, ry, rz)
	# Alternative (Z-Y-X order of rotations, not like for joints):
	r_world_to_mmm_pelvis = p.getQuaternionFromEuler([rx, ry, rz])
	#r_world_to_mmm_pelvis = generateQuaternionFromMMMGlobalRxRyRz(rx, ry, rz)

	# compose rotation from world to my chest frame
	t_tmp, r_world_to_chest = p.multiplyTransforms([0,0,0], r_world_to_mmm_pelvis,
		[0,0,0], r_mmm_pelvis_to_chest)

	# apply it to the base together with a zero translation
	p.resetBasePositionAndOrientation(urdf_body_id,
		[0, 0, 0], r_world_to_chest)

def applyMMMTranslationToURDFBody(urdf_body_id, tx, ty, tz):
	# get the translation to the pelvis frame
	#tpcom, rpcom, tplcom, rplcom, translation_to_pelvis_frame, rpf, v, omega = p.getLinkState(
	#	urdf_body_id, 1, True, True)

	# get the translation to the left leg frame
	tllcom, rllcom, tlllcom, rlllcom, translation_to_left_leg_frame, rllf, vll, omegall = p.getLinkState(
		urdf_body_id, 3, True, True)

	# get the translation to the right leg frame
	trlcom, rrlcom, trllcom, rrllcom, translation_to_right_leg_frame, rrlf, vrl, omegarl = p.getLinkState(
		urdf_body_id, 2, True, True)

	t_base_com, r_base_com = p.getBasePositionAndOrientation(urdf_body_id)

	t_phb_center_to_base_com = [0,0,0]
	for i in range(3):
		t_phb_center_to_base_com[i] = t_base_com[i] - 0.5*(
			translation_to_right_leg_frame[i] + translation_to_left_leg_frame[i])

	t_base_com = [
		tx + t_phb_center_to_base_com[0],
		ty + t_phb_center_to_base_com[1],
		tz + t_phb_center_to_base_com[2]]

	p.resetBasePositionAndOrientation(urdf_body_id, t_base_com, r_base_com)


# load the human-adult model
physicsClient = p.connect(p.GUI)
human_adult_ID = p.loadURDF("../data/human_adult_scan.urdf",
	flags=p.URDF_MAINTAIN_LINK_ORDER)

#the following file is from https://motion-database.humanoids.kit.edu/details/motions/37/
tree = ET.parse('../data/WalkingStraightForwards01.xml')
root = tree.getroot()

shape_data_list = p.getVisualShapeData(human_adult_ID)
for sd in shape_data_list:
	pass#p.changeVisualShape(human_adult_ID, sd[1], rgbaColor=[0, 0, 1, 0.5])

#p.changeVisualShape(human_adult_ID, -1, rgbaColor=[0, 0, 1, 0.5])
p.resetDebugVisualizerCamera(cameraDistance=3, 
	cameraYaw=-90, cameraPitch=-25, cameraTargetPosition=[-2,0,0])

time.sleep(2.0)

joint_position_trajectories = np.zeros([len(root[0][3]), 44])
pose_trajectories = np.zeros([len(root[0][3]), 6])

initial_position_value_string_list = re.split(' ', root[0][3][0][1].text)
x0 = float(initial_position_value_string_list[0])/1000
y0 = float(initial_position_value_string_list[1])/1000
z0 = float(initial_position_value_string_list[2])/1000

for t in range(len(root[0][3])):

	#p.resetBasePositionAndOrientation(human_adult_ID,
	#	[0, 0, 0], p.getQuaternionFromEuler([0,0,0]))

	value_string_list = re.split(' ', root[0][3][t][3].text)

	# store the values for plotting
	for j in range(44):
		joint_position_trajectories[t, j] = float(value_string_list[j])

	position_value_string_list = re.split(' ', root[0][3][t][1].text)
	rotation_value_string_list = re.split(' ', root[0][3][t][2].text)

	pose_trajectories[t, 0] = float(position_value_string_list[0])
	pose_trajectories[t, 1] = float(position_value_string_list[1])
	pose_trajectories[t, 2] = float(position_value_string_list[2])
	pose_trajectories[t, 3] = float(rotation_value_string_list[0])
	pose_trajectories[t, 4] = float(rotation_value_string_list[1])
	pose_trajectories[t, 5] = float(rotation_value_string_list[2])
	
	# chest to belly
	applyMMMRotationToURDFJoint(human_adult_ID, 0,
		float(value_string_list[6]),
		float(value_string_list[7]),
		float(value_string_list[8]), inverse = True)

	# belly to pelvis
	applyMMMRotationToURDFJoint(human_adult_ID, 1,
		float(value_string_list[3]),
		float(value_string_list[4]),
		float(value_string_list[5]), inverse = True)

	# pelvis to right leg
	applyMMMRotationToURDFJoint(human_adult_ID, 2,
		float(value_string_list[33]),
		float(value_string_list[34]),
		float(value_string_list[35]))
	# pelvis to left leg
	applyMMMRotationToURDFJoint(human_adult_ID, 3,
		float(value_string_list[17]),
		float(value_string_list[18]),
		float(value_string_list[19]))
	
	# right leg to right shin
	p.resetJointState(human_adult_ID, 4, -float(value_string_list[36]))
	# left leg to left shin
	p.resetJointState(human_adult_ID, 5, -float(value_string_list[20]))

	# right shin to right foot
	applyMMMRotationToURDFJoint(human_adult_ID, 6,
		float(value_string_list[28]),
		float(value_string_list[29]),
		float(value_string_list[30]))
	# left shin to left foot
	applyMMMRotationToURDFJoint(human_adult_ID, 7,
		float(value_string_list[12]),
		float(value_string_list[13]),
		float(value_string_list[14]))

	# right_shoulder_to_right_arm
	applyMMMRotationToURDFJoint(human_adult_ID, 10,
		float(value_string_list[37]),
		float(value_string_list[38]),
		float(value_string_list[39]))

	# left_shoulder_to_left_arm
	applyMMMRotationToURDFJoint(human_adult_ID, 11,
		float(value_string_list[21]),
		float(value_string_list[22]),
		float(value_string_list[23]))

	# right arm to right forearm
	p.resetJointState(human_adult_ID, 12, -float(value_string_list[31]))
	# left arm to left forearm
	p.resetJointState(human_adult_ID, 13, -float(value_string_list[15]))

	# right_forearm_to_right_hand
	applyMMMRotationToURDFJoint(human_adult_ID, 14,
		float(value_string_list[40]),
		float(value_string_list[41]),
		0.0)

	# left_forearm_to_left_hand
	applyMMMRotationToURDFJoint(human_adult_ID, 15,
		float(value_string_list[24]),
		float(value_string_list[25]),
		0.0)

	# chest_to_neck
	applyMMMRotationToURDFJoint(human_adult_ID, 16,
		float(value_string_list[0]),
		float(value_string_list[1]),
		float(value_string_list[2]))

	# neck_to_head
	applyMMMRotationToURDFJoint(human_adult_ID, 17,
		float(value_string_list[9]),
		float(value_string_list[10]),
		float(value_string_list[11]))

	# right foot to right sole
	p.resetJointState(human_adult_ID, 18, float(value_string_list[43]))
	# left foot to left sole
	p.resetJointState(human_adult_ID, 19, float(value_string_list[27]))

	# right sole to right toes
	p.resetJointState(human_adult_ID, 20, -float(value_string_list[42]))
	# left sole to left toes
	p.resetJointState(human_adult_ID, 21, -float(value_string_list[26]))

	# Base rotation and Zero Translation (for now)
	applyMMMRotationAndZeroTranslationToURDFBody(human_adult_ID,
		float(rotation_value_string_list[0]),
		float(rotation_value_string_list[1]),
		float(rotation_value_string_list[2]))

	# Base translation
	applyMMMTranslationToURDFBody(human_adult_ID,
		float(position_value_string_list[0])/1000 - x0,
		float(position_value_string_list[1])/1000 - y0,
		float(position_value_string_list[2])/1000 - z0)

	#p.stepSimulation()
	time.sleep(0.01)

while True:
	time.sleep(3.0)

p.disconnect()

for j in range(44):
	plt.plot(joint_position_trajectories[:, j])

plt.show()

plt.plot(pose_trajectories[:, 0], 'r')
plt.plot(pose_trajectories[:, 1], 'g')
plt.plot(pose_trajectories[:, 2], 'b')
plt.show()

plt.plot(pose_trajectories[:, 3], 'r')
plt.plot(pose_trajectories[:, 4], 'g')
plt.plot(pose_trajectories[:, 5], 'b')
plt.show()