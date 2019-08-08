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

def applyMMMRotationToURDFJoint(urdf_body_id, joint_index, rx, ry, rz, inverse=False):
	quat_0 = p.getQuaternionFromEuler([rx,rz,-ry])

	quat_tf = p.getQuaternionFromEuler([math.pi/2,0,0])
	translation, quat_tf_inv = p.invertTransform([0,0,0], quat_tf)

	quat_tf_urdf = p.getQuaternionFromEuler([-math.pi/2,0,-math.pi])
	translation, quat_tf_urdf_inv = p.invertTransform([0,0,0], quat_tf_urdf)

	translation, quat_tmp = p.multiplyTransforms([0,0,0], quat_tf_urdf,
		[0,0,0], quat_tf)

	translation, quat_tmp = p.multiplyTransforms([0,0,0], quat_tmp,
		[0,0,0], quat_0)

	translation, quat_tmp = p.multiplyTransforms([0,0,0], quat_tmp,
		[0,0,0], quat_tf_inv)

	translation, quat_final = p.multiplyTransforms([0,0,0], quat_tmp,
		[0,0,0], quat_tf_urdf_inv)

	if inverse:
		translation, quat_final = p.invertTransform([0,0,0], quat_final)

	p.resetJointStateMultiDof(urdf_body_id, joint_index, quat_final)

# call this function AFTER applying the BT- and BP-joint angles for the urdf (using the function above) 
def applyMMMPoseToURDFBody(urdf_body_id, px, py, pz, rx, ry, rz):
	# get the rotation from the pelvis to the chest
	p.get

# load the human-adult model
physicsClient = p.connect(p.GUI)
human_adult_ID = p.loadURDF("../data/human_adult_with_feet.urdf",
	flags=p.URDF_MAINTAIN_LINK_ORDER)

#the following file is from https://motion-database.humanoids.kit.edu/details/motions/37/
tree = ET.parse('../data/WalkingStraightForwards01.xml')
root = tree.getroot()


time.sleep(1.0)

joint_position_trajectories = np.zeros([len(root[0][3]), 44])
pose_trajectories = np.zeros([len(root[0][3]), 6])

for t in range(len(root[0][3])):

	p.resetBasePositionAndOrientation(human_adult_ID,
		[0, 0, 0], p.getQuaternionFromEuler([0,0,0]))

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
	applyMMMRotationToURDFJoint(human_adult_ID, 3,
		float(value_string_list[33]),
		float(value_string_list[34]),
		float(value_string_list[35]))
	# pelvis to left leg
	applyMMMRotationToURDFJoint(human_adult_ID, 4,
		float(value_string_list[17]),
		float(value_string_list[18]),
		float(value_string_list[19]))
	
	# right leg to right shin
	p.resetJointState(human_adult_ID, 5, -float(value_string_list[36]))
	# left leg to left shin
	p.resetJointState(human_adult_ID, 6, -float(value_string_list[20]))

	# right shin to right foot
	applyMMMRotationToURDFJoint(human_adult_ID, 7,
		float(value_string_list[28]),
		float(value_string_list[29]),
		float(value_string_list[30]))
	# left shin to left foot
	applyMMMRotationToURDFJoint(human_adult_ID, 8,
		float(value_string_list[12]),
		float(value_string_list[13]),
		float(value_string_list[14]))

	# right_shoulder_to_right_arm
	applyMMMRotationToURDFJoint(human_adult_ID, 12,
		float(value_string_list[37]),
		float(value_string_list[38]),
		float(value_string_list[39]))

	# left_shoulder_to_left_arm
	applyMMMRotationToURDFJoint(human_adult_ID, 13,
		float(value_string_list[21]),
		float(value_string_list[22]),
		float(value_string_list[23]))

	# right arm to right forearm
	p.resetJointState(human_adult_ID, 14, -float(value_string_list[31]))
	# left arm to left forearm
	p.resetJointState(human_adult_ID, 15, -float(value_string_list[15]))

	# right_forearm_to_right_hand
	applyMMMRotationToURDFJoint(human_adult_ID, 16,
		float(value_string_list[40]),
		float(value_string_list[41]),
		0.0)

	# left_forearm_to_left_hand
	applyMMMRotationToURDFJoint(human_adult_ID, 17,
		float(value_string_list[24]),
		float(value_string_list[25]),
		0.0)

	# chest_to_neck
	applyMMMRotationToURDFJoint(human_adult_ID, 18,
		float(value_string_list[0]),
		float(value_string_list[1]),
		float(value_string_list[2]))

	# neck_to_head
	applyMMMRotationToURDFJoint(human_adult_ID, 19,
		float(value_string_list[9]),
		float(value_string_list[10]),
		float(value_string_list[11]))

	# right foot to right sole
	p.resetJointState(human_adult_ID, 20, -float(value_string_list[43]))
	# left foot to left sole
	p.resetJointState(human_adult_ID, 21, -float(value_string_list[27]))

	# right sole to right toes
	p.resetJointState(human_adult_ID, 22, -float(value_string_list[42]))
	# left sole to left toes
	p.resetJointState(human_adult_ID, 23, -float(value_string_list[26]))

	#p.stepSimulation()
	time.sleep(0.01)

time.sleep(3.0)

p.disconnect()

for j in range(44):
	plt.plot(joint_position_trajectories[:, j])

plt.show()

quit()

plt.plot(pose_trajectories[:, 0], 'r')
plt.plot(pose_trajectories[:, 1], 'g')
plt.plot(pose_trajectories[:, 2], 'b')
plt.show()

plt.plot(pose_trajectories[:, 3], 'r')
plt.plot(pose_trajectories[:, 4], 'g')
plt.plot(pose_trajectories[:, 5], 'b')
plt.show()