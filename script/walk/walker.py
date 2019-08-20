import pybullet as p

import numpy as np

import math

import os

class Man:
	def __init__(self, pybtPhysicsClient):
		self.body_id = p.loadURDF("../data/human_adult_scan.urdf",
			flags=p.URDF_MAINTAIN_LINK_ORDER,
			physicsClientId = pybtPhysicsClient)

		# pose containers
		self.global_xyz = np.zeros([3])
		self.global_rpy = np.zeros([3])
		self.global_rpy[2] = -np.pi/2

		self.other_xyz = np.zeros([3])
		self.other_rpy = np.zeros([3])

		self.joint_positions = np.zeros([44])

		# gait motion data
		self.cyclic_joint_positions = np.load(
			os.path.join(os.path.dirname(__file__), "cyclic_joint_positions.npy"))
		self.cyclic_pelvis_rotations = np.load(
			os.path.join(os.path.dirname(__file__), "cyclic_pelvis_rotations.npy"))
		self.cyclic_pelvis_forward_velocity = np.load(
			os.path.join(os.path.dirname(__file__), "cyclic_pelvis_forward_velocity.npy"))
		self.cyclic_pelvis_lateral_position = np.load(
			os.path.join(os.path.dirname(__file__), "cyclic_pelvis_lateral_position.npy"))
		self.cyclic_pelvis_vertical_position = np.load(
			os.path.join(os.path.dirname(__file__), "cyclic_pelvis_vertical_position.npy"))
		self.cycle_time_steps = np.load(
			os.path.join(os.path.dirname(__file__), "cycle_time_steps.npy"))

		self.setGaitPhase(0)

	def greet(self):
		print("Hello")

	def resetToReference(self):
		pass

	def resetGlobalTransformation(self, xyz, rpy):
		for i in range(3):
			self.global_xyz[i] = xyz[i]
			self.global_rpy[i] = rpy[i]
		self.global_rpy[2] += -np.pi/2

		self.other_xyz[0] = 0.0
		self.setGaitPhase(0)
		

	def setGaitPhase(self, period_fraction):
		period_fraction = abs(period_fraction) - int(abs(period_fraction))
		self.gait_phase_step = int(period_fraction*np.size(self.cycle_time_steps))

		self.other_xyz[1] = self.cyclic_pelvis_lateral_position[self.gait_phase_step]
		self.other_xyz[2] = self.cyclic_pelvis_vertical_position[self.gait_phase_step]

		self.other_rpy[:] = self.cyclic_pelvis_rotations[:, self.gait_phase_step]

		self.joint_positions[:] = self.cyclic_joint_positions[:, self.gait_phase_step]

		self.__apply_pose()

	def advance(self):
		self.gait_phase_step += 1
		if self.gait_phase_step == np.size(self.cycle_time_steps):
			self.gait_phase_step = 0

		self.other_xyz[:] += self.cyclic_pelvis_forward_velocity[self.gait_phase_step]*0.01

		self.other_xyz[1] = self.cyclic_pelvis_lateral_position[self.gait_phase_step]
		self.other_xyz[2] = self.cyclic_pelvis_vertical_position[self.gait_phase_step]

		self.other_rpy[:] = self.cyclic_pelvis_rotations[:, self.gait_phase_step]

		self.joint_positions[:] = self.cyclic_joint_positions[:, self.gait_phase_step]

		self.__apply_pose()

	def __apply_pose(self):
		# chest to belly
		applyMMMRotationToURDFJoint(self.body_id, 0,
			self.joint_positions[6],
			self.joint_positions[7],
			self.joint_positions[8], inverse = True)

		# belly to pelvis
		applyMMMRotationToURDFJoint(self.body_id, 1,
			self.joint_positions[3],
			self.joint_positions[4],
			self.joint_positions[5], inverse = True)

		# pelvis to right leg
		applyMMMRotationToURDFJoint(self.body_id, 2,
			self.joint_positions[33],
			self.joint_positions[34],
			self.joint_positions[35])
		# pelvis to left leg
		applyMMMRotationToURDFJoint(self.body_id, 3,
			self.joint_positions[17],
			self.joint_positions[18],
			self.joint_positions[19])
		
		# right leg to right shin
		p.resetJointState(self.body_id, 4, -self.joint_positions[36])
		# left leg to left shin
		p.resetJointState(self.body_id, 5, -self.joint_positions[20])

		# right shin to right foot
		applyMMMRotationToURDFJoint(self.body_id, 6,
			self.joint_positions[28],
			self.joint_positions[29],
			self.joint_positions[30])
		# left shin to left foot
		applyMMMRotationToURDFJoint(self.body_id, 7,
			self.joint_positions[12],
			self.joint_positions[13],
			self.joint_positions[14])

		# chest_to_right_arm
		applyMMMRotationToURDFJoint(self.body_id, 8,
			self.joint_positions[37],
			self.joint_positions[38],
			self.joint_positions[39])

		# chest_to_left_arm
		applyMMMRotationToURDFJoint(self.body_id, 9,
			self.joint_positions[21],
			self.joint_positions[22],
			self.joint_positions[23])

		# right arm to right forearm
		p.resetJointState(self.body_id, 10, -self.joint_positions[31])
		# left arm to left forearm
		p.resetJointState(self.body_id, 11, -self.joint_positions[15])

		# right_forearm_to_right_hand
		applyMMMRotationToURDFJoint(self.body_id, 12,
			self.joint_positions[40],
			self.joint_positions[41],
			0.0)

		# left_forearm_to_left_hand
		applyMMMRotationToURDFJoint(self.body_id, 13,
			self.joint_positions[24],
			self.joint_positions[25],
			0.0)

		# chest_to_neck
		applyMMMRotationToURDFJoint(self.body_id, 14,
			self.joint_positions[0],
			self.joint_positions[1],
			self.joint_positions[2])

		# neck_to_head
		applyMMMRotationToURDFJoint(self.body_id, 15,
			self.joint_positions[9],
			self.joint_positions[10],
			self.joint_positions[11])

		# right foot to right sole
		p.resetJointState(self.body_id, 16, self.joint_positions[43])
		# left foot to left sole
		p.resetJointState(self.body_id, 17, self.joint_positions[27])

		# right sole to right toes
		p.resetJointState(self.body_id, 18, -self.joint_positions[42])
		# left sole to left toes
		p.resetJointState(self.body_id, 19, -self.joint_positions[26])

		# Base rotation and Zero Translation (for now)
		self.__applyMMMRotationAndZeroTranslationToURDFBody(
			self.other_rpy[0],
			self.other_rpy[1],
			self.other_rpy[2])

		# Base translation
		self.__applyMMMTranslationToURDFBody(
			self.other_xyz[0],
			self.other_xyz[1],
			self.other_xyz[2])

	# call this function AFTER applying the BT- and BP-joint angles for the urdf (as shown above) 
	def __applyMMMRotationAndZeroTranslationToURDFBody(self, rx, ry, rz):
		# get the rotation from the (hypothetical) world to the pelvis
		tpcom, rpcom, tplcom, rplcom, tpf, rotation_pelvis_frame, v, omega = p.getLinkState(
			self.body_id, 1, True, True)

		# get the rotation from the (hypothetical) world to the chest
		t_com, r_com = p.getBasePositionAndOrientation(self.body_id)
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

		# pre-multiply with global transform
		#t_final, r_final = p.multiplyTransforms([self.global_xyz[0], self.global_xyz[1], self.global_xyz[2]],
		#	p.getQuaternionFromEuler([self.global_rpy[0], self.global_rpy[1], self.global_rpy[2]]),
		#	[0, 0, 0], r_world_to_chest)

		# apply it to the base together with a zero translation
		p.resetBasePositionAndOrientation(self.body_id,
			[100, 100, 100], r_world_to_chest)
		#	t_final, r_final)

	def __applyMMMTranslationToURDFBody(self, tx, ty, tz):
		# get the translation to the pelvis frame
		#tpcom, rpcom, tplcom, rplcom, translation_to_pelvis_frame, rpf, v, omega = p.getLinkState(
		#	urdf_body_id, 1, True, True)

		# get the translation to the left leg frame
		tllcom, rllcom, tlllcom, rlllcom, translation_to_left_leg_frame, rllf, vll, omegall = p.getLinkState(
			self.body_id, 3, True, True)

		# get the translation to the right leg frame
		trlcom, rrlcom, trllcom, rrllcom, translation_to_right_leg_frame, rrlf, vrl, omegarl = p.getLinkState(
			self.body_id, 2, True, True)

		t_base_com, r_base_com = p.getBasePositionAndOrientation(self.body_id)

		t_phb_center_to_base_com = [0,0,0]
		for i in range(3):
			t_phb_center_to_base_com[i] = t_base_com[i] - 0.5*(
				translation_to_right_leg_frame[i] + translation_to_left_leg_frame[i])

		t_base_com = [
			tx + t_phb_center_to_base_com[0],
			ty + t_phb_center_to_base_com[1],
			tz + t_phb_center_to_base_com[2]]

		# pre-multiply with global transform
		t_final, r_final = p.multiplyTransforms([self.global_xyz[0], self.global_xyz[1], self.global_xyz[2]],
			p.getQuaternionFromEuler([self.global_rpy[0], self.global_rpy[1], self.global_rpy[2]]),
			t_base_com, r_base_com)

		# apply it to the base together with a zero translation
		p.resetBasePositionAndOrientation(self.body_id,
			t_final, r_final)
		#p.resetBasePositionAndOrientation(self.body_id, t_base_com, r_base_com)


def generateQuaternionFromMMMRxRyRz(rx, ry, rz):
	q_intermediate = p.getQuaternionFromEuler([math.pi/2, 0, math.pi/2])
	t, q_intermediate_inv = p.invertTransform([0,0,0], q_intermediate)
	q_tmp = p.getQuaternionFromEuler([ry, rz, rx])
	t, q_tmp = p.multiplyTransforms([0,0,0], q_intermediate, [0,0,0], q_tmp)
	t, q = p.multiplyTransforms([0,0,0], q_tmp, [0,0,0], q_intermediate_inv)
	return q

def applyMMMRotationToURDFJoint(urdf_body_id, joint_index, rx, ry, rz, inverse=False):
	q = generateQuaternionFromMMMRxRyRz(rx, ry, rz)
	quat_tf_urdf = p.getQuaternionFromEuler([-math.pi/2, math.pi, 0])
	translation, quat_tf_urdf_inv = p.invertTransform([0,0,0], quat_tf_urdf)
	t, q = p.multiplyTransforms([0,0,0], quat_tf_urdf, [0,0,0], q)
	t, q = p.multiplyTransforms([0,0,0], q, [0,0,0], quat_tf_urdf_inv)

	if inverse:
		t, q = p.invertTransform([0,0,0], q)

	p.resetJointStateMultiDof(urdf_body_id, joint_index, q)