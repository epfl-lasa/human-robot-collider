
import numpy as np
import pybullet as p

# Based on ISO/TS 15066 for 75 kg
eff_mass_human = np.array([
    40  , 	    # chest
    40  , 	    # belly
    40  , 	    # pelvis
    75  , 75  , # upper legs    <-- Previous implementation was different
    75  , 75  , # shins         <-- Previous implementation was different
    75  , 75  , # ankles/feet (Same as shin)    <-- Previous implementation was different
    3   , 3   , # upper arms    
    2   , 2   , # forearms
    0.6 , 0.6 , # hands
    1.2 , 	    # neck
    8.8 , 	    # head + face (?)
    75  , 75  , # soles (Same as shin)  <-- Previous implementation was different
    75  , 75  , # toes (Same as shin)   <-- Previous implementation was different
    40  , 	    # chest (back)
    40  , 	    # belly (back)
    40  , 	    # pelvis (back)
    1.2 , 	    # neck (back)
    4.4 , 	    # head (back)
]) / 75


eff_spring_const_human = np.array([
    25  , 	    # chest
    10  , 	    # belly
    25  , 	    # pelvis
    50  , 50  , # upper legs
    60  , 60  , # shins
    75  , 75  , # ankles/feet (Same as shin)    <-- From Previous implementation
    30  , 30  , # upper arms    <-- Previous implementation was different
    40  , 40  , # forearms      <-- Previous implementation was different
    75  , 75  , # hands
    50  , 	    # neck
    150 , 	    # head (face ?)
    75  , 75  , # soles (Same as shin)  <-- From Previous implementation
    75  , 75  , # toes (Same as shin)   <-- From Previous implementation
    35  , 	    # chest (back)
    35  , 	    # belly (back)
    35  , 	    # pelvis (back)
    50  , 	    # neck (back)
    150 , 	    # head (back)
]) * 1e3


eff_mass_robot = np.array([
    50  ,    # Main Body
    1.5 ,    # Left Wheel
    1.5 ,    # Right Wheel
    1.5 ,    # Bumper
]) / 54.5


eff_spring_const_robot = np.array([
    10 ,    # Main Body
    1  ,    # Left Wheel
    1  ,    # Right Wheel
    10 ,    # Bumper
]) * 1e3


class Collision:
    def __init__(self,
				 pybtPhysicsClient,
                 robot,
                 human,
                 human_mass=75,
                 robot_mass=54.5,
                 bumper_height=0.215,
                 ftsensor_loc=0.035,
                 ):
        self.pybtPhysicsClient = pybtPhysicsClient
        self.robot = robot
        self.human = human
        self.human_mass = human_mass
        self.robot_mass = robot_mass
        self.bumper_height = bumper_height
        self.ftsensor_loc = ftsensor_loc
        self.solver = None

    def get_collision_force(self):
        contact_points = p.getContactPoints(
            self.human.body_id,
            self.robot.body_id,
        )

        for contact_point in contact_points:
            if contact_point[8] <= 0:
                # Penetration or Contact
                human_part_id = contact_point[3]
                robot_part_id = contact_point[4]
                pos_on_robot = contact_point[6]
                normal_on_robot = np.array(contact_point[7])
                
                self.__collide(robot_part_id, human_part_id)

                F = self.__get_contact_force(contact_point[8])
                (h, theta) = self.__get_loc_on_bumper(pos_on_robot)

                return F
        
        return None

    def __collide(self,
                robot_part_id,
                human_part_id):
        self.eff_mass_robot = self.__get_eff_mass_robot(robot_part_id)
        self.eff_mass_human = self.__get_eff_mass_human(human_part_id)

        k_robot = self.__get_eff_spring_const_human(robot_part_id)
        k_human = self.__get_eff_spring_const_human(human_part_id)
        self.eff_spring_const = 1 / (1/k_robot + 1/k_human)

    def __get_eff_mass_human(self, part_id):
        return eff_mass_human[part_id] * self.human_mass

    def __get_eff_spring_const_human(self, part_id):
        return eff_spring_const_human[part_id]

    def __get_eff_spring_const_robot(self, part_id):
        return eff_spring_const_robot[part_id]

    def __get_eff_mass_robot(self, part_id):
        return eff_mass_robot[part_id] * self.robot_mass
    
    def __get_contact_force(self, penetration):
        if penetration > 0:
            # No Contact
            return 0
        else:
            return (-self.eff_spring_const * penetration)
            
    def __get_loc_on_bumper(self, pos):
        theta = np.arctan2(pos[0], - pos[1] - self.ftsensor_loc)
        h = self.bumper_height - pos[2]
        return (h, theta)


if __name__ == "__main__":
    pass
