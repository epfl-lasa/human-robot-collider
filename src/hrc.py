import argparse
import logging

import numpy as np

from human import Man, Child
from robot import Qolo
from controller import NoControl, AdmittanceController, PassiveDSController
from simulator import Simulator


human_class = {
    "man": Man,
    "child": Child,
}
robot_class = {
    "qolo": Qolo,
}
controller_class = {
    "no_control": NoControl,
    "admittance": AdmittanceController,
    "passive_ds": PassiveDSController,
}


def parse_arguments():
    parser = argparse.ArgumentParser(
        prog="HRC",
        description="""Simulation of robot and human collision"""
    )
    parser.add_argument("-b", "--human",
                        choices=[key for key in human_class],
                        default="man",
                        help="Human to collide with the robot (default = man)")
    parser.add_argument("-r", "--robot",
                        choices=[key for key in robot_class],
                        default="qolo",
                        help="Robot to collide with the human (default = qolo)")
    parser.add_argument("-c", "--controller",
                        choices=[key for key in controller_class],
                        default="no_control",
                        help="Adaptive controller to use (default = no_control)")
    parser.add_argument("-g", "--gui",
                        action="store_true",
                        help="Set to show GUI")
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_arguments()

    simulator = Simulator(
        Robot=robot_class[args.robot],
        Human=human_class[args.human],
        Controller=controller_class[args.controller],
        show_GUI=args.gui,
    )
    result = [
        simulator.simulate(
            robot_angle=robot_angle,
            human_angle=human_angle,
            gait_phase=gait_phase,
            robot_speed_factor=robot_speed_factor,
            human_speed_factor=human_speed_factor,
        )
        for robot_angle in np.linspace(0, np.pi*2, 16, False)
        for human_angle in np.linspace(0, np.pi*2, 16, False)
        for robot_speed_factor in np.linspace(0.6, 1.4, 3, True)
        for human_speed_factor in [1.0]
        for gait_phase in np.linspace(0, 1, 4, False)
    ]

    np.save("controlled_collision.npy", result)
