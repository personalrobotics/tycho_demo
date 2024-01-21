from tycho_env.utils import CHOPSTICK_CLOSE, CHOPSTICK_OPEN
from mujoco_assets.TychoSim import TychoSim
import numpy as np
from scipy.spatial.transform import Rotation as R

NUM_ITERATION = 100
BOTTOM_TIP_DISTANCE = 0.128

def get_IK_from_mujoco(sim, current_joint_position,
                      target_transformation,
                      target_open=None):
    x_axis = target_transformation[0:3, 0]
    bottom_chop_middle_point = target_transformation[0:3, 3]  # anchor_point

    # Calculate bottom_tip position
    bottom_tip = bottom_chop_middle_point + BOTTOM_TIP_DISTANCE * x_axis

    sim.set_joint_positions(current_joint_position)
    sim.set_leader_position(bottom_tip, R.from_matrix(target_transformation[0:3,0:3]).as_euler("xyz"))
    sim.run_simulation_iterations(NUM_ITERATION)
    next_angles = sim.get_joint_positions()
    if target_open:
        next_angles[-1] = np.clip(target_open, CHOPSTICK_CLOSE, CHOPSTICK_OPEN)
    return next_angles