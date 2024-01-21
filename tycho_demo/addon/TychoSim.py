import mujoco
import mujoco.viewer
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R

# This is the positional offset from the bottom tip of the leader to the bottom tip of the robot
# This is with inital joint poses [-1.93332, 2.10996, 2.33660, 0.23878, 1.20642, 0.00378, -0.37602]
POSITIONAL_OFFSET = [ 0.40315882, 0.10149135, -0.00672808 ]
# This is the initial pose of the leader on sim init
LEADER_OFFSET = [ 0.13087, -0.13826, 0.11274 ]

class TychoSim:
    def __init__(self, joint_positions, model_path='./src/mujoco_assets/assets/hebi.xml'):
        np.set_printoptions(suppress=True,precision=5)
        #os.environ['MUJOCO_GL']='cgl'
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.m.opt.timestep = 0.002
        self.d = mujoco.MjData(self.m)

        self.set_joint_positions(joint_positions)
        mujoco.mj_step(self.m, self.d)

        self.viewer = None  # Viewer will be initialized in the run_simulation method

        # Define actuator indices
        self.x_actuator_idx = self.m.actuator('x').id
        self.y_actuator_idx = self.m.actuator('y').id
        self.z_actuator_idx = self.m.actuator('z').id
        self.rx_actuator_idx = self.m.actuator('rx').id
        self.ry_actuator_idx = self.m.actuator('ry').id
        self.rz_actuator_idx = self.m.actuator('rz').id

        self.chop_x_actuator_idx = self.m.actuator('tx').id
        self.chop_y_actuator_idx = self.m.actuator('ty').id
        self.chop_z_actuator_idx = self.m.actuator('tz').id
        self.chop_rx_actuator_idx = self.m.actuator('trx').id
        self.chop_ry_actuator_idx = self.m.actuator('try').id
        self.chop_rz_actuator_idx = self.m.actuator('trz').id

        self.chop_x_offset = 0.0
        self.chop_y_offset = 0.0
        self.chop_z_offset = 0.0

        # Initial leader position and orientation
        self.leader_site_idx = self.m.site('leader_fixed_chop_tip_no_rot').id
        self.leader_starting_pos_x = self.d.site_xpos[self.leader_site_idx][0]
        self.leader_starting_pos_y = self.d.site_xpos[self.leader_site_idx][1]
        self.leader_starting_pos_z = self.d.site_xpos[self.leader_site_idx][2]
        self.leader_starting_mat = self.d.site_xmat[self.leader_site_idx]
        
        self.arm_site_idx = self.m.site('fixed_chop_tip_no_rot').id

        # Initial orientation angles
        self.leader_starting_pos_rx = 1.57
        self.leader_starting_pos_ry = 0
        self.leader_starting_pos_rz = 3.14

    def calculate_angle_difference(self, angle1, angle2):
        diff = angle2 - angle1
        if diff > np.pi:
            diff -= 2 * np.pi
        elif diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def run_simulation_thread(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as self.viewer:
            while self.viewer.is_running():
                self.update_simulation_state()
                self.get_comparable_transformations()

    # Returns the difference in target and actual in XYZ
    def get_comparable_transformations(self):
        global target_points, real_points
        # We add this constant offset to project the actual robot onto the leader
        actual_translation = self.d.site_xpos[self.arm_site_idx] + POSITIONAL_OFFSET
        leader_translation = self.d.site_xpos[self.leader_site_idx]

        # Subtract initial leader position, then add leader starting positions
        # This maps these translations to the real world leader position
        # Flip the axes of init pose to match the real world axes
        actual_translation = actual_translation - LEADER_OFFSET + [self.leader_starting_pos_x, self.leader_starting_pos_z, self.leader_starting_pos_y]
        leader_translation = leader_translation - LEADER_OFFSET + [self.leader_starting_pos_x, self.leader_starting_pos_z, self.leader_starting_pos_y]

        actual_rotation = R.from_matrix(self.d.site_xmat[self.leader_site_idx].reshape(3, -1)).as_euler("xyz") - [1.57, 0, 0]
        leader_rotation = R.from_matrix(self.d.site_xmat[self.arm_site_idx].reshape(3, -1)).as_euler("xyz") - [1.57, 0, 0]
        return np.hstack((leader_translation, leader_rotation)), np.hstack((actual_translation, actual_rotation))

    def run_simulation(self):
        simulation_thread = threading.Thread(target=self.run_simulation_thread)
        simulation_thread.start()

    def run_simulation_iterations(self, iterations):
        for i in range(iterations):
            self.update_simulation_state()

    def get_joint_positions(self):
        joint_positions = []

        joint_positions.append(self.d.joint("HEBI/base/X8_9").qpos[0])
        joint_positions.append(self.d.joint("HEBI/shoulder/X8_16").qpos[0])
        joint_positions.append(self.d.joint("HEBI/elbow/X8_9").qpos[0])
        joint_positions.append(self.d.joint("HEBI/wrist1/X5_1").qpos[0])
        joint_positions.append(self.d.joint("HEBI/wrist2/X5_1").qpos[0])
        joint_positions.append(self.d.joint("HEBI/wrist3/X5_1").qpos[0])
        joint_positions.append(self.d.joint("HEBI/chopstick/X5_1").qpos[0])

        return joint_positions

    def set_joint_positions(self, joint_positions):
        self.d.joint("HEBI/base/X8_9").qpos[0] = joint_positions[0]
        self.d.joint("HEBI/shoulder/X8_16").qpos[0] = joint_positions[1]
        self.d.joint("HEBI/elbow/X8_9").qpos[0] = joint_positions[2]
        self.d.joint("HEBI/wrist1/X5_1").qpos[0] = joint_positions[3]
        self.d.joint("HEBI/wrist2/X5_1").qpos[0] = joint_positions[4]
        self.d.joint("HEBI/wrist3/X5_1").qpos[0] = joint_positions[5]
        self.d.joint("HEBI/chopstick/X5_1").qpos[0] = joint_positions[6]

    def set_leader_position(self, leader_positions, leader_rotations):
        self.d.ctrl[self.chop_x_actuator_idx] = -(leader_positions[0] - self.leader_starting_pos_x)
        self.d.ctrl[self.chop_y_actuator_idx] = -(leader_positions[2] - self.leader_starting_pos_y)
        self.d.ctrl[self.chop_z_actuator_idx] = -(leader_positions[1] - self.leader_starting_pos_z)

        ALLOWABLE_ROT_MOVEMENT = 10.0

        self.d.ctrl[self.chop_rx_actuator_idx] = self.calculate_angle_difference(
            -leader_rotations[0], self.leader_starting_pos_rx
        )
        if (
            self.d.ctrl[self.chop_rx_actuator_idx] < -ALLOWABLE_ROT_MOVEMENT
            or self.d.ctrl[self.chop_rx_actuator_idx] > ALLOWABLE_ROT_MOVEMENT
        ):
            self.d.ctrl[self.chop_rx_actuator_idx] = 0.0

        self.d.ctrl[self.chop_ry_actuator_idx] = self.calculate_angle_difference(
            leader_rotations[2], self.leader_starting_pos_ry
        )
        if (
            self.d.ctrl[self.chop_ry_actuator_idx] < -ALLOWABLE_ROT_MOVEMENT
            or self.d.ctrl[self.chop_ry_actuator_idx] > ALLOWABLE_ROT_MOVEMENT
        ):
            self.d.ctrl[self.chop_ry_actuator_idx] = 0.0
            
        self.d.ctrl[self.chop_rz_actuator_idx] = self.calculate_angle_difference(
            -leader_rotations[1], self.leader_starting_pos_rz
        )
        if (
            self.d.ctrl[self.chop_rz_actuator_idx] < -ALLOWABLE_ROT_MOVEMENT
            or self.d.ctrl[self.chop_rz_actuator_idx] > ALLOWABLE_ROT_MOVEMENT
        ):
            self.d.ctrl[self.chop_rz_actuator_idx] = 0.0

    def init_leader_position(self, leader_positions, leader_rotations):
        self.leader_starting_pos_x = leader_positions[0]
        self.leader_starting_pos_y = leader_positions[2]
        self.leader_starting_pos_z = leader_positions[1]

        self.leader_starting_pos_rx = -leader_rotations[0]
        self.leader_starting_pos_ry = leader_rotations[2]
        self.leader_starting_pos_rz = -leader_rotations[1]

    def update_simulation_state(self):
        mujoco.mj_step(self.m, self.d)

        # Update control signals
        self.d.ctrl[self.x_actuator_idx] = self.d.ctrl[self.chop_x_actuator_idx]
        self.d.ctrl[self.y_actuator_idx] = self.d.ctrl[self.chop_y_actuator_idx]
        self.d.ctrl[self.z_actuator_idx] = self.d.ctrl[self.chop_z_actuator_idx]

        self.d.ctrl[self.rx_actuator_idx] = self.d.ctrl[self.chop_rx_actuator_idx]
        self.d.ctrl[self.ry_actuator_idx] = self.d.ctrl[self.chop_ry_actuator_idx]
        self.d.ctrl[self.rz_actuator_idx] = self.d.ctrl[self.chop_rz_actuator_idx]

        if self.viewer is not None:
            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            self.viewer.sync()

if __name__ == '__main__':
    simulation_instance = TychoSim([-1.93332, 2.10996, 2.33660, 0.23878, 1.20642, 0.00378, -0.37602])
    simulation_instance.run_simulation()