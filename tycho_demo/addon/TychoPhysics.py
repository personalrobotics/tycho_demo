from dm_control import mujoco
from dm_control.utils import inverse_kinematics
import numpy as np
from scipy.spatial.transform import Rotation as R

class TychoPhysics:
    def __init__(self, joint_positions, model_path='./src/mujoco_assets/assets/hebi.xml'):
        self.physics = mujoco.Physics.from_xml_path(model_path)
        self.set_joint_positions(joint_positions)

    def set_joint_positions(self, joint_positions):
        with self.physics.reset_context():
            self.physics.named.data.qpos["HEBI/base/X8_9"] = joint_positions[0]
            self.physics.named.data.qpos["HEBI/shoulder/X8_16"] = joint_positions[1]
            self.physics.named.data.qpos["HEBI/elbow/X8_9"] = joint_positions[2]
            self.physics.named.data.qpos["HEBI/wrist1/X5_1"] = joint_positions[3]
            self.physics.named.data.qpos["HEBI/wrist2/X5_1"] = joint_positions[4]
            self.physics.named.data.qpos["HEBI/wrist3/X5_1"] = joint_positions[5]
            self.physics.named.data.qpos["HEBI/chopstick/X5_1"] = joint_positions[6]

    def get_IK(self, target_pos, target_quat, max_steps):
        IK_all_joints = inverse_kinematics.qpos_from_site_pose(
            self.physics,
            site_name="fixed_chop_tip_no_rot",
            target_pos=target_pos,
            target_quat=target_quat,
            joint_names=np.array(["HEBI/base/X8_9", "HEBI/shoulder/X8_16", "HEBI/elbow/X8_9", "HEBI/wrist1/X5_1", "HEBI/wrist2/X5_1", "HEBI/wrist3/X5_1", "HEBI/chopstick/X5_1"]),
            max_steps=max_steps,
            max_update_norm=np.Infinity,
            progress_thresh=np.Infinity,
            regularization_strength=0.0,
            tol=0.0,
            inplace=True)
        translation = self.physics.named.data.site_xpos["fixed_chop_tip_no_rot"]

        return IK_all_joints.qpos[0:7], IK_all_joints.err_norm
    
    def get_FK(self):
        translation = self.physics.named.data.site_xpos["fixed_chop_tip_no_rot"]
        euler = R.from_matrix(self.physics.named.data.site_xmat["fixed_chop_tip_no_rot"].reshape(3, -1)).as_euler("xyz")
        actual_euler = [euler[2] - np.pi / 2, -euler[1], -euler[0] + np.pi]
        return np.hstack((translation, actual_euler))

if __name__ == '__main__':
    simulation_instance = TychoPhysics([-2.05810131, 1.41744053, 2.06072881, -0.9061228, 1.57976982, -2.04056325, -0.37010479])