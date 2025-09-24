import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
import numpy as np
from math import tan, pi, degrees, acos
import cv2, time, os
cv2.setNumThreads(1)          # use one worker, disables the buggy pool
cv2.ocl.setUseOpenCL(False)   # stop OpenCL spawning extra threads
os.environ['OPENCV_OPENCL_DEVICE'] = 'disabled'


class T265_streaming():

    def __init__(self, serial_number="929122110141", enable_fisheye=False):
        self.enable_fisheye = enable_fisheye
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.pose)
        if self.enable_fisheye:
            # Enable fisheye streams (for image)
            self.cfg.enable_stream(rs.stream.fisheye, 1)
            self.cfg.enable_stream(rs.stream.fisheye, 2)

        if serial_number is not None:
            self.cfg.enable_device(serial_number)

        self.pipe.start(self.cfg)
        print(f'Initialized T265 with serial: {serial_number}')
        if self.enable_fisheye:
            self._init_stereo_undistort()

    def _init_stereo_undistort(self,):
        # Get stream profiles and intrinsics for fisheye cameras
        profile = self.pipe.get_active_profile()
        left_stream = profile.get_stream(rs.stream.fisheye, 1).as_video_stream_profile()
        right_stream = profile.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()
        self.left_intrinsics = left_stream.get_intrinsics()
        self.right_intrinsics = right_stream.get_intrinsics()

        K_left = np.array([[self.left_intrinsics.fx, 0, self.left_intrinsics.ppx],
                           [0, self.left_intrinsics.fy, self.left_intrinsics.ppy],
                           [0, 0, 1]])
        D_left = np.array(self.left_intrinsics.coeffs[:4])
        K_right = np.array([[self.right_intrinsics.fx, 0, self.right_intrinsics.ppx],
                            [0, self.right_intrinsics.fy, self.right_intrinsics.ppy],
                            [0, 0, 1]])
        D_right = np.array(self.right_intrinsics.coeffs[:4])

        # Get extrinsics between cameras
        extrinsics = self.pipe.get_active_profile().get_stream(rs.stream.fisheye, 1).get_extrinsics_to(
            self.pipe.get_active_profile().get_stream(rs.stream.fisheye, 2))
        R = np.reshape(extrinsics.rotation, (3,3)).T
        T = np.array(extrinsics.translation)
         # Stereo rectification params
        stereo_fov_rad = 110 * (pi/180)
        stereo_height_px = 300
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)
        max_disp = 112
        stereo_width_px = stereo_height_px + max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + max_disp
        stereo_cy = (stereo_height_px - 1)/2

        R_left = np.eye(3)
        R_right = R

        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                           [0, stereo_focal_px, stereo_cy, 0],
                           [0, 0, 1, 0]])
        P_right = P_left.copy()
        P_right[0][3] = T[0] * stereo_focal_px

        self.left_map1, self.left_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_left, D_left, R_left, P_left, stereo_size, cv2.CV_32FC1)
        self.right_map1, self.right_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_right, D_right, R_right, P_right, stereo_size, cv2.CV_32FC1)

    def _q_norm(self, q):
        w, x, y, z = q
        s = (w*w + x*x + y*y + z*z) ** 0.5
        if s == 0:
            return (1.0, 0.0, 0.0, 0.0)
        return (w/s, x/s, y/s, z/s)

    def _quat_angle_deg_from_R(self, R_rel):
        # Convert relative rotation matrix to quaternion (trace method), then to angle
        tr = float(R_rel[0, 0] + R_rel[1, 1] + R_rel[2, 2])
        if tr > 0:
            S = (tr + 1.0) ** 0.5 * 2.0
            w = 0.25 * S
            x = (R_rel[2, 1] - R_rel[1, 2]) / S
            y = (R_rel[0, 2] - R_rel[2, 0]) / S
            z = (R_rel[1, 0] - R_rel[0, 1]) / S
        else:
            i = int(np.argmax(np.diag(R_rel)))
            if i == 0:
                S = (1.0 + R_rel[0, 0] - R_rel[1, 1] - R_rel[2, 2]) ** 0.5 * 2.0
                w = (R_rel[2, 1] - R_rel[1, 2]) / S
                x = 0.25 * S
                y = (R_rel[0, 1] + R_rel[1, 0]) / S
                z = (R_rel[0, 2] + R_rel[2, 0]) / S
            elif i == 1:
                S = (1.0 - R_rel[0, 0] + R_rel[1, 1] - R_rel[2, 2]) ** 0.5 * 2.0
                w = (R_rel[0, 2] - R_rel[2, 0]) / S
                x = (R_rel[0, 1] + R_rel[1, 0]) / S
                y = 0.25 * S
                z = (R_rel[1, 2] + R_rel[2, 1]) / S
            else:
                S = (1.0 - R_rel[0, 0] - R_rel[1, 1] + R_rel[2, 2]) ** 0.5 * 2.0
                w = (R_rel[1, 0] - R_rel[0, 1]) / S
                x = (R_rel[0, 2] + R_rel[2, 0]) / S
                y = (R_rel[1, 2] + R_rel[2, 1]) / S
                z = 0.25 * S
        w, x, y, z = self._q_norm((w, x, y, z))
        w = max(-1.0, min(1.0, abs(w)))
        return 2.0 * degrees(acos(w))

    def _read(self,):
        # Wait for the next set of frames from the camera
        frames = self.pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()

        if pose:

            # Print some of the pose data to the terminal
            data = pose.get_pose_data()

            translation = [data.translation.x, data.translation.y, data.translation.z]
            quat = [data.rotation.x,data.rotation.y,data.rotation.z,data.rotation.w]
            r = Rotation.from_quat(quat)

            # Get the rotation matrix
            matrix = r.as_matrix()
            T_4x4 = np.zeros((4,4))
            T_4x4[:3,:3] = matrix
            T_4x4[:3,3] = translation
            T_4x4[3][3] = 1
            rot = np.zeros((4,4))
            rot2 = np.zeros((4,4))
            # x axis rotate 90 then z rotate 180
            rot[:3,:3] = np.array(
            [[-1, 0, 0],
              [0, 0,1],
              [0, -1,0]]
                        )
            rot[3,3] = 1
            rot2[:3,:3] = np.array(
            [[1, 0, 0],
              [0, 1,0],
              [0, 0,-1]]
                        )
            rot2[3,3] = 1

        if self.enable_fisheye:
            left_frame = frames.get_fisheye_frame(1)
            right_frame = frames.get_fisheye_frame(2)
            if left_frame is None or right_frame is None:
                return None, None, None

            left_img  = np.array(left_frame.get_data(),  copy=True)
            right_img = np.array(right_frame.get_data(), copy=True)

            # Undistort
            left_undistorted = cv2.remap(left_img, self.left_map1, self.left_map2, interpolation=cv2.INTER_LINEAR)
            right_undistorted = cv2.remap(right_img, self.right_map1, self.right_map2, interpolation=cv2.INTER_LINEAR)

            return T_4x4@rot@rot2, left_undistorted, right_undistorted
        else:
            return T_4x4@rot@rot2, None, None


if __name__ == '__main__':
    # teleop t265 serial 929122110141
    t265_teleop = T265_streaming(serial_number="929122111689",enable_fisheye=True)
    t265_ontool = T265_streaming(serial_number="929122110141",enable_fisheye=True) # knife
    # t265_ontool = T265_streaming(serial_number="929122111169",enable_fisheye=True) # brushs
    align_ready = False
    while True:
        pose_mat_teleop, left_teleop, right_teleop = t265_teleop._read()
        pose_mat_ontool, left_ontool, right_ontool = t265_ontool._read()
        print("Teleop t265 Pose:\n", pose_mat_teleop)
        print("Ontool t265 Pose:\n", pose_mat_ontool)
        # no need for image from teleop t265
        # if left_ontool is not None:
        #     cv2.imshow("Left Ontool Undistorted", left_ontool)
        #     cv2.imshow("Right Ontool Undistorted", right_ontool)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        R1= pose_mat_teleop
        R2 = pose_mat_ontool
        if not align_ready:
            A_R = R1 @ R2.T
            align_ready = True
        R2_aligned = A_R @ R2
        R_rel = R2_aligned @ R1.T
        rot_deg = t265_teleop._quat_angle_deg_from_R(R_rel)
        print(rot_deg)
