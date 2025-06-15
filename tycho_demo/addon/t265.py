import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
import numpy as np

class T265_streaming():

    def __init__(self,):
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        # Build config object and request pose data
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.pose)

        # Start streaming with requested config
        self.pipe.start(self.cfg)
        print('Initialized.')
    # Implementation details


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
        return T_4x4@rot@rot2