# #!/usr/bin/env python3
# import pyrealsense2 as rs
# import math as m
# import time
# import threading
# from datetime import datetime
# import numpy as np

# T265_SERIAL_1 = "929122111689"
# T265_SERIAL_2 = "929122110141"

# class TongAngleMonitor:
#     def __init__(self):
#         self.t265_1_data = {
#             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
#             'quaternion': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
#             'frame': 0, 'timestamp': None
#         }
#         self.t265_2_data = {
#             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
#             'quaternion': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
#             'frame': 0, 'timestamp': None
#         }
        
#         self.running = True
#         self.t265_1_ready = False
#         self.t265_2_ready = False
        
#         self.init_t265_cameras()
        
#     def init_t265_cameras(self):
#         try:
#             self.pipe1 = rs.pipeline()
#             cfg1 = rs.config()
#             cfg1.enable_stream(rs.stream.pose)
#             cfg1.enable_device(T265_SERIAL_1)
#             self.pipe1.start(cfg1)
#             self.t265_1_ready = True
#             print(f'✓ T265 #1 initialized - Serial: {T265_SERIAL_1}')
#         except Exception as e:
#             print(f'✗ T265 #1 initialization failed: {e}')
#             self.t265_1_ready = False
        
#         try:
#             self.pipe2 = rs.pipeline()
#             cfg2 = rs.config()
#             cfg2.enable_stream(rs.stream.pose)
#             cfg2.enable_device(T265_SERIAL_2)
#             self.pipe2.start(cfg2)
#             self.t265_2_ready = True
#             print(f'✓ T265 #2 initialized - Serial: {T265_SERIAL_2}')
#         except Exception as e:
#             print(f'✗ T265 #2 initialization failed: {e}')
#             self.t265_2_ready = False
    
#     def quaternion_to_euler(self, w, x, y, z, sensor_id=1):
#         if sensor_id == 1:
#             x_adj, y_adj, z_adj = -z, x, -y
#         else:
#             x_adj, y_adj, z_adj = z, -x, y
        
#         pitch = -m.asin(2.0 * (x_adj*z_adj - w*y_adj)) * 180.0 / m.pi
#         roll = m.atan2(2.0 * (w*x_adj + y_adj*z_adj), w*w - x_adj*x_adj - y_adj*y_adj + z_adj*z_adj) * 180.0 / m.pi
#         yaw = m.atan2(2.0 * (w*z_adj + x_adj*y_adj), w*w + x_adj*x_adj - y_adj*y_adj - z_adj*z_adj) * 180.0 / m.pi
        
#         return roll, pitch, yaw
    
#     def get_axis_differences(self):
#         if not (self.t265_1_ready and self.t265_1_data['timestamp'] and
#                 self.t265_2_ready and self.t265_2_data['timestamp']):
#             return None
        
#         roll_diff = abs(self.t265_1_data['roll'] - self.t265_2_data['roll'])
#         pitch_diff = abs(self.t265_1_data['pitch'] - self.t265_2_data['pitch'])
#         yaw_diff = abs(self.t265_1_data['yaw'] - self.t265_2_data['yaw'])
        
#         roll_diff = min(roll_diff, 360 - roll_diff)
#         pitch_diff = min(pitch_diff, 360 - pitch_diff)
#         yaw_diff = min(yaw_diff, 360 - yaw_diff)
        
#         return {
#             'roll': roll_diff,
#             'pitch': pitch_diff,
#             'yaw': yaw_diff
#         }
    
#     def read_t265_data(self, sensor_id):
#         pipe = self.pipe1 if sensor_id == 1 else self.pipe2
#         ready = self.t265_1_ready if sensor_id == 1 else self.t265_2_ready
        
#         while self.running and ready:
#             try:
#                 frames = pipe.wait_for_frames(timeout_ms=100)
#                 pose = frames.get_pose_frame()
                
#                 if pose:
#                     data = pose.get_pose_data()
                    
#                     quaternion = {
#                         'w': data.rotation.w,
#                         'x': data.rotation.x,
#                         'y': data.rotation.y,
#                         'z': data.rotation.z
#                     }
                    
#                     roll, pitch, yaw = self.quaternion_to_euler(
#                         data.rotation.w, data.rotation.x,
#                         data.rotation.y, data.rotation.z,
#                         sensor_id=sensor_id
#                     )
                    
#                     sensor_data = {
#                         'roll': roll,
#                         'pitch': pitch,
#                         'yaw': yaw,
#                         'quaternion': quaternion,
#                         'frame': pose.frame_number,
#                         'timestamp': datetime.now()
#                     }
                    
#                     if sensor_id == 1:
#                         self.t265_1_data = sensor_data
#                     else:
#                         self.t265_2_data = sensor_data
                    
#             except Exception as e:
#                 if self.running:
#                     print(f"T265 #{sensor_id} read error: {e}")
#                 time.sleep(0.01)
    
#     def display_monitoring_data(self):
#         while self.running:
#             try:
#                 print("\033[2J\033[H", end="")
                
#                 differences = self.get_axis_differences()
                
#                 if differences is not None:
#                     print("="*60)
#                     print(f"{'IMU AXIS DIFFERENCES':^60}")
#                     print("="*60)
                    
#                     print(f"T265 #1: R={self.t265_1_data['roll']:>7.1f}° P={self.t265_1_data['pitch']:>7.1f}° Y={self.t265_1_data['yaw']:>7.1f}°")
#                     print(f"T265 #2: R={self.t265_2_data['roll']:>7.1f}° P={self.t265_2_data['pitch']:>7.1f}° Y={self.t265_2_data['yaw']:>7.1f}°")
#                     print("-"*60)
                    
#                     print(f"Roll Difference:  {differences['roll']:>8.2f}°")
#                     print(f"Pitch Difference: {differences['pitch']:>8.2f}°")
#                     print(f"Yaw Difference:   {differences['yaw']:>8.2f}°")
                    
#                 else:
#                     print("="*60)
#                     print("Waiting for sensor data...")
                
#                 print("="*60)
#                 print("Press Ctrl+C to stop")
                
#                 time.sleep(0.1)
                
#             except KeyboardInterrupt:
#                 break
    
#     def start(self):
#         threads = []
        
#         if self.t265_1_ready:
#             t265_1_thread = threading.Thread(target=self.read_t265_data, args=(1,), daemon=True)
#             t265_1_thread.start()
#             threads.append(t265_1_thread)
        
#         if self.t265_2_ready:
#             t265_2_thread = threading.Thread(target=self.read_t265_data, args=(2,), daemon=True)
#             t265_2_thread.start()
#             threads.append(t265_2_thread)
        
#         if not (self.t265_1_ready and self.t265_2_ready):
#             print("\n✗ ERROR: Both T265 cameras must be connected!")
#             print("Please check:")
#             print(f"  - T265 #1 (Serial: {T265_SERIAL_1}): {'OK' if self.t265_1_ready else 'FAILED'}")
#             print(f"  - T265 #2 (Serial: {T265_SERIAL_2}): {'OK' if self.t265_2_ready else 'FAILED'}")
#             return
        
#         print("\nWaiting for sensor data...")
#         time.sleep(2)
        
#         try:
#             self.display_monitoring_data()
#         except KeyboardInterrupt:
#             print("\nStopping...")
#         finally:
#             self.cleanup()
    
#     def cleanup(self):
#         self.running = False
        
#         if self.t265_1_ready:
#             try:
#                 self.pipe1.stop()
#                 print("T265 #1 stopped")
#             except:
#                 pass
        
#         if self.t265_2_ready:
#             try:
#                 self.pipe2.stop()
#                 print("T265 #2 stopped")
#             except:
#                 pass

# def main():
#     print(f"Current serials:")
#     print(f"  T265 #1: {T265_SERIAL_1}")
#     print(f"  T265 #2: {T265_SERIAL_2}")
#     print("\nPress Enter to continue...")
#     input()
    
#     monitor = TongAngleMonitor()
#     monitor.start()

# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
import pyrealsense2 as rs
import math as m
import time
import threading
from datetime import datetime
import numpy as np

T265_SERIAL_1 = "929122110141"
T265_SERIAL_2 = "929122111689"

class TongAngleMonitor:
    # -------------------- math helpers --------------------
    def _q_norm(self, q):
        w, x, y, z = q
        s = (w*w + x*x + y*y + z*z) ** 0.5
        if s == 0:
            return (1.0, 0.0, 0.0, 0.0)
        return (w/s, x/s, y/s, z/s)

    def _q_to_R(self, q):
        w, x, y, z = self._q_norm(q)
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
        ], dtype=float)

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
        return 2.0 * m.degrees(m.acos(w))

    def __init__(self):
        # Shared state for each sensor
        self.t265_1_data = {
            'quaternion': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
            'frame': 0, 'timestamp': None
        }
        self.t265_2_data = {
            'quaternion': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
            'frame': 0, 'timestamp': None
        }

        self.running = True
        self.t265_1_ready = False
        self.t265_2_ready = False

        # Alignment state: maps #2 world -> #1 world
        self.align_ready = False
        self.A_R = np.eye(3)

        self.data_lock = threading.Lock()

        self.init_t265_cameras()

    # -------------------- device init --------------------
    def init_t265_cameras(self):
        try:
            self.pipe1 = rs.pipeline()
            cfg1 = rs.config()
            cfg1.enable_stream(rs.stream.pose)
            cfg1.enable_device(T265_SERIAL_1)
            self.pipe1.start(cfg1)
            self.t265_1_ready = True
            print(f'✓ T265 #1 initialized - Serial: {T265_SERIAL_1}')
        except Exception as e:
            print(f'✗ T265 #1 initialization failed: {e}')
            self.t265_1_ready = False

        try:
            self.pipe2 = rs.pipeline()
            cfg2 = rs.config()
            cfg2.enable_stream(rs.stream.pose)
            cfg2.enable_device(T265_SERIAL_2)
            self.pipe2.start(cfg2)
            self.t265_2_ready = True
            print(f'✓ T265 #2 initialized - Serial: {T265_SERIAL_2}')
        except Exception as e:
            print(f'✗ T265 #2 initialization failed: {e}')
            self.t265_2_ready = False

    # -------------------- alignment & difference --------------------
    def _try_build_alignment(self):
        if self.align_ready:
            return
        if not (self.t265_1_data['timestamp'] and self.t265_2_data['timestamp']):
            return

        with self.data_lock:
            q1 = (self.t265_1_data['quaternion']['w'],
                  self.t265_1_data['quaternion']['x'],
                  self.t265_1_data['quaternion']['y'],
                  self.t265_1_data['quaternion']['z'])
            q2 = (self.t265_2_data['quaternion']['w'],
                  self.t265_2_data['quaternion']['x'],
                  self.t265_2_data['quaternion']['y'],
                  self.t265_2_data['quaternion']['z'])

        q1 = self._q_norm(q1)
        q2 = self._q_norm(q2)
        R1 = self._q_to_R(q1)
        R2 = self._q_to_R(q2)

        self.A_R = R1 @ R2.T
        self.align_ready = True
        print("✓ Alignment established (maps #2 into #1 frame)")

    def get_rotation_difference(self):
        if not (self.t265_1_ready and self.t265_2_ready and
                self.t265_1_data['timestamp'] and self.t265_2_data['timestamp']):
            return None

        self._try_build_alignment()
        if not self.align_ready:
            return None

        with self.data_lock:
            q1 = (self.t265_1_data['quaternion']['w'],
                  self.t265_1_data['quaternion']['x'],
                  self.t265_1_data['quaternion']['y'],
                  self.t265_1_data['quaternion']['z'])
            q2 = (self.t265_2_data['quaternion']['w'],
                  self.t265_2_data['quaternion']['x'],
                  self.t265_2_data['quaternion']['y'],
                  self.t265_2_data['quaternion']['z'])

        q1 = self._q_norm(q1)
        q2 = self._q_norm(q2)
        R1 = self._q_to_R(q1)
        R2 = self._q_to_R(q2)

        # Align #2 into #1
        R2_aligned = self.A_R @ R2
        R_rel = R2_aligned @ R1.T
        rot_deg = self._quat_angle_deg_from_R(R_rel)

        return rot_deg

    # -------------------- readers --------------------
    def read_t265_data(self, sensor_id):
        pipe = self.pipe1 if sensor_id == 1 else self.pipe2
        ready = self.t265_1_ready if sensor_id == 1 else self.t265_2_ready

        while self.running and ready:
            try:
                frames = pipe.wait_for_frames(timeout_ms=100)
                pose = frames.get_pose_frame()

                if pose:
                    data = pose.get_pose_data()
                    quaternion = {
                        'w': data.rotation.w,
                        'x': data.rotation.x,
                        'y': data.rotation.y,
                        'z': data.rotation.z
                    }
                    sensor_data = {
                        'quaternion': quaternion,
                        'frame': pose.frame_number,
                        'timestamp': datetime.now()
                    }
                    with self.data_lock:
                        if sensor_id == 1:
                            self.t265_1_data = sensor_data
                        else:
                            self.t265_2_data = sensor_data

            except Exception as e:
                if self.running:
                    print(f"T265 #{sensor_id} read error: {e}")
                time.sleep(0.01)

    # -------------------- UI --------------------
    def display_monitoring_data(self):
        while self.running:
            try:
                print("\033[2J\033[H", end="")  # clear screen
                rot_diff = self.get_rotation_difference()

                print("="*60)
                print(f"{'T265 ROTATION DIFFERENCE MONITOR':^60}")
                print("="*60)

                if rot_diff is not None:
                    print(f"Rotation Δ: {rot_diff:8.3f}°")
                else:
                    print("Waiting for alignment / sensor data...")

                print("="*60)
                print("Press Ctrl+C to stop")

                time.sleep(0.1)

            except KeyboardInterrupt:
                break

    # -------------------- lifecycle --------------------
    def start(self):
        threads = []

        if self.t265_1_ready:
            t1 = threading.Thread(target=self.read_t265_data, args=(1,), daemon=True)
            t1.start()
            threads.append(t1)

        if self.t265_2_ready:
            t2 = threading.Thread(target=self.read_t265_data, args=(2,), daemon=True)
            t2.start()
            threads.append(t2)

        if not (self.t265_1_ready and self.t265_2_ready):
            print("\n✗ ERROR: Both T265 cameras must be connected!")
            print(f"  - T265 #1 (Serial: {T265_SERIAL_1}): {'OK' if self.t265_1_ready else 'FAILED'}")
            print(f"  - T265 #2 (Serial: {T265_SERIAL_2}): {'OK' if self.t265_2_ready else 'FAILED'}")
            return

        print("\nWaiting for sensor data...")
        time.sleep(2)

        try:
            self.display_monitoring_data()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False

        if self.t265_1_ready:
            try:
                self.pipe1.stop()
                print("T265 #1 stopped")
            except:
                pass

        if self.t265_2_ready:
            try:
                self.pipe2.stop()
                print("T265 #2 stopped")
            except:
                pass


def main():
    print(f"Current serials:")
    print(f"  T265 #1: {T265_SERIAL_1}")
    print(f"  T265 #2: {T265_SERIAL_2}")
    print("\nPress Enter to continue...")
    input()

    monitor = TongAngleMonitor()
    monitor.start()


if __name__ == "__main__":
    main()
