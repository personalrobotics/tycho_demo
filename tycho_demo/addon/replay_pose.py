from threading import Lock, Thread
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import rospy

from tycho_env.utils import print_and_cr, construct_command
from tycho_demo_ros.msg import ChopPose
from tycho_demo.addon import TychoSim
from tycho_demo.addon.MujocoIK import get_IK_from_mujoco, BOTTOM_TIP_DISTANCE

#######################################################################
# Replay
# ------------------------------------------------------------------------------
# Replay a recording rosbag and command the robot to repeat the movement
# Specifically, replay chopstick pose
#######################################################################

REPLAY_INIT_THREAD = None
TARGET_POSE_TOPIC = '/Choppose_target2'

target_points = np.zeros(6)
real_points = np.zeros(6)
translation_error_accumulator = np.zeros(1)
rot_error_accumulator = np.zeros(1)

def add_replay_pose_function(state):
  state.handlers['p'] = _replay
  state.modes['replay'] = __replay
  state.modes['wait_for_replay'] = __wait_for_replay
  state._replay_lock = Lock()
  launch_replay_subscriber(state)

def _replay(key, state):
  global target_points, real_points, translation_error_accumulator, rot_error_accumulator 
  state.lock()
  global REPLAY_INIT_THREAD
  if (state.mode != 'replay' and
      (REPLAY_INIT_THREAD is None or not REPLAY_INIT_THREAD.isAlive())):
    if not np.all(target_points == 0):
      plot_3d_graph(target_points, real_points, 'pose_graph_old.png')
      plot_accumulated_error(translation_error_accumulator, 'translation_error_old.png')
      plot_accumulated_error(rot_error_accumulator, 'rot_error_old.png')
      target_points = np.zeros(6)
      real_points = np.zeros(6)
      translation_error_accumulator = np.zeros(1)
      rot_error_accumulator = np.zeros(1)
    print_and_cr("Entering replay mode ... initializing ...")
    print_and_cr("Please remap rostopic with: ")
    print_and_cr("rosbag play 22-07-19-23-35-32-pose.bag --topics /Choppose_target /Choppose_target:=/Choppose_target2")
    state.mode = 'wait_for_replay'
    state.fix_position = list(state.current_position)
    REPLAY_INIT_THREAD = Thread(target=init_replay_topic, args=(state,))
    REPLAY_INIT_THREAD.start()
  state.unlock()

def __replay(state, cur_time):
  global target_points, real_points, translation_error_accumulator, rot_error_accumulator
  target_transformation = np.zeros((3,4))
  state._replay_lock.acquire()
  target_open = state.last_replay_open
  _p = state.last_replay_pose.position
  _q = state.last_replay_pose.orientation
  target_transformation[0:3, 3] = [_p.x, _p.y, _p.z]
  target_transformation[0:3,0:3] = R.from_quat([_q.x, _q.y, _q.z, _q.w]).as_matrix()

  # Grab the bottom tip of the leader chopsticks
  x_axis = target_transformation[0:3, 0]
  bottom_chop_middle_point = target_transformation[0:3, 3]
  bottom_tip = bottom_chop_middle_point + BOTTOM_TIP_DISTANCE * x_axis
  target_transformation[0:3, 3] = bottom_tip

  # Grab the rotation of the leader chopsticks and combine it into a single transform
  target_rot = R.from_matrix(target_transformation[0:3,0:3]).as_euler("xyz")

  target_transform = np.hstack((bottom_tip, target_rot))

  pos_cmd = construct_command(state.arm, state.current_position,
                            target_transformation=target_transformation,
                            target_open=target_open)

  # Get the old FK
  actual_transform = np.array(state.arm.get_FK_ee(pos_cmd))
  actual_transform = np.hstack((actual_transform[0:3, 3], R.from_matrix(actual_transform[0:3, 0:3]).as_euler("xyz")))
  translation_error = np.linalg.norm(np.absolute(bottom_tip - actual_transform[0:3]))
  rot_error = np.linalg.norm(np.absolute(target_rot - actual_transform[3:6]))
  # mujoco_pos_cmd = get_IK_from_mujoco(state.sim, state.current_position, target_transformation, target_open)

  # # Get the FK of the simulation
  # _, actual_transform = state.sim.get_comparable_transformations()
  print_and_cr(f"Target = {target_transform}")
  print_and_cr(f"Actual = {actual_transform}")

  translation_error = np.linalg.norm(bottom_tip - actual_transform[0:3])
  print_and_cr(f"Translation error = {translation_error}")
  rot_error = np.linalg.norm(target_rot - actual_transform[3:6])

  # For logging purposes
  if np.all(target_points != 0):
    # Logging XYZ
    target_points = np.vstack((target_points, target_transform))
    real_points = np.vstack((real_points, actual_transform))

    # Logging error accumulation
    translation_error_accumulator = np.vstack((translation_error_accumulator, translation_error))
    rot_error_accumulator = np.vstack((rot_error_accumulator, rot_error))
  else:
    target_points = target_transform
    real_points = actual_transform
    translation_error_accumulator = translation_error
    rot_error_accumulator = rot_error
  state._replay_lock.release()
  return pos_cmd, [None] * 7

def __wait_for_replay(state, cur_time):
  return state.fix_position, [None] * 7

def launch_replay_subscriber(state):
  print_and_cr('Launch Ros Subscriber for replaying ... ')
  def callback(data):
    if state.mode == 'replay':
      state._replay_lock.acquire()
      state.last_replay_pose = data.pose
      state.last_replay_open = data.open
      state._replay_lock.release()
  rospy.Subscriber(TARGET_POSE_TOPIC, ChopPose, callback, queue_size=1)

def init_replay_topic(state):
  replay_msg = rospy.wait_for_message(TARGET_POSE_TOPIC, ChopPose)
  rospy.loginfo(
    'Established connection with target topic {}'.format(TARGET_POSE_TOPIC))
  state._replay_lock.acquire()
  state.last_replay_pose = replay_msg.pose
  state.last_replay_open = replay_msg.open
  # _p = state.last_replay_pose.position
  # _q = state.last_replay_pose.orientation
  # target_transformation = np.zeros((3,4))
  # x_axis = target_transformation[0:3, 0]
  # bottom_chop_middle_point = target_transformation[0:3, 3]
  # bottom_tip = bottom_chop_middle_point + BOTTOM_TIP_DISTANCE * x_axis

  # target_transformation[0:3, 3] = bottom_tip
  # target_transformation[0:3,0:3] = R.from_quat([_q.x, _q.y, _q.z, _q.w]).as_matrix()
  # print_and_cr(f"Init target = {bottom_tip}")
  # state.sim.init_leader_position(target_transformation[0:3, 3], R.from_matrix(target_transformation[0:3, 0:3]).as_euler("xyz"))
  state._replay_lock.release()
  if state.mode == 'wait_for_replay':
    state.mode = 'replay'
    print_and_cr('Entered replay mode')

def plot_3d_graph(target_points, real_points, file_name):
  plt.clf()
  fig = plt.figure()
  ax = fig.add_subplot(projection='3d')

  # Skip just the first point, it tends to be an outlier due to sim pose initialization
  # Plotting target points
  ax.scatter(target_points[1:, 0], target_points[1:, 1], target_points[1:, 2], c='r', marker='o', label='Target Points')

  # Plotting real points
  ax.scatter(real_points[1:, 0], real_points[1:, 1], real_points[1:, 2], c='b', marker='^', label='Real Points')

  ax.set_xlabel('X-axis')
  ax.set_ylabel('Y-axis')
  ax.set_zlabel('Z-axis')

  plt.savefig(file_name)

def plot_accumulated_error(error_accumulator, file_name):
  plt.figure()
  plt.clf()
  # Generate x-axis values (timesteps)
  timesteps = np.arange(len(error_accumulator.flatten()))
  
  sums = np.cumsum(error_accumulator, axis=0)

  # Plotting
  plt.plot(timesteps, sums, marker='o')
  plt.xlabel('Timesteps')
  plt.ylabel('Accumulated Error')
  plt.title('Accumulated Error Over Time')
  plt.grid(True)

  plt.savefig(file_name)