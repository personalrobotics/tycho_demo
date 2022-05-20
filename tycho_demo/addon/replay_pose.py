from threading import Lock, Thread
import numpy as np
from scipy.spatial.transform import Rotation as scipyR
import rospy

from tycho_env.utils import print_and_cr, construct_command
from tycho_demo_ros.msg import ChopPose

#######################################################################
# Replay
# ------------------------------------------------------------------------------
# Replay a recording rosbag and command the robot to repeat the movement
# Specifically, replay chopstick pose
#######################################################################

REPLAY_INIT_THREAD = None
TARGET_POSE_TOPIC = '/Choppose_target'

def add_replay_pose_function(state):
  state.handlers['p'] = _replay
  state.modes['replay'] = __replay
  state.modes['wait_for_replay'] = __wait_for_replay
  state._replay_lock = Lock()
  launch_replay_subscriber(state)

def _replay(key, state):
  state.lock()
  global REPLAY_INIT_THREAD
  if (state.mode != 'replay' and
      (REPLAY_INIT_THREAD is None or not REPLAY_INIT_THREAD.isAlive())):
    print_and_cr("Entering replay mode ... initializing ...")
    state.mode = 'wait_for_replay'
    state.fix_position = list(state.current_position)
    REPLAY_INIT_THREAD = Thread(target=init_replay_topic, args=(state,))
    REPLAY_INIT_THREAD.start()
  state.unlock()

def __replay(state, cur_time):
  target_transform = np.zeros((3,4))
  state._replay_lock.acquire()
  target_open = state.last_replay_open
  _p = state.last_replay_pose.position
  _q = state.last_replay_pose.orientation
  target_transform[0:3, 3] = [_p.x, _p.y, _p.z]
  target_transform[0:3,0:3] = scipyR.from_quat([_q.x, _q.y, _q.z, _q.w]).as_matrix()
  state._replay_lock.release()
  pos_cmd = construct_command(state.arm, state.current_position,
                           target_transformation=target_transform, target_open=target_open)
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
  state._replay_lock.release()
  if state.mode == 'wait_for_replay':
    state.mode = 'replay'
    print_and_cr('Entered replay mode')
