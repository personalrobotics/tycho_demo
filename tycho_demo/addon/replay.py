from threading import Lock, Thread
import numpy as np
from tycho_env.utils import print_and_cr
import rospy
from sensor_msgs.msg import JointState

#######################################################################
# Replay
# ------------------------------------------------------------------------------
# Replay a recording rosbag and command the robot to repeat the movement
#######################################################################

REPLAY_INIT_THREAD = None
TARGET_POSE_TOPIC = '/joint_commands_replay'

def add_replay_function(state):
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
  state._replay_lock.acquire()
  cmd = list(state.last_replay_cmd)
  state._replay_lock.release()
  return cmd, [None] * 7

def __wait_for_replay(state, cur_time):
  return state.fix_position, [None] * 7

def launch_replay_subscriber(state):
  print_and_cr('Launch Ros Subscriber for replaying ... ')
  def callback(data):
    if state.mode == 'replay':
      state._replay_lock.acquire()
      state.last_replay_cmd = list(data.position)
      state._replay_lock.release()
  rospy.Subscriber(TARGET_POSE_TOPIC, JointState, callback, queue_size=1)

def init_replay_topic(state):
  replay_msg = rospy.wait_for_message(TARGET_POSE_TOPIC, JointState)
  state.last_replay_cmd = list(replay_msg.position)
  rospy.loginfo(
    'Established connection with target topic {}'.format(TARGET_POSE_TOPIC))
  if state.mode == 'wait_for_replay':
    state.mode = 'replay'
    print_and_cr('Entered replay mode')