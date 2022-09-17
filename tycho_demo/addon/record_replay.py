from threading import Lock, Thread
import numpy as np
from scipy.spatial.transform import Rotation as scipyR
import rospy

from tycho_env.utils import print_and_cr, construct_command, colors
from tycho_demo_ros.msg import ChopPose
from tycho_demo.utils import ChopPublisher, ChopPosePublisher
from tycho_demo.addon.recording import start_rosbag_recording, stop_rosbag_recording

import os

from subprocess import Popen, STDOUT
from time import strftime, localtime, sleep


#######################################################################
# Record Replay
# ------------------------------------------------------------------------------
# Replay a recording rosbag and command the robot to repeat the movement
# Specifically, replay chopstick pose
# and record the trajectory
#######################################################################

RECORD_REPLAY_INIT_THREAD = None
TARGET_POSE_TOPIC = '/Choppose_target2'
CHOPPOSE_PUBLISHER = ChopPosePublisher('/Choppose')
TARGET_CHOPPOSE_PUBLISHER = ChopPosePublisher('/Choppose_target')


# Singleton
FNULL = open(os.devnull, 'w')
ROSBAG_PROC = []
DEFAULT_CAMERAS = ['435','415_1','415_2']

def add_record_replay_function(state):
  state.handlers['a'] = _record_replay
  state.handlers['F'] = _failure_replay_label
  state.modes['record_replay'] = __record_replay
  state.modes['wait_for_record_replay'] = __wait_for_record_replay
  state._record_replay_lock = Lock()
  launch_replay_subscriber(state)

def _record_replay(key, state):
  state.lock()
  global RECORD_REPLAY_INIT_THREAD
  if (state.mode != 'record_replay' and
      (RECORD_REPLAY_INIT_THREAD is None or not RECORD_REPLAY_INIT_THREAD.isAlive())):
    print_and_cr("Entering record_replay mode ... initializing ...")
    print_and_cr("Please remap rostopic with: ")
    print_and_cr("rosbag play 22-07-19-23-35-32-pose.bag --topics /Choppose_target /Choppose_target:=/Choppose_target2")
    state.mode = 'wait_for_record_replay'
    state.fix_position = list(state.current_position)
    RECORD_REPLAY_INIT_THREAD = Thread(target=init_record_replay_topic, args=(state,))
    RECORD_REPLAY_INIT_THREAD.start()
  state.unlock()

  if state.rosbag_recording_to: # Stop recording if it has been running
    stop_rosbag_recording()
    state.rosbag_recording_to = False
  else:                         # Start recording
    if not os.path.exists(os.path.join(state.save_record_folder,'replay')):
      os.mkdir(os.path.join(state.save_record_folder,'replay'))

    rosbag_recording_to = os.path.join(
      state.save_record_folder,'replay',
      strftime('%y-%m-%d-%H-%M-%S', localtime()))
    state.rosbag_recording_to = True
    state.last_rosbag = rosbag_recording_to


    while rospy.wait_for_message(TARGET_POSE_TOPIC, ChopPose):
      break
    print_and_cr("start recording")
    start_rosbag_recording(rosbag_recording_to, state.ros_record_topics)

def __record_replay(state, cur_time):
  CHOPPOSE_PUBLISHER.update(state.ee_pose, state.current_position[-1])
  target_transform = np.zeros((3,4))
  state._record_replay_lock.acquire()
  target_open = state.last_replay_open
  _p = state.last_replay_pose.position
  _q = state.last_replay_pose.orientation
  target_transform[0:3, 3] = [_p.x, _p.y, _p.z]
  target_transform[0:3,0:3] = scipyR.from_quat([_q.x, _q.y, _q.z, _q.w]).as_matrix()

  TARGET_CHOPPOSE_PUBLISHER.update(target_transform, target_open)

  state._record_replay_lock.release()
  pos_cmd = construct_command(state.arm, state.current_position,
                           target_transformation=target_transform, target_open=target_open)
  return pos_cmd, [None] * 7

def __wait_for_record_replay(state, cur_time):
  return state.fix_position, [None] * 7

def launch_replay_subscriber(state):
  print_and_cr('Launch Ros Subscriber for replaying ... ')
  def callback(data):
    if state.mode == 'record_replay':
      state._record_replay_lock.acquire()
      state.last_replay_pose = data.pose
      state.last_replay_open = data.open
      state._record_replay_lock.release()
  rospy.Subscriber(TARGET_POSE_TOPIC, ChopPose, callback, queue_size=1)

def init_record_replay_topic(state):
  replay_msg = rospy.wait_for_message(TARGET_POSE_TOPIC, ChopPose)
  rospy.loginfo(
    'Established connection with target topic {}'.format(TARGET_POSE_TOPIC))
  state._record_replay_lock.acquire()
  state.last_replay_pose = replay_msg.pose
  state.last_replay_open = replay_msg.open
  state._record_replay_lock.release()
  if state.mode == 'wait_for_record_replay':
    state.mode = 'record_replay'
    print_and_cr('Entered record replay mode')

def _failure_replay_label(key, state):
  if state.rosbag_recording_to: # Stop recording
    stop_rosbag_recording()
    state.rosbag_recording_to = False
  if state.last_rosbag is not None:
    label_failure_demo(state.last_rosbag)

def label_failure_demo(rosbag_recording_to, cameras=DEFAULT_CAMERAS):
  print_and_cr(colors.bg.red + 'Re-label rosbag recording' + colors.reset)
  list_fn = [rosbag_recording_to+'-pose.bag'] + [
    rosbag_recording_to+'-camera_'+_camera+'.bag' for _camera in cameras]
  for fn in list_fn:
    while not os.path.isfile(fn):
      sleep(0.05)
    split_fn = fn.split('/')
    tmp = split_fn[-1]
    tmp = tmp.split('-')
    tmp[-1] = 'fail-'+tmp[-1]
    split_fn[-1] = '-'.join(tmp)
    cmd = "mv {} {}".format(fn, '/'.join(split_fn))
    os.system(cmd)