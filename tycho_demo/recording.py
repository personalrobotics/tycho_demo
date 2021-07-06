#######################################################################
# Recording
# ------------------------------------------------------------------------------
# Record selected topics to a rosbag
# Which topic to record? is stored in state.ros_record_topics
# - 'r' to record
# - 'D' to delete the last record (can only delete one last record)
# - 'R' to count the number of recordings in the folder
#
# To change the topics to record, edit state.ros_record_topics before you press the record button
# It's a list of topic names.
#######################################################################

import os

import tycho_env
from tycho_env.utils import print_and_cr, colors
from subprocess import Popen, STDOUT
from multiprocessing import Process
from time import strftime, localtime, sleep

# Singleton
FNULL = open(os.devnull, 'w')
ROSBAG_PROC = []
DEFAULT_CAMERAS = ['435','415_1','415_2']

def add_recording_function(state):
  state.handlers['r'] = _record
  state.handlers['D'] = _delete_recording
  state.handlers['R'] = _count_recording
  state.rosbag_recording_to = False
  state.ros_record_topics = [
      '/joint_states', '/joint_commands',
      '/MocapPointArray',
      '/Choppose', '/Choppose_target',
      '/Ball/point',
      ]
  state.onclose.append(_stop_recording_on_quit)

  if hasattr(state, "save_record_folder") and state.save_record_folder is not None:
    SAVE_RECORD_FOLDER = state.save_record_folder
  else:
    dir_path = os.path.dirname(os.path.realpath(__file__))
    SAVE_RECORD_FOLDER = os.path.join(dir_path, '..', 'recording')
  if SAVE_RECORD_FOLDER != '' and not os.path.isdir(SAVE_RECORD_FOLDER):
    os.mkdir(SAVE_RECORD_FOLDER)

def _record(key, state):
  if state.rosbag_recording_to: # Stop recording if it has been running
    Process(target=stop_rosbag_recording).start()
    state.rosbag_recording_to = False
  else:                         # Start recording
    rosbag_recording_to = os.path.join(
      SAVE_RECORD_FOLDER,
      strftime('%y-%m-%d-%H-%M-%S', localtime()))
    state.rosbag_recording_to = True
    state.last_rosbag = rosbag_recording_to
    Process(target=start_rosbag_recording,
            args=(rosbag_recording_to, state.ros_record_topics)).start()

def _delete_recording(key, state):
  if state.rosbag_recording_to: # Stop recording
    stop_rosbag_recording()
    state.rosbag_recording_to = False
  if state.last_rosbag is not None:
    delete_recording(state.last_rosbag)
    state.last_rosbag = None

def _count_recording(key, state):
  number_of_recordings = len( [f for f in os.listdir(SAVE_RECORD_FOLDER)
    if f.endswith('-pose.bag')])
  print("Number of recordings:", number_of_recordings)

def _stop_recording_on_quit(state):
  if state.rosbag_recording_to:
    stop_rosbag_recording()

# ------------------------------------------------------------------------------

def start_rosbag_recording(record_prefix, pose_topics, cameras=DEFAULT_CAMERAS):
  print_and_cr(colors.bg.green + 'Recording to rosbag {}'.format(
    os.path.basename(record_prefix)))
  args1 = ['rosbag', 'record'] + pose_topics + \
          ['-O', record_prefix+'-pose.bag', '__name:=pose_bag']
  ROSBAG_PROC = [Popen(args1, stdout=FNULL, stderr=STDOUT)]
  for _camera in cameras:
      args2 = ['rosbag', 'record',
               '/'+_camera+'/color/image_raw/compressed',
               '-O', record_prefix+'-camera_'+_camera+'.bag',
               '__name:=camera_'+_camera+'_bag']
      ROSBAG_PROC.append(Popen(args2, stdout=FNULL, stderr=STDOUT))
      #args3 = ['rosbag', 'record',
      #         '/'+_camera+'/aligned_depth_to_color/image_raw/compressedDepth',
      #         '-O', record_prefix+'-depth_'+_camera+'.bag',
      #         '__name:=depth_'+_camera+'_bag']
      #rosbag_writer_cameras.append(Popen(args3, stdout=FNULL, stderr=STDOUT))

def stop_rosbag_recording(cameras=DEFAULT_CAMERAS):
  print_and_cr(colors.bg.lightgrey + 'Stop rosbag recording' + colors.reset)
  for p in ROSBAG_PROC:
    p.terminate()
  for p in ROSBAG_PROC:
    p.kill()
  args1 = ['rosnode', 'kill', '/pose_bag']
  rosbag_killer = Popen(args1, stdout=FNULL, stderr=STDOUT)
  for _camera in cameras:
      args2 = ['rosnode', 'kill', '/camera_'+_camera+'_bag']
      rosbag_killer = Popen(args2, stdout=FNULL, stderr=STDOUT)
      #args3 = ['rosnode', 'kill', '/depth_'+_camera+'_bag']
      #rosbag_killer = Popen(args3, stdout=FNULL, stderr=STDOUT)

def delete_recording(rosbag_recording_to, cameras=DEFAULT_CAMERAS):
  print_and_cr(colors.bg.red + 'Deleting rosbag recording' + colors.reset)
  list_fn = [rosbag_recording_to+'-pose.bag'] + [
    rosbag_recording_to+'-camera_'+_camera+'.bag' for _camera in cameras]
  for fn in list_fn:
    while not os.path.isfile(fn):
      sleep(0.05)
    os.remove(fn)