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

from tycho_env.utils import print_and_cr, colors
from time import strftime, localtime, sleep
from threading import Lock, Thread
from queue import Queue
from functools import partial
from typing import Dict

import rospy
import rosbag
import rostopic


# Singleton
RECORD_LOCK = Lock()
MSG_QUEUE = Queue()
BAG_WRITERS: Dict[str, rosbag.Bag] = {}
ROSBAG_RECORDING = False
RECORDED_TOPICS = set()
WORKER_LAUNCHED = False

def subscriber_callback(bagname, topic, msg):
  if ROSBAG_RECORDING:
    MSG_QUEUE.put((bagname, topic, msg))

def recording_worker():
  while True:
    bagname, topic, msg = MSG_QUEUE.get()
    try:
      with RECORD_LOCK:
        try:
          writer: rosbag.Bag = BAG_WRITERS[bagname]
          writer.write(topic, msg)
        except KeyError:
          print_and_cr(f"ERR: No bag name: {bagname}")
    finally:
      MSG_QUEUE.task_done()

def launch_recording_subs(pose_topics, dict_topics):
  global RECORDED_TOPICS, WORKER_LAUNCHED
  unknown_topics = []
  for topic in pose_topics:
    if topic not in RECORDED_TOPICS:
      TopicType, _, _ = rostopic.get_topic_class(topic)
      if TopicType is not None:
        callback = partial(subscriber_callback, "pose", topic)
        rospy.Subscriber(topic, TopicType, callback, queue_size=10)
        RECORDED_TOPICS.add(topic)
      else:
        unknown_topics.append(topic)
  for bagname, topic in dict_topics.items():
    if topic not in RECORDED_TOPICS:
      TopicType, _, _ = rostopic.get_topic_class(topic)
      if TopicType is not None:
        callback = partial(subscriber_callback, bagname, topic)
        rospy.Subscriber(topic, TopicType, callback, queue_size=10)
        RECORDED_TOPICS.add(topic)
      else:
        unknown_topics.append(topic)
  if not WORKER_LAUNCHED:
    Thread(target=recording_worker, daemon=True).start()
    WORKER_LAUNCHED = True
  print_and_cr(f"Following topics will not be recorded: {', '.join(unknown_topics)}")

def add_recording_function(state):
  state.handlers['r'] = _record
  state.handlers['D'] = _delete_recording
  state.handlers['R'] = _count_recording
  state.handlers['f'] = _relabel_failure_recording
  state.rosbag_recording_to = False
  state.ros_record_topics = [
      '/joint_states', '/joint_commands',
      '/MocapPointArray',
      '/Choppose', '/Choppose_target',
      '/Ball/point', '/target/pose',
      '/R0/point', '/R1/point',
      '/R2/point',
      ]
  state.ros_record_dicts = {
    'camera_kinect': '/azcam_front/rgb/image_raw/compressed'
  }
  state.onclose.append(_stop_recording_on_quit)

  if 'save_record_folder' in state.params:
    state.save_record_folder = state.params['save_record_folder']
  else:
    dir_path = os.path.dirname(os.path.realpath(__file__))
    state.save_record_folder = os.path.join(dir_path, '..', 'recording')
  if not os.path.isdir(state.save_record_folder):
    print(f"Attempt to create the recording folder: {state.save_record_folder}")
    os.mkdir(state.save_record_folder)

def _record(key, state):
  toggle_rosbag_recording(state)

def _delete_recording(key, state):
  if state.rosbag_recording_to: # Stop recording
    stop_rosbag_recording(state.ros_record_dicts)
    state.rosbag_recording_to = False
  if state.last_rosbag is not None:
    delete_recording(state.last_rosbag, state.ros_record_dicts)
    state.last_rosbag = None

def _count_recording(key, state):
  number_of_recordings = len( [f for f in os.listdir(state.save_record_folder)
    if f.endswith('-pose.bag')])
  print("Number of recordings:", number_of_recordings)

def _stop_recording_on_quit(state):
  if state.rosbag_recording_to:
    stop_rosbag_recording(state.ros_record_dicts)

def _relabel_failure_recording(key, state):
  if state.rosbag_recording_to: # Stop recording
    stop_rosbag_recording(state.ros_record_dicts)
    state.rosbag_recording_to = False
  if state.last_rosbag is not None:
    label_failure_demo(state.last_rosbag, state.ros_record_dicts)

# ------------------------------------------------------------------------------

def start_rosbag_recording(record_prefix, pose_topics, dict_topics):
  print_and_cr(colors.bg.green + 'Recording to rosbag {}'.format(
    os.path.basename(record_prefix)))

  launch_recording_subs(pose_topics, dict_topics)

  def create_writer(bagname):
    return rosbag.Bag(f"{record_prefix}-{bagname}.bag", "w")

  with RECORD_LOCK:
    if len(BAG_WRITERS) != 0 or not MSG_QUEUE.empty():
      print_and_cr("ERR: Recording already in progress!")
      return

    BAG_WRITERS["pose"] = create_writer("pose")
    for bagname in dict_topics:
      BAG_WRITERS[bagname] = create_writer(bagname)
  global ROSBAG_RECORDING
  ROSBAG_RECORDING = True

def stop_rosbag_recording(dict_topics):
  print_and_cr(colors.bg.lightgrey + f'Stop rosbag recording' + colors.reset)

  global ROSBAG_RECORDING
  ROSBAG_RECORDING = False
  MSG_QUEUE.join()
  with RECORD_LOCK:
    for _, writer in BAG_WRITERS.items():
      writer.close()
    BAG_WRITERS.clear()

def delete_recording(rosbag_recording_to, dict_topics):
  print_and_cr(colors.bg.red + 'Deleting rosbag recording' + colors.reset)
  list_fn = [rosbag_recording_to+'-pose.bag'] + [
    f'{rosbag_recording_to}-{_topic}.bag' for _topic in dict_topics.keys()]
  for fn in list_fn:
    while not os.path.isfile(fn):
      sleep(0.05)
    os.remove(fn)

def label_failure_demo(rosbag_recording_to, dict_topics):
  print_and_cr(colors.bg.red + 'Re-label rosbag recording' + colors.reset)
  list_fn = [rosbag_recording_to+'-pose.bag'] + [
    f'{rosbag_recording_to}-{_topic}.bag' for _topic in dict_topics.keys()]
  for fn in list_fn:
    while not os.path.isfile(fn):
      sleep(0.05)
    split_fn = fn.split('/')
    new_fn = split_fn[-1].split('-')
    new_fn[-1] = 'fail-'+new_fn[-1]
    split_fn[-1] = '-'.join(new_fn)
    cmd = "mv {} {}".format(fn, '/'.join(split_fn))
    os.system(cmd)

def set_rosbag_recording(state, enabled: bool):
  if enabled != bool(state.rosbag_recording_to):
    toggle_rosbag_recording(state)

def toggle_rosbag_recording(state):
  if state.rosbag_recording_to: # Stop recording if it has been running
    stop_rosbag_recording(state.ros_record_dicts)
    state.rosbag_recording_to = False
  else:                         # Start recording
    rosbag_recording_to = os.path.join(
      state.save_record_folder,
      strftime('%y-%m-%d-%H-%M-%S', localtime()))
    state.rosbag_recording_to = True
    state.last_rosbag = rosbag_recording_to
    start_rosbag_recording(rosbag_recording_to,
      state.ros_record_topics,
      state.ros_record_dicts)
