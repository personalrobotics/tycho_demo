from __future__ import print_function

import sys, os
from time import time, sleep
import numpy as np

from subprocess import Popen, STDOUT
from threading import Lock, Thread
from multiprocessing import Process

# Ros
import rospy
from sensor_msgs.msg import JointState

# HEBI
import hebi

from hebi_env import arm_container, Smoother, HebiController
from hebi_env.utils import (
  get_gains_path, load_gain,
  print_and_cr, colors,
  euler_angles_from_rotation_matrix)
# Import Constant
from hebi_env.utils import OFFSET_JOINTS, SMOOTHER_WINDOW_SIZE

# Local
from keyboard import getch
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path = [dir_path] + sys.path

# Custom functions
from moving import add_moving_function
from recording import add_recording_function
from tuning import add_tuning_function

# Feedback frequency (100 * x) Hz
FEEDBACK_FREQUENCY = 5

# Cameras
DEFAULT_CAMERAS = ['435', '415_1', '415_2']

#######################################################################
# Rospy publisher and subscriber
#######################################################################

joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)
joint_command_publisher = rospy.Publisher('/joint_commands', JointState, queue_size=1)

#######################################################################
# Store robot state and send command in a multi-thread safe way
#######################################################################

class State(object):
  def __init__(self, arm, gains_file):
    self.quit = False
    self.mode = 'idle'
    self._mute = True
    self.arm = arm
    self.gains_file = gains_file
    self.publishers = []
    self.rosbag_recording_to = None
    self.controller_save_file = None

    # For feedback
    self.current_position = np.empty(arm.dof_count, dtype=np.float64)
    self.current_velocity = np.empty(arm.dof_count, dtype=np.float64)
    self.current_effort = np.empty(arm.dof_count, dtype=np.float64)

    self.ee_pose = arm.get_FK_ee(self.current_position)
    self.cmd_pose = arm.get_FK_ee(self.current_position)

    # For command
    self.command_position = [None] * 7
    self.command_effort = [None] * 7

    # For threading safety
    self._mutex = Lock()

    # Misc functions
    self.print_state = False
    self.env_state = {}
    self._env_mutex = Lock()

  def lock(self):
    self._mutex.acquire()

  def unlock(self):
    self._mutex.release()

  def env_state_lock(self):
    self._env_mutex.acquire()

  def env_state_unlock(self):
    self._env_mutex.release()

  def _print_state_fn(self):
    print_and_cr('current_position')
    print(self.current_position)
    print_and_cr('End effector xyz rpy')
    pose_xyz_rpy = np.empty(6)
    pose_xyz_rpy[0:3] = self.ee_pose[0:3,3].reshape(3)
    pose_xyz_rpy[3:6] = euler_angles_from_rotation_matrix(self.ee_pose[0:3,0:3])
    print_and_cr(str(pose_xyz_rpy))
    print_and_cr('\r\n----\r\n')

#######################################################################
# Utility scripts
#######################################################################

def init_joint_state_msg(DOF=7):
  joint_state_msg = JointState()
  joint_name_list = ['HEBI/base/X8_9', 'HEBI/shoulder/X8_16', 'HEBI/elbow/X8_9', 'HEBI/wrist1/X5_1', 'HEBI/wrist2/X5_1', 'HEBI/wrist3/X5_1', 'HEBI/chopstick_actuator/X5_1']
  for i in range(DOF):
    joint_state_msg.name.append(joint_name_list[i])
    joint_state_msg.position.append(0.0)
    joint_state_msg.effort.append(0.0)
    joint_state_msg.velocity.append(0.0)
  return joint_state_msg

def init_robotarm():
  # initialize the arm
  controller_gains_xml_file = get_gains_path()
  directPWM_xml_file = get_gains_path('-directPWM')

  try:
    arm = arm_container.create_robot(dof=7)
    load_gain(arm.group, directPWM_xml_file)
  except:
    print("Unexpected error:", sys.exc_info()[0])
    print(colors.bg.red + "Cannot load the arm modules. Are all modules on?" + colors.reset)
    raise

  state = State(arm, controller_gains_xml_file)
  state.controller = HebiController(controller_gains_xml_file, False)
  return state, controller_gains_xml_file, directPWM_xml_file

#######################################################################
# Control loop that sends command to the arm
#######################################################################

def is_ik_jumping(state, command_pos):
  a = np.array(state.current_position)
  b = np.array(command_pos)
  threshold = np.array([0.3,0.3,0.5,0.8,0.5,0.5,10])
  if np.greater(np.abs(a-b), threshold).any():
    print_and_cr("IK Jumps \t" +
                 np.array2string(np.abs(a-b), precision=4, separator=',',
                      suppress_small=True))
    return True
  return False

def update_position_command(state, command_pos):
  if command_pos[0] is not None:
    state.command_position = np.array(command_pos)
  else:
    state.command_position = [None] * 7

# The following function, send_command, is not in use. Instead use send_command_controller
def send_command(state, command_pos, command_eff):
  hebi_command = hebi.GroupCommand(7)
  if command_pos[0] is not None:
    # Apply offset iff sending to the original hebi controller IF WE ARE ON POSITION CONTROL!
    hebi_command.position = np.array(command_pos) - OFFSET_JOINTS
  if command_eff[0] is not None:
    hebi_command.effort = command_eff
  state.arm.group.send_command(hebi_command)

def send_command_controller(state, timestamp=None):
  pwm = state.controller.gen_pwm(
    state.current_position, state.current_velocity, state.current_effort,
    state.command_position, [None] * 7, state.command_effort)
  command = hebi.GroupCommand(7)
  command.effort = np.array(pwm)
  state.arm.group.send_command(command)
  # TODO: at some point maybe abstract away the controller_queue object
  if state.controller_save_file:
    state.controller_queue.put((np.array(timestamp),
      np.array(state.current_position), np.array(state.current_velocity), np.array(state.current_effort),
      np.array(state.command_position), np.array(state.command_effort), np.array(pwm)))

def save_fdbk_to_file(controller_save_fn, q, last_tuned_joint=None):
    file_handler = open(controller_save_fn, 'a')
    while True:
        new_items = q.get(block=True)
        if new_items == 'DONE':
            file_handler.close()
            Process(target=viz_errors,args=(controller_save_fn, last_tuned_joint),).start() # UNCOMMENT THIS for gain tune
            return
        for item in new_items[:-1]:
            file_handler.write(np.array2string(item,
              precision=8, separator=' ', max_line_width=9999)[1:-1])
            file_handler.write(',')
        file_handler.write(np.array2string(new_items[-1],
          precision=8, separator=' ', max_line_width=9999)[1:-1])
        file_handler.write('\n')

def command_proc(state):
  group = state.arm.group
  group.feedback_frequency = 100.0 * FEEDBACK_FREQUENCY
  state.command_smoother = Smoother(7, SMOOTHER_WINDOW_SIZE)

  num_modules = group.size
  feedback = hebi.GroupFeedback(num_modules)

  joint_state_msg = init_joint_state_msg(num_modules)
  joint_command_msg = init_joint_state_msg(num_modules)

  counter = 0

  while not state.quit:
    if group.get_next_feedback(reuse_fbk=feedback) is None:
      print_and_cr('Did not receive feedback')
      state.lock()
      state.quit = True
      state.unlock()
      break

    state.lock()
    # Update feedback
    feedback.get_position(state.current_position)
    state.current_position += OFFSET_JOINTS
    feedback.get_velocity(state.current_velocity)
    feedback.get_effort(state.current_effort)
    state.ee_pose = state.arm.get_FK_ee(state.current_position)
    current_mode = state.mode
    cur_time = time()

    if not state._mute:
      state.command_effort = state.arm._get_grav_comp_efforts(state.current_position).copy()
      send_command_controller(state, feedback.hardware_receive_time)

    counter += 1
    if counter % FEEDBACK_FREQUENCY != 0:
      state.unlock()
      continue

    # Print state info
    if state.print_state:
      state._print_state_fn()
      state.print_state = False

    # Generating command
    assert current_mode in state.mode_keys
    command_pos = state.modes[current_mode](state, time())

    # Check for IK jump, apply smoother, and send out command
    if not state._mute:
      if np.all(np.array(command_pos) != None):
        if is_ik_jumping(state, command_pos):
          command_pos = state.command_smoother.get()
        else:
          state.command_smoother.append(command_pos)
          command_pos = np.array(state.command_smoother.get())
      update_position_command(state, command_pos)

    # Publish Joint State
    if not rospy.is_shutdown():
      if joint_state_msg:
        joint_state_msg.header.stamp = rospy.Time.now()
        for i in range(num_modules):
          joint_state_msg.position[i] = state.current_position[i]
          joint_state_msg.velocity[i] = state.current_velocity[i]
          joint_state_msg.effort[i] = state.current_effort[i]
        joint_state_publisher.publish(joint_state_msg)

      if joint_command_msg and command_pos[0] is not None:
        joint_command_msg.header.stamp = rospy.Time.now()
        for i in range(num_modules):
          joint_command_msg.position[i] = state.command_position[i]
          #joint_command_msg.velocity[i] = command.velocity[i]
          joint_command_msg.effort[i] = state.command_effort[i]
        joint_command_publisher.publish(joint_command_msg)

    # Update publisher
    for publisher in state.publishers:
      publisher(state)

    state.unlock()

###########################################################
# Key Press Handler
###########################################################

def _load_gain(key, state):
  state.lock()
  print_and_cr('Controller loading gains from {}'.format(state.gains_file))
  state.controller.load_gains(state.gains_file)
  state.unlock()

def _idle(key, state):
  state.lock()
  state.mode = 'idle'
  state.unlock()

def _mute(key, state):
  state.lock()
  state._mute = not state._mute
  state.controller.reset()
  state.unlock()

def _print_state(key, state):
  state.lock()
  state.print_state = True
  state.unlock()

# ------------------------------------------------------------------------------
# Modes Handler
# ------------------------------------------------------------------------------

def __idle(state, curr_time):
  return [None] * 7 # No position command in grav comp / idle

#######################################################################
# Main thread switches running mode by accepting keyboard command
#######################################################################

def run(demo_rosnode_name='hebi_demo', callback_func=None):
  state, _, _ = init_robotarm()

  # Basic demo functions
  handlers = {}
  modes = {}
  onclose = []

  handlers['L'] = handlers['l'] = _load_gain
  handlers['z'] = _idle
  handlers['Z'] = _mute
  handlers['v'] = _print_state

  modes['idle'] = __idle

  state.handlers = handlers
  state.modes = modes
  state.onclose = onclose

  # Examples of installing new functions
  add_moving_function(state)
  add_recording_function(state)
  add_tuning_function(state)

  # Caller install custom handlers
  if callback_func is not None:
    callback_func(state)

  state.handlers_keys = state.handlers.keys()
  state.mode_keys = state.modes.keys()

  rospy.init_node(demo_rosnode_name)

  # command is sent out via a separate thread
  cmd_thread = Thread(target=command_proc, name='Command Thread', args=(state,))
  cmd_thread.start()

  # this script is preserved for switching modes
  np.set_printoptions(suppress=True)
  print_and_cr("Press keys to invoke modes.\r\n")
  res = getch()

  while res != 'q' and not state.quit:
    print_and_cr('')
    if res in state.handlers_keys:
      state.handlers[res](res, state)
    sleep(0.01)
    res = getch()

  print_and_cr('Quitting...')
  state.lock()
  state.quit = True
  state.unlock()

  # Cleaning up
  for func in state.onclose:
    func(state)

if __name__ == '__main__':
  run()
