################################################################################
# Tuning
# ------------------------------------------------------------------------------
# - Swing
#    Fix the robot. Press a key (0-7) to select a joint. Swing that joint.
# - Step
#    Fix the robot. Press a key (0-7) to select a joint. Send a step command.
# - Rotate Base
#    Command Joint 0 at a constant velocity. Require pressing swing/step first.
###########################################################

from tycho_env.utils import CHOPSTICK_CLOSE, CHOPSTICK_OPEN, print_and_cr
import numpy as np
from time import time

OPEN_LIMIT = CHOPSTICK_OPEN - 0.02
CLOSE_LIMIT =  CHOPSTICK_CLOSE + 0.02
CHOP_MIDDLE = (OPEN_LIMIT + CLOSE_LIMIT) / 2
CHOP_RANGE = (OPEN_LIMIT - CLOSE_LIMIT ) / 2

TUNING_MODES = {'step', 'swing', 'swing_vel'}

# singleton (not constant)
STEP_SIZE = 0.1

def add_tuning_function(state):
  state.handlers['x'] = state.handlers['s'] = _fix
  state.handlers['X'] = _fix
  state.handlers['0'] = state.handlers['1'] = state.handlers['2'] = \
    state.handlers['3'] = state.handlers['4'] = state.handlers['5'] = \
    state.handlers['6'] = state.handlers['7'] = _select_tuning_joint
  state.modes['step'] = __step
  state.modes['swing'] = __swing
  state.modes['swing_vel'] = __swing_vel
  state.handlers['b'] = _rotate_base
  state.modes['rotate'] = __rotate
  state.last_tuned_joint = None

def _rotate_base(key, state):
  state.lock()
  if state.mode == 'swing' or state.mode == 'step':
    print_and_cr('Entering determine base plane mode')
    state.tuning_start_time = time()
    assert(all(np.isclose(state.fix_position, np.array(state.current_position), atol=0.3)))
    state.base_vel = 0.3 if state.current_position[0] < np.pi else -0.3
    state.mode = 'rotate'
  else:
    print_and_cr('To rotate the base, first enter swing/step mode.')
  state.unlock()

def _fix(key, state):
  print_and_cr('Entering tuning mode')
  state.lock()
  state.fix_position = np.array(state.current_position)
  state.command_smoother.reset()
  if key == 'x':
    state.mode = 'swing'
  elif key == 'X':
    state.mode = 'swing_vel'
  elif key == 's':
    state.mode = 'step'
  else:
    print_and_cr(f"Unrecognized key: {key}")
    return
  state.tuning_joint = None
  state.unlock()

def _select_tuning_joint(key, state):
  state.lock()
  if state.mode in TUNING_MODES:
    if key == '7':
      state.last_tuned_joint = None
    else:
      state.tuning_joint = int(key)
      state.tuning_start_time = time()
      state.last_tuned_joint = int(key)
  state.unlock()

def __swing(state, cur_time):
  position = state.fix_position.copy()
  if state.tuning_joint is not None:
    if state.tuning_joint == 6:
      close_limit = CHOPSTICK_CLOSE + 0.02
      open_limit = CHOPSTICK_OPEN - 0.02
      position[state.tuning_joint] = CHOP_MIDDLE
      position[state.tuning_joint] += CHOP_RANGE * \
        np.sin(cur_time - state.tuning_start_time)
    else:
      position[state.tuning_joint] += np.pi * 0.2 * \
        np.sin(cur_time - state.tuning_start_time)
  return position, [None] * 7

def __swing_vel(state, cur_time):
  if state.tuning_joint is None:
    return state.fix_position, [None] * 7
  else:
    amplitude = 0.2 * np.pi
    elapsed = cur_time - state.tuning_start_time
    position = state.fix_position.copy()
    # position[state.tuning_joint] = None # fix every joint except for tuning joint
    position[state.tuning_joint] += amplitude - amplitude * np.cos(elapsed)
    velocity = [0] * 7
    velocity[state.tuning_joint] = amplitude * np.sin(elapsed)
    return position, velocity

def __step(state, cur_time):
  if state.tuning_joint is not None:
    global STEP_SIZE
    if state.tuning_joint == 6:
      target = min(state.fix_position[state.tuning_joint] + STEP_SIZE, OPEN_LIMIT)
      target = max(CLOSE_LIMIT, target)
      state.fix_position[state.tuning_joint] = target
    else:
      state.fix_position[state.tuning_joint] += STEP_SIZE
    STEP_SIZE = -STEP_SIZE
    state.tuning_joint = None
  return state.fix_position, [None] * 7

def __rotate(state, cur_time):
  position = state.fix_position.copy()
  position[0] += (cur_time - state.tuning_start_time) * state.base_vel
  return position, [None] * 7
