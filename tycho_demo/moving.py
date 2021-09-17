#######################################################################
# Moving
# ------------------------------------------------------------------------------
# Move the robot to a preset starting location (ignoring collision etc)
# - Moving
#   Move quickly (3 seconds)
# - Slow Moving
#   Move slowly (7 seconds)
#######################################################################

from functools import partial
import hebi
from tycho_env.utils import MOVING_POSITION, CHOPSTICK_CLOSE, CHOPSTICK_OPEN, print_and_cr
import numpy as np
from time import time
from functools import partial

SLOW_MOVING_KEY = "M"
FAST_MOVING_KEY = "m"
MOVING_MODE = "moving"

OPEN_LIMIT = CHOPSTICK_OPEN - 0.02
CLOSE_LIMIT =  CHOPSTICK_CLOSE + 0.02

def add_moving_function(state):
  state.handlers[FAST_MOVING_KEY] = state.handlers[SLOW_MOVING_KEY] = _move
  state.modes[MOVING_MODE] = __move

def do_move(state, moving_positions, total_time, return_mode=None):
  state.lock()
  state.trajectory = None
  state.moving_positions = moving_positions
  state.mode = MOVING_MODE
  state.command_smoother.reset()
  state.return_mode = return_mode
  state.per_step_time = total_time / len(moving_positions)
  state.unlock()

def _move(key, state):
  print_and_cr('Move to a predefined sets of positions')
  do_move(state, [MOVING_POSITION], 7.0 if key == SLOW_MOVING_KEY else 3.0)

def __move(state, cur_time):
  if state.trajectory is None:
    state.trajectory_start = cur_time
    state.trajectory = create_moving_trajectory(
	    state.current_position, state.moving_positions, per_step_time=state.per_step_time)
  elapse_time = cur_time - state.trajectory_start
  if elapse_time > state.trajectory.duration:
    # allows other modes programmatic access to moving mode
    if state.return_mode and elapse_time >= 1.1 * state.trajectory.duration:
      state.mode = state.return_mode
    return state.moving_positions[-1], [None] * 7
  pos, _, _ = state.trajectory.get_state(elapse_time)
  return list(pos), [None] * 7

def create_moving_trajectory(cur_positions, _positions, per_step_time=3.0):
  num_points = len(_positions) + 3
  time_span = per_step_time * len(_positions)
  time_vector = np.empty(num_points, dtype=np.float64)
  positions = np.empty((7, num_points), dtype=np.float64)
  velocities = np.zeros_like(positions)

  time_vector[0] = 0
  time_vector[1:-1] = 0.2 + np.linspace(0, time_span, num=num_points-2)
  time_vector[-1] = 0.2 + time_span + 0.05
  positions[:,0] = cur_positions
  positions[:,1] = cur_positions
  force_open = positions[-1, 1] + 0.02
  positions[-1,1] = np.clip(force_open, OPEN_LIMIT, CLOSE_LIMIT)
  # ? why does the generated trajectory makes the last joint move a lot?
  for i in range(len(_positions)):
    positions[:,i+2] = _positions[i]
  positions[:,-1] = _positions[-1]
  velocities[:, 1:-1] = np.nan


  return hebi.trajectory.create_trajectory(time_vector, positions, velocities)



