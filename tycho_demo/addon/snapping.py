#######################################################################
# Snaping
# ------------------------------------------------------------------------------
# Snap the robot to a fixed set of joint position (ignoring collision!)
# - 'm' to rapidly move (3 seconds)
# - 'M' to slowly move (7 seconds)
#######################################################################

from functools import partial
import hebi
from tycho_env.utils import MOVING_POSITION, CHOPSTICK_CLOSE, CHOPSTICK_OPEN, CHOPSTICK_PARALLEL, print_and_cr
import numpy as np
from time import time
from functools import partial

FLAT_MOVING_POS = [-1.549781576050989, 1.7630800247683762, 2.156477754791279, 0.39334130278068974, 1.592408334181594, 0.001258663470587632, -0.437]

SLOW_MOVING_KEY = "M"
FAST_MOVING_KEY = "m"
FLAT_MOVING_KEY = "j"
MOVING_MODE = "moving"

OPEN_LIMIT = CHOPSTICK_OPEN - 0.02
CLOSE_LIMIT =  CHOPSTICK_CLOSE + 0.02

def add_snapping_function(state):
  state.handlers[FAST_MOVING_KEY] = state.handlers[SLOW_MOVING_KEY] = _move
  state.handlers[FLAT_MOVING_KEY] = _flat_move
  state.modes[MOVING_MODE] = __move

def do_snapping(state, moving_positions, total_time, return_mode=None):
  # Utility function that makes the robot go through a set of fixed keypoints
  state.lock()
  state.trajectory = None
  state.moving_positions = moving_positions
  state.mode = MOVING_MODE
  state.command_smoother.reset()
  state.return_mode = return_mode
  state.per_step_time = total_time / len(moving_positions)
  state.unlock()

def _flat_move(key, state):
  print_and_cr('Move to a predefined position ' + np.array2string(np.array(FLAT_MOVING_POS), precision=3))
  do_snapping(state, [FLAT_MOVING_POS], 3.0)

def _move(key, state):
  print_and_cr('Move to a predefined position ' + np.array2string(np.array(MOVING_POSITION), precision=3))
  do_snapping(state, [MOVING_POSITION], 7.0 if key == SLOW_MOVING_KEY else 3.0)

def __move(state, cur_time):
  if state.trajectory is None:
    print_and_cr('Start moving / snapping')
    state.lock()
    state.trajectory_start = cur_time
    state.trajectory = create_moving_trajectory(
	    state.current_position, state.moving_positions, per_step_time=state.per_step_time)
    state.unlock()

  elapse_time = cur_time - state.trajectory_start
  if elapse_time > state.trajectory.duration:
    # allows other modes programmatic access to moving mode
    if state.return_mode and elapse_time >= 1.1 * state.trajectory.duration:
      print_and_cr('Finish moving / snapping and return to ' + state.return_mode)
      state.lock()
      state.mode = state.return_mode
      state.unlock()
    return state.moving_positions[-1], [None] * 7
  pos, _, _ = state.trajectory.get_state(elapse_time)
  return list(pos), [None] * 7

def create_moving_trajectory(cur_positions, _positions, per_step_time=3.0):
  assert len(_positions) > 0
  dim = np.asarray(_positions).shape[-1]
  num_points = len(_positions) + 3
  time_span = per_step_time * len(_positions)
  time_vector = np.empty(num_points, dtype=np.float64)
  positions = np.empty((dim, num_points), dtype=np.float64)
  velocities = np.zeros_like(positions)

  time_vector[0] = 0
  time_vector[1:-1] = 0.2 + np.linspace(0, time_span, num=num_points-2)
  time_vector[-1] = 0.2 + time_span + 0.05
  positions[:,0] = cur_positions
  positions[:,1] = cur_positions
  force_open = positions[-1, 1] + 0.15 if positions[-1,1] < -0.3 else positions[-1, 1] + 0.02
  force_open = np.clip(force_open, CLOSE_LIMIT, OPEN_LIMIT)
  positions[-1,1] = force_open
  positions[-1,0] = force_open
  # ? why does the generated trajectory makes the last joint move a lot?
  for i in range(len(_positions)):
    positions[:,i+2] = _positions[i]
  positions[:,-1] = _positions[-1]
  velocities[:, 1:-1] = np.nan

  return hebi.trajectory.create_trajectory(time_vector, positions, velocities)
