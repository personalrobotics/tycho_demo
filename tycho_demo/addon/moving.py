#######################################################################
# Safe Move
# ------------------------------------------------------------------------------
# Move the robot to a preset starting location
# (using a RRT planner that checks for self-collision)
# (Not recommended for use right now becuase the RRT plan is not smooth)
# - Moving
#   Move quickly (4 seconds)
# - Slow Moving
#   Move slowly (7 seconds)
#######################################################################

from tycho_env import TychoRRT
from tycho_env.utils import MOVING_POSITION, print_and_cr

from .snapping import do_snapping

SLOW_MOVING_KEY = "N"
FAST_MOVING_KEY = "n"

def add_moving_function(state):
    state.handlers[FAST_MOVING_KEY] = state.handlers[SLOW_MOVING_KEY] = _move
    state.joint_rrt = TychoRRT()

def do_move(state, target_joint_pos, total_time, return_mode=None):
    moving_positions = state.joint_rrt.plan(state.current_position, target_joint_pos)
    do_snapping(state, moving_positions, total_time, return_mode=return_mode)

def _move(key, state):
    print_and_cr('Move to predefined pos, using RRT + collision checking')
    do_move(state, MOVING_POSITION, 7.0 if key == SLOW_MOVING_KEY else 4.0)
