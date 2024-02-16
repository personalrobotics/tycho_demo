import numpy as np
from tycho_env.utils import CHOPSTICK_PARALLEL

GRAB_KEY = "g"
UNGRAB_KEY = "G"

def add_grabbing_function(state):
    state.handlers[GRAB_KEY] = _grab
    state.handlers[UNGRAB_KEY] = _grab
    state.modes["grab"] = __grab

def _grab(key, state):
    state.lock()
    state.mode = "grab"
    state.fix_position = np.array(state.current_position)
    state.fix_position[-1] = -0.57 if key == GRAB_KEY else CHOPSTICK_PARALLEL
    state.unlock()

def __grab(state, cur_time):
    return state.fix_position, [None] * 7
