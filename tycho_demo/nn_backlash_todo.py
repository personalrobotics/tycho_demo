# Residual estimator import
import torch
from tycho_env import ResEstimator
from tycho_env.utils import get_res_estimator_path

setup_nn_residual(state)
state.use_nn_backlash = False
handlers['o'] = _toggle_nn_backlash

def _toggle_nn_backlash(key, state):
  state.lock()
  if state.res_estimator is not None:
    state.use_nn_backlash = not state.use_nn_backlash
    state.uncorrected_pos = None
    print_and_cr("NN backlash is %s" %
                ("on" if state.use_nn_backlash else "off"))
    state.unlock()
  else:
    print_and_cr("Cannot turn on NN backlash: no model loaded")

# in command_proc
    if state.use_nn_backlash:
      state.uncorrected_pos = state.current_position.copy() + OFFSET_JOINTS
      state.current_position[:6] += \
          state.res_estimator.predict(state.current_position[0:6])

#

def setup_nn_residual(state):
  model_path = get_res_estimator_path()
  print_and_cr("Loading residual estimator from %s" % model_path)
  try:
    device = torch.device("cpu") # TODO check if GPU can speed up
    state.res_estimator = ResEstimator.static_load(model_path, device)
    state.res_estimator.eval()
  except:
    print_and_cr("Cannot load residual estimator model")
