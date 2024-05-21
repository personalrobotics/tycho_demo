import sys; sys.path.append("/usr/lib/python3/dist-packages")
import rospy

from tycho_demo import run_demo
from tycho_demo.addon import add_tuning_function,\
  add_visualize_function, add_replay_function

def handler_installer(state):
  add_visualize_function(state)
  add_replay_function(state)
  add_tuning_function(state)
  add_visualize_function(state)

if __name__ == '__main__':
  rospy.init_node("tycho_demo")
  run_demo(callback_func=handler_installer, cmd_freq=20)
