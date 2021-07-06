import sys; sys.path.append("/usr/lib/python3/dist-packages")
import rospy

from demo_interface import run
from replay import add_replay_function
from visualize import add_visualize_function

def handler_installer(state):
  add_replay_function(state)
  add_visualize_function(state)

if __name__ == '__main__':
  rospy.init_node("tycho_demo")
  run(callback_func=handler_installer)
