from demo_interface import run

# Custom functions
from replay import add_replay_function
from visualize import add_visualize_function

def handler_installer(state):
  add_replay_function(state)
  add_visualize_function(state)

if __name__ == '__main__':
  run(callback_func=handler_installer)