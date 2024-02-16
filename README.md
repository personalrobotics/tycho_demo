# Tycho Demo

The base demo interface for our chopsticks robot, Tycho (Teach Your CHOpsticks).

## Installation

You need to install both (1) through ros catkin build, `catkin build tycho_demo_ros` and (2) through pip `pip install -e .`

## Getting Started

```bash
$ ./launch/start_demo.sh
$ python src/launch_demo.py
```

On the command line, hit 'h' for help.

You can try grabbing (closing the end effector) by pressing 'g'.

You can try out the basic step function 's' or swing function 'x'.

You can import the `run_demo` function for your downstream demo needs.