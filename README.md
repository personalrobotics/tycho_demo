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

## About `tycho_demo`

### Mode and commands

Depending on the value of `state.mode`, the demo interface will invoke a callback (specified by `state.modes`) to get the current command, consisting of a position and velocity setpoint.
To omit one of the setpoints (i.e. set position but not velocity setpoints) return a list of `None`s.

### Pre and Post command hooks

You can specify callbacks to be invoked before and after the mode is invoked to get the command, in `state.pre_command_hooks` and `state.post_command_hooks`.
- `pre_command_hooks` are invoked before calling the mode. These callbacks can be used to poll sensors and populate the `state.info` dict with relevant state information.
- `post_command_hooks` are invoked after calling the mode. These callbacks can be used to save the results of actions, i.e. for logging purposes.

Both `state.pre_command_hooks` and `state.post_command_hooks` are of type `Dict[str, List[Callable[[State], None]]]`, mapping the mode name to a list of callbacks.
This means that callbacks are invoked for the specified mode. To set a callback to be invoked for any mode, set the key as `"*"`, which is the wildcard.
