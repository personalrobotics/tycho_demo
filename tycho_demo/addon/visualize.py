from time import strftime, localtime
from multiprocessing import Process, Queue
from tycho_env.utils import get_fk_tips, get_transformation_matrix_from_quat, \
  print_and_cr, R_OPTITRACK2BASE
import numpy as np
from tycho_demo.utils import PointPublisher, TextPublisher
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PointStamped
import os

################################################################################
# Visualize
# ------------------------------------------------------------------------------
# - Swing
#    Fix the robot position. Press a key to select a joint. Swing that joint.
# - Step
#    Fix the robot position. Press a key to select a joint. Send a step command.
# - Rotate Base
#    Command Joint 0 at a constant velocity. Require pressing swing/step first.
###########################################################

CONTROLLER_SAVE_FN = 'controller_{}.csv'.format(
      strftime('%y-%m-%d-%H-%M', localtime()))

def add_visualize_function(state):
  state.handlers['c'] = _visualize_controller
  state.controller_save_file = None
  state.controller_queue = Queue()
  state.last_tuned_joint = None
  state.onclose.append(clean_controller_queue_on_quit)
  state.handlers['V'] = _visualize_tip
  state.handlers['J'] = _print_urdf_error

def _visualize_controller(key, state):
  state.lock()
  if state.controller_save_file:
    state.controller_queue.put('DONE')
    state.controller_save_file = False
    print_and_cr("Stop saving controller output")
  else:
    if os.path.isfile(CONTROLLER_SAVE_FN):
      os.remove(CONTROLLER_SAVE_FN)
    state.controller_save_file = True
    Process(
      target=save_fdbk_to_file,
      args=(CONTROLLER_SAVE_FN,
            state.controller_queue,
            state.last_tuned_joint)).start()
    print_and_cr("Saving controller output to {}".format(CONTROLLER_SAVE_FN))
  state.unlock()

def _visualize_tip(key, state):
  state.lock()
  state.tip_publisher = PointPublisher('/FK_tips')
  state.publishers.append(update_tip_publisher)
  init_tip_fk_error_publisher(state)
  state.unlock()

def _print_urdf_error(key, state):
  list_jp = list(state.current_position)
  gt_tip = get_fk_tips([list_jp])[0]
  arm_container_tip = np.array(state.ee_pose[0:3, 3]).reshape(3)
  print_and_cr('FK Func Tip {}\n URDFarm Tip {}\n'.format(
    gt_tip, arm_container_tip))
  print_and_cr('Diff = {}'.format(np.linalg.norm(gt_tip - arm_container_tip)))

def update_tip_publisher(state):
  fk_tip = np.array(state.ee_pose[0:3, 3]).reshape(3)
  state.tip_publisher.update(*fk_tip)

def init_tip_fk_error_publisher(state):
  state.error_publisher = TextPublisher('/FK_error')
  def callback(data):
    state.lock()
    if not state.quit:
      pos_in_optitrack = [data.point.x, data.point.y, data.point.z, 1]
      pos_in_optitrack = np.array(pos_in_optitrack).reshape(4,1)
      pos_in_robot = get_transformation_matrix_from_quat(R_OPTITRACK2BASE).dot(pos_in_optitrack)
      pos_in_robot = np.array(pos_in_robot[0:3]).reshape(3,1)
      list_jp = [state.current_position]
      fk_tip = get_fk_tips(list_jp)[0]
      tip_error = np.linalg.norm(pos_in_robot - fk_tip.reshape(3,1))

      text = 'FK error: {:2.3f} (mm)'.format(tip_error * 1000)
      color = (1,1,1)
      if hasattr(state, "uncorrected_pos") and state.uncorrected_pos is not None:
        uncorrected_err = pos_in_robot - get_fk_tips([state.uncorrected_pos])[0].reshape((3,1))
        uncorrected_err = np.linalg.norm(uncorrected_err)
        text += "\nFK error (uncorrected): {:2.3f}".format(uncorrected_err * 1000)
        improved = tip_error < uncorrected_err
        color = (0,1,0) if improved else (1,0,0)

      text_pos = np.array(pos_in_robot).reshape(-1)
      text_pos[0] -= 0.1
      text_pos[1] += 0.1
      state.error_publisher.update(text_pos, text, color=color)
    state.unlock()
  rospy.Subscriber('/Ball/point', PointStamped, callback, queue_size=10)

def clean_controller_queue_on_quit(state):
  state.controller_queue.close()
  state.controller_queue.join_thread()

def save_fdbk_to_file(fn, q, last_tuned_joint=None):
  file_handler = open(fn, 'a')
  while True:
    new_items = q.get(block=True)
    if new_items == 'DONE':
        file_handler.close()
        Process(target=viz_errors,args=(fn, last_tuned_joint),).start()
        return
    for item in new_items[:-1]:
        file_handler.write(np.array2string(item,
          precision=8, separator=' ', max_line_width=9999)[1:-1])
        file_handler.write(',')
    file_handler.write(np.array2string(new_items[-1],
      precision=8, separator=' ', max_line_width=9999)[1:-1])
    file_handler.write('\n')

def viz_errors(fn="controller.csv", last_tuned_joint=None):
  def str2nparray(_string):
    return np.fromstring(_string, dtype=np.float, sep=' ')
  pullData = open(fn,"r").read()
  dataArray = pullData.split('\n')
  xar = []
  yar = [[] for _ in range(7)]
  y2ar = [[] for _ in range(7)]
  vels = [[] for _ in range(7)]
  command_vels = [[] for _ in range(7)]
  for eachLine in dataArray[:-1]:
    try:
        timestamp_s,positions,velocities,efforts,positionCommands,velocityCommands,effortCommands,pwmCommands = eachLine.split(',')
        timestamp_s = np.mean(str2nparray(timestamp_s))
        positions = str2nparray(positions)
        positionCommands = str2nparray(positionCommands)
        velocities = str2nparray(velocities)
        velocityCommands = str2nparray(velocityCommands)
        if len(positionCommands) == 0:
          positionCommands = positions
        if len(velocityCommands) == 0:
          velocityCommands = velocities
        xar.append(timestamp_s)
        for _j in range(7):
          yar[_j].append(positions[_j])
          y2ar[_j].append(positionCommands[_j])
          vels[_j].append(velocities[_j])
          command_vels[_j].append(velocityCommands[_j])
    except:
        print('Skip a row when loading controller save file')
  viz_targets = [last_tuned_joint] if last_tuned_joint is not None else range(6)
  errors = np.abs(np.array(yar) - np.array(y2ar))

  for _j in viz_targets:
    max_idx = np.argmax(errors[_j, :]) # Find maximum error
    avg_err = np.average(errors[_j, :])
    max_err = errors[_j, max_idx]
    x_at_max_err = xar[max_idx]
    print_and_cr('Joint {} max error: {} at x={}\tavg err: {}\n'.format(_j, max_err, x_at_max_err, avg_err))
    plt.figure()
    plt.axvline(x=x_at_max_err, color='g')  #vertical line at max diff
    plt.plot(xar,yar[_j], color='red', label='pos', marker='o')
    plt.plot(xar,y2ar[_j], color='blue', label='cmd', marker='o')
    plt.title('Joint {}'.format(_j))
    plt.legend()

    plt.figure()
    plt.plot(xar, vels[_j], color='red', label='vel', marker='o')
    plt.plot(xar, command_vels[_j], color='blue', label='cmd', marker='o')
    plt.title('Joint {} vel'.format(_j))
    plt.legend()

  plt.show()