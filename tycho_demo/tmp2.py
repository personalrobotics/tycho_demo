

def create_moving_trajectory2(cur_positions, _positions, per_step_time=3.0):
  # import pdb;pdb.set_trace()
  num_points = len(_positions) + 3
  time_span = per_step_time * len(_positions)
  time_vector = np.empty(num_points, dtype=np.float64)
  positions = np.empty((7, num_points), dtype=np.float64)
  velocities = np.zeros_like(positions)

  time_vector[0] = 0
  time_vector[1:-1] = 0.2 + np.linspace(0, time_span, num=num_points-2)
  time_vector[-1] = 0.2 + time_span + 0.5
  positions[:,0] = cur_positions
  positions[:,1] = cur_positions
  force_open = positions[-1, 1] + 0.02
  # positions[-1,1] = np.clip(force_open, OPEN_LIMIT, CLOSE_LIMIT)
  positions[-1,0] = -0.24
  positions[-1,1] = -0.24
  # ? why does the generated trajectory makes the last joint move a lot?
  for i in range(len(_positions)):
    positions[:,i+2] = _positions[i]
  positions[:,-1] = _positions[-1]
  velocities[:, 1:-1] = np.nan
  positions[-1,-1] = -0.23
  # print_and_cr(f'time:{time_vector}')
  # print_and_cr(f'positions:{positions}')
  return hebi.trajectory.create_trajectory(time_vector, positions, velocities)