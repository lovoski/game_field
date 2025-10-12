"""
Trajectory sample rate 0.5 Hz
"""
import os
import json
import numpy as np
import utils.bvh as bvh
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import CubicHermiteSpline

def interpolate_trajectory_spring(points, target_fps = 60):
  points_frametime = 0.5
  interp_points = int(np.round(points_frametime * target_fps)) - 1
  nframes = points.shape[0]
  dx, dy = np.gradient(points[:,0]), np.gradient(points[:,1])
  velocity = np.zeros_like(points)
  velocity[:,0] = dx
  velocity[:,1] = dy
  lbd = np.log(2)/(0.05*np.log(np.e))

  x_history = []

  for i in range(1, nframes):
    x0 = points[i-1]
    v0 = velocity[i-1]
    xt = points[i]
    vt = velocity[i]

    for j in range(interp_points):
      dt = j/target_fps
      x = x0-xt
      v = v0-vt
      x_prev = x.copy()
      x = (x_prev+(v+lbd*x_prev)*dt)*np.exp(-lbd*dt)
      v = (v+lbd*x_prev)*np.exp(-lbd*dt)-lbd*x
      
      x_history.append(x+xt)

  x_history = np.array(x_history)
  plt.figure()
  plt.plot(x_history[:,0],x_history[:,1], '-o')
  plt.plot(points[:,0],points[:,1], '-o')
  plt.show()
  return x_history

def interpolate_trajectory_hermit(points, target_fps = 30):
  points_frametime = 0.5
  nframes = points.shape[0]
  x, y = points[:,0], points[:,1]
  t = np.arange(nframes) * points_frametime
  dx_dt = np.gradient(x, t)
  dy_dt = np.gradient(y, t)
  spline_x = CubicHermiteSpline(t, x, dx_dt)
  spline_y = CubicHermiteSpline(t, y, dy_dt)
  t_dense = np.linspace(t[0], t[-1], int(np.round(nframes*points_frametime*target_fps)))
  x_dense = spline_x(t_dense)
  y_dense = spline_y(t_dense)
  plt.figure()
  plt.plot(x_dense,y_dense, '-o')
  plt.plot(points[:,0],points[:,1], '-o')
  plt.show()
  return np.array([x_dense, y_dense]).transpose()

def extract_motion_traj(motion_dir):
  for filename in os.listdir(motion_dir):
    data = bvh.load(os.path.join(motion_dir, filename))
    grot, gpos = bvh.fk(data['rotations'], data['positions'], data['parents'])
    sim_pos = gpos[:, 0] * np.array([1, 0, 1])
    sim_pos = savgol_filter(sim_pos, 15, 3, axis=0, mode='interp')
    sim_dir = bvh.mul_vec(grot[:, 0], np.array([0, 0, 1])) * np.array([1, 0, 1])
    sim_dir /= np.linalg.norm(sim_dir, axis=1, keepdims=True)
    sim_dir = savgol_filter(sim_dir, 31, 3, axis=0, mode='interp')
    sim_dir /= np.linalg.norm(sim_dir, axis=1, keepdims=True)

def write_traj_txt(filepath, traj_data):
  traj_init_pos = traj_data[0].copy()
  traj_data -= traj_init_pos
  with open(filepath, 'w') as f:
    nframes = traj_data.shape[0]
    for i in range(nframes):
      f.write(f'{traj_data[i,0]:.3f} {0:.3f} {traj_data[i,1]:.3f}\n')

if __name__ == '__main__':
  base_dir = '/mnt/c/Users/10128/Downloads/batch_1/traj_sub'
  motion_dir = '/mnt/d/datasets/InterAct/bvh'
  filenames = os.listdir(base_dir)
  for filename in filenames:
    with open(os.path.join(base_dir, filename), 'r') as f:
      data = json.load(f)
      points = np.array(data['trajectory']) # (nframes, 2, points)
      states = data['states']
      traj_a = interpolate_trajectory_hermit(points[:, 0])
      write_traj_txt(os.path.join(base_dir, f'{filename}.traj_a.txt'), traj_a)
      traj_b = interpolate_trajectory_hermit(points[:, 1])
      write_traj_txt(os.path.join(base_dir, f'{filename}.traj_b.txt'), traj_b)
      
