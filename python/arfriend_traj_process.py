"""
Trajectory sample rate 0.5 Hz
"""
import os
import json
import numpy as np
import utils.bvh as bvh
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
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
  # plt.figure()
  # plt.plot(x_dense,y_dense, '-o')
  # plt.plot(points[:,0],points[:,1], '-o')
  # plt.show()
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

def format_traj(traj_data):
  sim_pos = np.zeros((traj_data.shape[0], 3))
  sim_pos[:, [0, 2]] = savgol_filter(traj_data, 31, 3, axis=0, mode='interp')
  sim_dir = np.diff(sim_pos, axis=0)
  
  # plt.figure()
  # plt.plot(np.linalg.norm(sim_dir, axis=1))
  # plt.show()
  
  sim_dir /= np.linalg.norm(sim_dir, axis=1, keepdims=True)
  sim_dir = savgol_filter(sim_dir, 61, 3, axis=0, mode='interp')
  sim_dir /= np.linalg.norm(sim_dir, axis=1, keepdims=True)
  sim_rot = np.zeros((sim_dir.shape[0], 4))
  ref_vec = np.array([[0, 0, 1]])
  __dir = np.zeros((sim_dir.shape[0],3))
  for i in range(sim_dir.shape[0]):
    target_vec = sim_dir[i].reshape(1, 3)
    # If target_vec is zero, use identity rotation
    if np.allclose(target_vec, 0):
      sim_rot[i] = np.array([1, 0, 0, 0])
    else:
      # Use align_vectors for robust rotation
      rot, _ = Rotation.align_vectors(target_vec, ref_vec)
      sim_rot[i, [1,2,3,0]] = rot.as_quat()
    __dir[i] = Rotation.from_quat(sim_rot[i, [3,0,1,2]]).apply(ref_vec)

  with open('aaa.txt', 'w') as f:
    for i in range(sim_dir.shape[0]):
      f.write(f'{sim_pos[i,0]:.6f} {sim_pos[i,1]:.6f} {sim_pos[i,2]:.6f} {sim_dir[i,0]:.6f} {sim_dir[i,1]:.6f} {sim_dir[i,2]:.6f}\n')

  sim_pos[:] -= sim_pos[0].copy()

  step = 1
  pos_idxs = np.arange(0, sim_dir.shape[0], step)
  arrow_pos = sim_pos[pos_idxs][:,[0,2]]  # shape (M, 2)
  arrow_vecs = sim_dir[pos_idxs][:,[0,2]]  # shape (M, 2)
  traj_scale = np.linalg.norm(sim_pos.max(axis=0) - sim_pos.min(axis=0))
  arrow_scale = 0.005 * traj_scale  # adjust factor as needed
  arrow_plot = arrow_vecs * arrow_scale
  plt.figure(figsize=(8, 8))
  plt.plot(sim_pos[:50, 0], sim_pos[:50, 2], '-o', label='trajectory', markersize=3)
  plt.quiver(arrow_pos[:50, 0], arrow_pos[:50, 1], arrow_plot[:50, 0], arrow_plot[:50, 1],
             angles='xy', scale_units='xy', scale=1, color='red', width=0.003, label='direction')
  plt.xlabel('X')
  plt.ylabel('Z')
  plt.axis('equal')  # keep aspect ratio so arrows are accurate
  plt.grid(True)
  plt.legend()
  plt.title('Trajectory (X-Z) with direction arrows')
  plt.show()
  
  return sim_rot, sim_pos

if __name__ == '__main__':
  # base_dir = '/mnt/c/Users/10128/Downloads/batch_1/traj_sub'
  # motion_dir = '/mnt/d/datasets/InterAct/bvh'
  # filenames = os.listdir(base_dir)
  # for filename in filenames:
  #   with open(os.path.join(base_dir, filename), 'r') as f:
  #     data = json.load(f)
  #     points = np.array(data['trajectory']) # (nframes, 2, points)
  #     states = data['states']
  #     traj_a = interpolate_trajectory_hermit(points[:, 0])
  #     # write_traj_txt(os.path.join(base_dir, f'{filename}.traj_a.txt'), traj_a)
  #     traj_b = interpolate_trajectory_hermit(points[:, 1])
  #     # write_traj_txt(os.path.join(base_dir, f'{filename}.traj_b.txt'), traj_b)
  #     a_rot, a_pos = format_traj(traj_a)
  #     b_rot, b_pos = format_traj(traj_b)
  #     np.savez(f'{filename}.npz', a_rot=a_rot, b_rot=b_rot, a_pos=a_pos, b_pos=b_pos)

  with open('t1.txt', 'r') as f:
    lines = f.readlines()
    points = []
    for line in lines:
      line = line.strip()
      if line == '':
        continue
      segs = line.split(' ')
      points.append(np.array([float(segs[0]),float(segs[1]),float(segs[2])]))
    traj_pos = savgol_filter(np.array(points, dtype=float), 31, 3, axis=0)[10:]
    traj_pos[:] -= traj_pos[0].copy()
    traj_vel = np.gradient(traj_pos, axis=0)
    traj_dir = traj_vel / np.linalg.norm(traj_vel, axis=1, keepdims=True)
    traj_rot = bvh.normalize(bvh.between(np.array([0, 0, 1]), traj_dir))

    with open('t1.obj', 'w') as wf:
      line = 'l '
      for i in range(traj_pos.shape[0]):
        line = f'{line} {i+1}'
        wf.write(f'v {traj_pos[i,0]} {traj_pos[i,1]} {traj_pos[i,2]}\n')
      wf.write(line)
    
    np.savez('t1.npz', pos=traj_pos, rot=traj_rot)
    
    plt.figure()
    plt.plot(traj_pos[:,0],traj_pos[:,2],'-o')
    plt.show()
    