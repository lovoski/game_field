from scipy.spatial.transform import Rotation, Slerp
from utils.bvh_motion import Motion
import numpy as np

def batch_quat_to_so3(q):
  # x,y,z,w -> x,y,z
  half_theta = np.acos(q[:,3]).reshape(-1,1)
  u = q[:,:3]/np.sin(half_theta)
  return u*half_theta*2

def batch_so3_to_quat(a):
  theta = np.linalg.norm(a, axis=1).reshape(-1,1)
  q = np.zeros((a.shape[0],4))
  q[:,:3] = a/theta*np.sin(theta/2)
  q[:,3] = np.cos(theta/2).reshape(-1)
  return q

def inertial_blending(motion_a: Motion, motion_b: Motion, blending_frames=20, num_steps=20):
  nframes_a = motion_a.frame_num
  nframes_b = motion_b.frame_num
  assert nframes_b > blending_frames, "motion b should be longer than blending_frames"
  dt = motion_a.frametime
  step = dt/num_steps
  duration = dt*blending_frames
  decay_rate = np.log(32)/(duration*np.log(np.e))
  result = motion_a.copy()
  # make sure consistent translation
  offset = motion_b.positions[0,0]-motion_a.positions[-1,0]
  motion_b.positions[:,0] -= offset
  result.frame_num = nframes_a + nframes_b
  result.positions = np.concat([motion_a.get_joint_positions(),motion_b.get_joint_positions()], axis=0)
  result.rotations = np.concat([motion_a.rotations, motion_b.rotations], axis=0)

  q0 = motion_a.get_joint_orientation()
  njoints = result.joint_num
  for j in range(njoints):
    if np.dot(q0[-1,j],q0[-2,j]) < 0:
      q0[-2,j] *= -1
  dq0 = batch_quat_to_so3((Rotation.from_quat(q0[-1])*(Rotation.from_quat(q0[-2]).inv())).as_quat())
  dq1 = batch_quat_to_so3((Rotation.from_quat(q0[-2])*(Rotation.from_quat(q0[-3]).inv())).as_quat())
  w0 = -(dq0+dq1)/(2*dt)
  q0 = q0[-1,:]
  qt = motion_b.get_joint_orientation()[blending_frames]
  for j in range(njoints):
    if np.dot(q0[j],qt[j]) < 0:
      qt[j] *= -1
  q0 = Rotation.from_quat(q0)
  qt = Rotation.from_quat(qt)

  x = batch_quat_to_so3((qt*(q0.inv())).as_quat())
  v = w0
  for f in range(nframes_a, nframes_a+blending_frames):
    for _ in range(num_steps):
      x_prev = x.copy()
      x = (x_prev+(v+decay_rate*x_prev)*step)*np.exp(-decay_rate*step)
      v = (v+decay_rate*x_prev)*np.exp(-decay_rate*step)-decay_rate*x
    q = Rotation.from_quat(batch_so3_to_quat(x)).inv()*qt
    for j in range(njoints):
      if result.parents[j] != -1:
        lq = (q[result.parents[j]].inv() * q[j]).as_quat()
        result.rotations[f,j] = lq[[3,0,1,2]] # x,y,z,w -> w,x,y,z
      else:
        lq = (q[j]).as_quat()
        result.rotations[f,j] = lq[[3,0,1,2]] # x,y,z,w -> w,x,y,z

  for f in range(nframes_a,nframes_a+blending_frames):
    for j in range(njoints):
      if np.dot(result.rotations[f-1,j],result.rotations[f,j]) < 0:
        result.rotations[f,j] *= -1

  return result


def cross_fade_blending(motion_a: Motion, motion_b: Motion, blending_frames = 20):
  nframes_a = motion_a.frame_num
  nframes_b = motion_b.frame_num
  assert nframes_b > blending_frames, "motion b should be longer than blending_frames"
  result = motion_a.copy()
  # make sure consistent translation
  offset = motion_b.positions[0,0]-motion_a.positions[-1,0]
  motion_b.positions[:,0] -= offset
  result.frame_num = nframes_a + nframes_b
  result.positions = np.concat([motion_a.get_joint_positions(),motion_b.get_joint_positions()], axis=0)
  result.rotations = np.concat([motion_a.rotations, motion_b.rotations], axis=0)

  njoints = result.joint_num
  start_rot = result.rotations[nframes_a-1][:,[1,2,3,0]]
  end_rot = result.rotations[nframes_a + blending_frames-1][:,[1,2,3,0]]
  slerp = []
  for j in range(njoints):
    slerp.append(Slerp([0,1], Rotation.from_quat(np.stack([start_rot[j],end_rot[j]]))))
  for f in range(nframes_a-1, nframes_a + blending_frames):
    alpha = (f-nframes_a+1)/(blending_frames)
    for j in range(njoints):
      result.rotations[f,j] = slerp[j](alpha).as_quat()[[3,0,1,2]]
  return result


def critical_spring_damper(x0, v0, xt, t, half_life=1.0):
  decay_rate = np.log(2) / half_life
  x = x0-xt
  v = v0
  step = 1e-3
  time = 0.0

  while time < t:
    x_prev = x
    x = (x_prev + (v + decay_rate * x_prev) * step) * np.exp(-decay_rate * step)
    v = (v + decay_rate * x_prev) * np.exp(-decay_rate * step) - decay_rate * x
    time += step

  return xt + x


if __name__ == '__main__':
  # a = Motion.load_bvh('run_0.bvh')
  # b = Motion.load_bvh('run_1.bvh')
  # motion = inertial_blending(a, b)
  # motion.export('result_inertial.bvh')
  # motion = cross_fade_blending(a, b)
  # motion.export('result_cross_fade.bvh')

  # motion = Motion.load_bvh('Swimming_FW.bvh')
  # for i in range(4):
  #   start = i*120
  #   end = start+100
  #   motion_clip = motion.copy()
  #   motion_clip.frame_num = 100
  #   motion_clip.positions = motion.positions[start:end]
  #   motion_clip.rotations = motion.rotations[start:end]
  #   motion_clip.export(f'run_{i}.bvh')

  import matplotlib.pyplot as plt
  plt.figure()

  # t = np.arange(100)*0.01
  # y = np.zeros_like(t)
  # y[50:] = 1
  # y[80:90] = 0.5
  # y[85] = 0.7
  # y[10:20] = 0.2
  # ys = y.copy()

  # x = 0
  # v = 0
  # step = 1e-3
  # half_life = 0.03
  # decay_rate = np.log(1e12)/(np.log(np.e))
  # for i in range(1,100):
  #   time = 0
  #   if y[i] != y[i-1]:
  #     x = ys[i-1]-y[i]
  #   while time < 1e-2:
  #     x_prev = x 
  #     x = (x_prev + (v + decay_rate * x_prev) * step) * np.exp(-decay_rate * step)
  #     v = (v + decay_rate * x_prev) * np.exp(-decay_rate * step) - decay_rate * x
  #     time += step
  #   ys[i] = x+y[i]
  
  # plt.plot(t,y,label='raw')
  # plt.plot(t,ys,label='smooth')

  x0 = 1
  v0 = 0
  t = np.arange(100)*0.05
  y = np.zeros_like(t)
  for i in range(100):
    y[i] = critical_spring_damper(x0,v0,0,t[i],0.5)
  plt.plot(t,y,label='critical damped')

  plt.legend()
  plt.show()