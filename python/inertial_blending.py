from scipy.spatial.transform import Rotation, Slerp
from utils.bvh_motion import Motion
import numpy as np

def batch_quat_to_so3(q):
  # x,y,z,w -> x,y,z
  half_theta = np.acos(q[:,3])
  sin_half_theta = np.sin(half_theta)
  u = np.zeros((q.shape[0], 3), dtype=float)
  for i in range(sin_half_theta.shape[0]):
    if abs(sin_half_theta[i]) < 1e-5:
      u[i] = 0
    else:
      u[i] = q[i,:3]/sin_half_theta[i]*half_theta[i]*2
  return u

def batch_so3_to_quat(a):
  theta = np.linalg.norm(a, axis=1)
  q = np.zeros((a.shape[0],4), dtype=float)
  for i in range(a.shape[0]):
    if abs(theta[i]) < 1e-5:
      q[i] = np.array([0,0,0,1],dtype=float)
    else:
      q[i,:3] = a[i]/theta[i]*np.sin(theta[i]/2)
      q[i,3] = np.cos(theta[i]/2)
  return q


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


def inertial_blending_quat(q0, w0, qt, dt, frames=20):
  """
  q0: initial rotation in xyzw order, shape [n, 4]
  w0: initial angular velocity, shape [n, 3]
  qt: target rotation in xyzw order, shape [n, 4]
  dt: delta time between each frames
  frames: the desired frames between q0 and qt

  return: blended frames between q0 and qt in xyzw order, shape [frames, n, 4]
  """
  n = q0.shape[0]
  q0_rot = Rotation.from_quat(q0)
  qt_rot = Rotation.from_quat(qt)
  x = batch_quat_to_so3((q0_rot*(qt_rot.inv())).as_quat())
  v = w0.copy()
  decay_rate = np.log(64)/(dt*frames*np.log(np.e))
  result = np.zeros((frames, n, 4))
  for f in range(frames):
    x_prev = x.copy()
    x = (x_prev+(v+decay_rate*x_prev)*dt)*np.exp(-decay_rate*dt)
    v = (v+decay_rate*x_prev)*np.exp(-decay_rate*dt)-decay_rate*x
    q = Rotation.from_quat(batch_so3_to_quat(x))*qt_rot
    for i in range(n):
      result[f,i] = q[i].as_quat()
  return result


def inertial_blending(motion_a: Motion, motion_b: Motion, blending_frames=20):
  nframes_a = motion_a.frame_num
  nframes_b = motion_b.frame_num
  assert nframes_b > blending_frames, "motion b should be longer than blending_frames"
  dt = motion_a.frametime
  duration = dt*blending_frames
  decay_rate = np.log(32)/(duration*np.log(np.e))
  result = motion_a.copy()
  # make sure consistent translation
  offset = motion_b.positions[0,0]-motion_a.positions[-1,0]
  motion_b.positions[:,0] -= offset
  result.frame_num = nframes_a + nframes_b
  result.positions = np.concat([motion_a.get_joint_positions(),motion_b.get_joint_positions()], axis=0)
  result.rotations = np.concat([motion_a.rotations, motion_b.rotations], axis=0)

  q0_wxyz = motion_a.rotations
  q0 = q0_wxyz[...,[1,2,3,0]]
  njoints = result.joint_num
  for j in range(njoints):
    if np.dot(q0[-1,j],q0[-2,j]) < 0:
      q0[-2,j] *= -1
  dq = batch_quat_to_so3((Rotation.from_quat(q0[-1])*(Rotation.from_quat(q0[-2]).inv())).as_quat())
  w0 = dq/dt
  q0 = q0[-1,:]
  qt_wxyz = motion_b.rotations[blending_frames]
  qt = qt_wxyz[...,[1,2,3,0]]
  for j in range(njoints):
    if np.dot(q0[j],qt[j]) < 0:
      qt[j] *= -1

  blended_frames = inertial_blending_quat(q0, w0, qt, dt, blending_frames)
  for f in range(nframes_a, nframes_a+blending_frames):
    for j in range(njoints):
      result.rotations[f,j] = blended_frames[f-nframes_a, j, [3,0,1,2]] # xyzw -> wxyz

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


if __name__ == '__main__':
  a = Motion.load_bvh('run_0.bvh')
  b = Motion.load_bvh('run_1.bvh')
  motion = inertial_blending(a, b)
  motion.export('result_inertial.bvh')
  motion = cross_fade_blending(a, b)
  motion.export('result_cross_fade.bvh')

  # motion = Motion.load_bvh('Swimming_FW.bvh')
  # for i in range(4):
  #   start = i*120
  #   end = start+100
  #   motion_clip = motion.copy()
  #   motion_clip.frame_num = 100
  #   motion_clip.positions = motion.positions[start:end]
  #   motion_clip.rotations = motion.rotations[start:end]
  #   motion_clip.export(f'run_{i}.bvh')

  # import matplotlib.pyplot as plt
  # plt.figure()

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
  # decay_rate = np.log(2)/(0.5*np.log(np.e))
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

  # x0 = 1
  # v0 = 1
  # t = np.arange(100)*0.05
  # x = (x0+(v0+decay_rate*x0)*t)*np.exp(-decay_rate*t)
  # v = (v0+decay_rate*x0)*np.exp(-decay_rate*t)-decay_rate*x
  # plt.plot(t,x,label='x')
  # plt.plot(t,v,label='v')
  # # y = np.zeros_like(t)
  # # for i in range(100):
  # #   y[i] = critical_spring_damper(x0,v0,0,t[i],0.5)
  # # plt.plot(t,y,label='critical damped')

  # plt.legend()
  # plt.show()