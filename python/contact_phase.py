from scipy.spatial.transform import Rotation as R
from utils.bvh_motion import Motion
import utils.motion_modules as mm
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import numpy as np
import scipy as sp
import torch

def optimize_phase(x, y, wnd_radius=15, nepochs=20):
  y = torch.from_numpy(y).to(torch.float)
  x = torch.from_numpy(x).to(torch.float)
  n = y.shape[0]
  ret_A,ret_f,ret_phi,ret_b = np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)
  for frame in range(wnd_radius, n-wnd_radius, wnd_radius):
    wnd_start = frame-wnd_radius
    wnd_end = frame+wnd_radius
    wnd_y = y[wnd_start:wnd_end]

    wnd_max = torch.max(wnd_y)
    wnd_min = torch.min(wnd_y)
    wnd_mean = torch.mean(wnd_y)
    wnd_scale = 0.5*(wnd_max-wnd_min)
    wnd_y = (wnd_y-wnd_mean)/wnd_scale

    nextreme = 0
    for fid in range(wnd_start+1, wnd_end-1):
      if y[fid]>y[fid-1] and y[fid]>y[fid+1]:
        nextreme += 1
      if y[fid]<y[fid-1] and y[fid]<y[fid+1]:
        nextreme += 1

    min_freq = torch.pi*(nextreme-1)/(x[wnd_end-1]-x[wnd_start])
    max_freq = torch.pi*(nextreme+2)/(x[wnd_end-1]-x[wnd_start])
    min_amp = 0.8

    A = torch.ones(1).to(torch.float)
    f = 0.5*(min_freq+max_freq)*torch.ones(1).to(torch.float)
    phi = torch.zeros(1).to(torch.float)
    b = torch.zeros(1).to(torch.float)
    A.requires_grad_(True)
    f.requires_grad_(True)
    phi.requires_grad_(True)
    b.requires_grad_(True)

    optim = torch.optim.LBFGS(
      params=[A, f, phi, b],
      lr=1e-2,
      max_iter=100
    )

    def closure():
      optim.zero_grad()
      loss = torch.nn.functional.mse_loss(
        A*torch.sin(f*x[wnd_start:wnd_end]+phi)+b,
        wnd_y
      )
      loss.backward()
      return loss

    plt.figure()
    plt.plot(wnd_y, label='y')
    plt.plot((A*torch.sin(f*x[wnd_start:wnd_end]+phi)+b).detach(), label='fit')
    plt.legend()
    plt.show()

    for epoch in range(nepochs):
      optim.step(closure)
      with torch.no_grad():
        if A<min_amp:
          A.data.clamp_(min=min_amp)
        if f<min_freq or f>max_freq:
          f.data.clamp_(min=min_freq, max=max_freq)

    plt.figure()
    plt.plot(wnd_y, label='y')
    plt.plot((A*torch.sin(f*x[wnd_start:wnd_end]+phi)+b).detach(), label='fit')
    plt.plot((f*x[wnd_start:wnd_end]+phi).detach()%torch.pi, label='phase')
    plt.legend()
    plt.show()

    ret_A[frame] = A.detach()
    ret_f[frame] = f.detach()
    ret_phi[frame] = phi.detach()
    ret_b[frame] = b.detach()

  return ret_A, ret_f, ret_phi, ret_b

def contact_label(motion: Motion, idx: list[int], threshold=0.1):
  """
  return binary mask as contact label, true for contact
  """
  motion = mm.scaling(motion) # scale motion to height 1
  pos = motion.get_joint_positions()
  vel = np.zeros_like(pos)
  vel[:-1] = (pos[1:]-pos[:-1])/(motion.frametime)
  vel[-1] = vel[-2]
  vel_norm = np.linalg.norm(vel[:, idx], axis=-1)

  label = vel_norm < threshold

  return label

def contact_phase(contact):
  nframes, njoints = contact.shape
  tmp_b, tmp_a = butter(4, 0.1, btype='low', analog=False)
  filtered = np.zeros_like(contact, dtype=float)
  phase = np.zeros_like(contact, dtype=float)
  for d in range(njoints):
    filtered[:, d] = filtfilt(tmp_b, tmp_a, contact[:, d])

    x = np.arange(nframes)
    A, f, phi, b = optimize_phase(x[400:600], filtered[400:600,d])

    plt.figure()
    plt.plot(filtered[:,d], label='filtered')
    plt.plot(contact[:, d], label='contact')
    plt.legend()
    plt.show()

if __name__ == '__main__':
  import os

  base_dir = r'lafan/processed'
  files = os.listdir(base_dir)
  for file in files:
    if not file.endswith('.bvh'):
      continue
    motion = Motion.load_bvh(os.path.join(base_dir, file))
    toe_idx = []
    for idx, name in enumerate(motion.names):
      if name.lower().find('toe') != -1:
        toe_idx.append(idx)

    contact = contact_label(motion, toe_idx)

    contact_phase(contact)
