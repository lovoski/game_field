import numpy as np
import os
import utils.bvh as bvh
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from lmp_evolve_algorithm import Population
from scipy.ndimage import gaussian_filter1d
from tqdm import tqdm

def butterworth(values, deltatime, max_frequency):
  if max_frequency == 0.0:
    return values.copy()
  dF2 = values.shape[0]-1
  Dat2 = np.zeros(dF2+4)
  Dat2[2:(dF2+2)] = values[:dF2]
  Dat2[1] = values[0]
  Dat2[0] = values[0]
  Dat2[dF2+3] = values[dF2]
  Dat2[dF2+2] = values[dF2]
  
  wc = np.tan(max_frequency*np.pi*deltatime)
  k1 = np.sqrt(2) * wc
  k2 = wc*wc
  a = k2/(1+k1+k2)
  b = 2*a
  c = a
  k3 = b/k2
  d = -2*a+k3
  e = 1-2*a-k3
  
  DatYt = np.zeros(dF2+4)
  DatYt[1] = values[0]
  DatYt[0] = values[0]
  for s in range(2,2+dF2):
    DatYt[s] = a*Dat2[s]+b*Dat2[s-1]+c*Dat2[s-2]+d*DatYt[s-1]+e*DatYt[s-2]
  DatYt[dF2+3] = DatYt[dF2+1]
  DatYt[dF2+2] = DatYt[dF2+1]
  
  DatZt = np.zeros(dF2+2)
  DatZt[dF2] = DatYt[dF2+2]
  DatZt[dF2+1] = DatYt[dF2+3]
  for t in range(-dF2+1,0):
    DatZt[-t] = a*DatYt[-t+2]+b*DatYt[-t+3]+c*DatYt[-t+4]+d*DatZt[-t+1]+e*DatZt[-t+2]
  
  return DatZt[:(dF2+1)]

def repeat_clamp(t, length):
  return np.clip(t-np.floor(t/length)*length, 0.0, length)

def process_local_phase(data):
  cutoff_vel = 0.005
  max_iterations = 10
  individuals = 50
  elites = 5
  max_frequency = 4
  filter_window = 0.5
  exploration = 0.2

  # capture contacts for left and right foot
  foot_ids = [data['names'].index('RightFoot'), data['names'].index('LeftFoot')]
  framerate = int(np.round(1.0/data['frametime']))
  deltatime = data['frametime']

  grot, gpos = bvh.fk(data['rotations'], data['positions'], data['parents'])
  nframes = gpos.shape[0]
  contacts = np.zeros((len(foot_ids),nframes))
  velocity = np.zeros((len(foot_ids),nframes))
  for i, foot_id in enumerate(foot_ids):
    pos = gpos[:, foot_id, :]
    velocity[i] = np.linalg.norm(np.gradient(pos, axis=0), axis=1)
    vel_mask = (velocity[i] < cutoff_vel).astype(float)
    contacts[i] = vel_mask

  for enumerate_idx, foot_id in enumerate(foot_ids):
    # process the data
    phase_vectors = np.zeros((nframes, 2))
    timestamps = np.arange(nframes) * deltatime
    G = np.zeros(nframes)
    for i in range(nframes):
      sampled_size = int(np.round((filter_window+filter_window)/deltatime))
      sampled_timestamps = np.arange(sampled_size) * deltatime + timestamps[i] - filter_window
      sampled_indices = np.clip(np.round(sampled_timestamps * framerate).astype(int), 0, nframes-1)
      sampled_contacts = contacts[enumerate_idx][sampled_indices]
      G[i] = (contacts[enumerate_idx,i]-np.mean(sampled_contacts))/(np.std(sampled_contacts)+(1e-8))
    G = butterworth(G, deltatime, max_frequency)
    F = np.zeros((nframes,4), dtype=float)

    for i in tqdm(range(nframes)):
      sampled_size = int(np.round((filter_window+filter_window)/deltatime))
      sampled_timestamps = np.arange(sampled_size) * deltatime + timestamps[i] - filter_window
      sampled_indices = np.clip(np.round(sampled_timestamps * framerate).astype(int), 0, nframes-1)
      
      sampled_G = G[sampled_indices]
      min_G, max_G = np.min(sampled_G), np.max(sampled_G)
      
      lowerbounds = np.array([0.0, 1.0/max_frequency, -0.5, min_G])
      upperbounds = np.array([max_G-min_G, max_frequency, 0.5, max_G])
      seed = 0.5*(lowerbounds + upperbounds)
      
      def loss_func(x):
        ret = (x[0]*np.sin(2*np.pi*(x[1]*timestamps[sampled_indices]-x[2]))+x[3])-G[sampled_indices]
        return np.sqrt(np.sum(ret*ret)/sampled_size)

      pop = Population(
        individuals, 
        elites, 
        exploration, 
        lowerbounds, 
        upperbounds, 
        seed,
        loss_func
      )

      for k in range(max_iterations):
        pop.evolve()
      
      F[i] = pop.get_solution()

      if i >= 4000:
        break

    amplitudes = np.zeros(nframes, dtype=float)
    phase_values = np.zeros((nframes), dtype=float)
    for i in range(nframes):
      x = F[i]
      t = timestamps[i]
      f = x[1]
      s = x[2]
      phase = 2*np.pi*repeat_clamp(f*t-s,1)
      phase_window = 0 if f == 0 else int(np.round(framerate/f))
      sampled_size = int(np.round(phase_window/deltatime))
      sampled_timestamps = np.arange(sampled_size) * deltatime + timestamps[i] - filter_window
      sampled_indices = np.clip(np.round(sampled_timestamps * framerate).astype(int), 0, nframes-1)
      amplitudes[i] = np.mean(gaussian_filter1d(F[sampled_indices][:,0],2))
      
      phase_vectors[i] = np.array([np.sin(phase),np.cos(phase)])

    phase_vectors[:,0] = butterworth(phase_vectors[:,0], deltatime, max_frequency)
    phase_vectors[:,1] = butterworth(phase_vectors[:,1], deltatime, max_frequency)

    for i in range(nframes):
      rad = np.acos(np.array([0, 1]).dot(phase_vectors[i]/(np.linalg.norm(phase_vectors[i])+(1e-8))))
      if rad < 0:
        rad += 2*np.pi
      phase_values[i] = repeat_clamp(rad, np.pi)

    plt.figure()
    plt.plot(G, label='G')
    plt.plot(contacts[enumerate_idx], label='contacts')
    plt.plot(phase_values, label='phase')
    plt.plot(amplitudes, label='amplitude')
    plt.plot(F[:,0], label='F0')
    # plt.plot(F[:,1], label='F1')
    # plt.plot(F[:,2], label='F2')
    # plt.plot(F[:,3], label='F3')
    plt.legend()
    plt.show()


if __name__ == '__main__':
  base_dir = 'data/lafan1_subset'
  for filename in os.listdir(base_dir):
    data = bvh.load(os.path.join(base_dir, filename))
    # cm -> m
    data['offsets'] *= 0.01
    data['positions'] *= 0.01
    
    process_local_phase(data)
    
    # TODO: remove this
    break