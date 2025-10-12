import os
import numpy as np
from tqdm import tqdm
import utils.bvh as bvh
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, find_peaks

def extract_foot_phase(data, foot_ids, height_threshold = 3):
  """
  The unit for lafan1 dataset is cm
  """
  for foot_id in foot_ids:
    grot, gpos = bvh.fk(data['rotations'], data['positions'], data['parents'])
    foot_pos = gpos[:, foot_id]
    foot_height = gpos[:, foot_id, 1]
    smoothen_foot_height = savgol_filter(foot_height, 31, 2)
    foot_vel = np.linalg.norm(np.diff(foot_pos, axis=0), axis=1)

    local_maximas, _ = find_peaks(smoothen_foot_height)
    maximas = []
    for local_maxima in local_maximas:
      if foot_height[local_maxima] > height_threshold:
        maximas.append(local_maxima)
    mask = np.zeros_like(foot_height)
    mask[maximas] = 1

    plt.figure()
    plt.plot(foot_height, label='height')
    plt.plot(smoothen_foot_height, label='smooth height')
    plt.plot(mask, label='mask')
    plt.legend()
    plt.show()

if __name__ == '__main__':
  base_dir = 'data/lafan1_subset'
  filenames = os.listdir(base_dir)
  for filename in filenames:
    data = bvh.load(os.path.join(base_dir, filename))
    foot_ids = [data['names'].index('RightToeBase'), data['names'].index('LeftToeBase')]
    extract_foot_phase(data, foot_ids)
