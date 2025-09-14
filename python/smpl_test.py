import smplx
import os
from smpl_joint_names import SMPLX_JOINT_NAMES, SMPLH_JOINT_NAMES, SMPL_JOINT_NAMES
import numpy as np
import torch
from scipy.spatial.transform import Rotation

def smplx_to_bvh():
  pass

if __name__ == '__main__':
  smpl_model_base_dir = 'data/models'
  smpl_data = dict(np.load(os.path.join(smpl_model_base_dir, 'smpl/SMPL_NEUTRAL.npz')))
  
  print(smpl_data.keys())

  model = smplx.create('./data/models', model_type='smpl', gender='NEUTRAL')
  print(model.parents)
