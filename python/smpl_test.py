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
  # for idx, x in enumerate(smpl_data['J_regressor'][0]):
  #   if x != 0:
  #     print(idx)

  data = smpl_data['J_regressor']
  njoints, nvertices = data.shape
  jregressor_indices, jregressor_weights = [], []
  for i in range(njoints):
    non_zeros, non_zeros_data = [], []
    for j in range(nvertices):
      if data[i,j] != 0:
        non_zeros_data.append(float(data[i,j]))
        non_zeros.append(j)
    jregressor_indices.append(non_zeros)
    jregressor_weights.append(non_zeros_data)
    # print(non_zeros)
    # print(non_zeros_data)
    # print(non_zeros)
  result = ''
  for i in range(len(jregressor_weights)):
    tmp = '{'
    for j in range(len(jregressor_weights[i])-1):
      tmp = tmp + str(jregressor_weights[i][j]) + ','
    tmp = tmp + str(jregressor_weights[i][-1]) + '},\n'
    result = result + tmp
  print(result)



  # print(non_zero_digits)
  # print(non_zero_data)

  # model = smplx.create('./data/models', model_type='smpl', gender='NEUTRAL')
  # print(model.parents)
