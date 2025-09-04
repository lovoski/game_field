import smplx
import os
from smpl_joint_names import SMPLX_JOINT_NAMES, SMPLH_JOINT_NAMES, SMPL_JOINT_NAMES
import numpy as np
import torch
from scipy.spatial.transform import Rotation

def smplx_to_bvh():
  pass

if __name__ == '__main__':
  # data = np.load('data/amass/ACCAD/Male1Running_c3d/Run_C24_-_quick_side_step_left_stageii.npz', allow_pickle=True)
  # # data = np.load('data/amass/CNRS/283/-01_L_1_stageii.npz', allow_pickle=True)
  # model_type = data['surface_model_type'].tolist()
  # gender = data['gender'].tolist()
  # betas = data['betas']
  # root_trans = data['trans']
  # root_orien = data['root_orient']
  # poses = data['poses']
  # model = smplx.create(model_path='data/models', model_type=model_type, gender=gender, batch_size=1)

  # rest = model(betas=(torch.from_numpy(betas).unsqueeze(0)))
  # rest_pose = rest.joints.detach().cpu().numpy().squeeze()
  # # create skeleton from rest_pose
  

  # data = np.load('data/models/smplx/SMPLX_NEUTRAL.npz', allow_pickle=True)
  # print(data)

  # base_dir, output_dir = 'data/models/smplx', 'data/models/smplx_modified'
  # os.makedirs(output_dir, exist_ok=True)
  # files = os.listdir(base_dir)
  # for file in files:
  #   if not file.endswith('.npz'):
  #     continue
  #   data = np.load(os.path.join(base_dir, file), allow_pickle=True)
  #   data_dict = dict(data)
  #   fields_to_remove = ['part2num', 'joint2num', 'allow_pickle']
  #   for field in fields_to_remove:
  #       if field in data_dict:
  #           del data_dict[field]
  #   np.savez(os.path.join(output_dir, file), **data_dict)

    base_dir = 'data/models/smpl'
    files = os.listdir(base_dir)
    for file in files:
      if not file.endswith('.npz'):
        continue
      data = dict(np.load(os.path.join(base_dir, file)))
      print(data)
