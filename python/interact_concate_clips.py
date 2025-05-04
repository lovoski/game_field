from utils.bvh_motion import Motion
from scipy.spatial.transform import Rotation
import numpy as np
import torch
import os

def concat_clips(filepath_list):
  motions = []
  for filepath in filepath_list:
    motions.append(Motion.load_bvh(filepath))
  concat_motion = motions[0]
  init_facing_dir = np.array([0,0,1])
  for i in range(1, len(motions)):
    cur_motion = motions[i]
    # adjust root facing and translation
    root_rot = Rotation.from_quat(concat_motion.rotations[-1,0,[1,2,3,0]]) # wxyz->xyzw
    n_root_rot = Rotation.from_quat(cur_motion.rotations[0,0,[1,2,3,0]])
    n_rot = cur_motion.rotations.copy()
    n_pos = cur_motion.get_joint_positions()
    face_dir0 = root_rot.apply(init_facing_dir)
    face_dir1 = n_root_rot.apply(init_facing_dir)
    face_dir0[1] = 0
    face_dir1[1] = 0
    delta_rot,_ = Rotation.align_vectors(face_dir0/np.linalg.norm(face_dir0), face_dir1/np.linalg.norm(face_dir1))
    nframes = n_rot.shape[0]
    for f in range(nframes):
      n_pos[f,0] = delta_rot.apply(n_pos[f,0])
      m_rot = (delta_rot*Rotation.from_quat(n_rot[f,0,[1,2,3,0]])).as_quat()
      n_rot[f,0] = m_rot[[3,0,1,2]]
    delta_pos = concat_motion.positions[-1,0]-n_pos[0,0]
    delta_pos[1] = 0
    n_pos[:,0] += delta_pos

    # dead blending between clip transition
    # ...

    # final concatenation
    concat_motion.frame_num += cur_motion.frame_num
    concat_motion.positions = np.concatenate([concat_motion.positions, n_pos], axis=0)
    concat_motion.rotations = np.concatenate([concat_motion.rotations, n_rot], axis=0)

  return concat_motion

if __name__ == '__main__':
  # base_dir = 'eval_one_person'
  # output_dir = 'eval_one_person_concat'

  base_dir = 'latent_decode'
  output_dir = 'latent_decode_concat'

  os.makedirs(output_dir, exist_ok=True)
  dir_names = os.listdir(base_dir)
  for dir_name in dir_names:
    files = os.listdir(os.path.join(base_dir, dir_name))
    # person_gt, person_ref, person_pred = [], [], []
    person_a, person_b = [], []
    for file in files:
      # if file.endswith('gt.bvh'):
      #   person_gt.append(os.path.join(base_dir, dir_name, file))
      # elif file.endswith('ref.bvh'):
      #   person_ref.append(os.path.join(base_dir, dir_name,file))
      # elif file.endswith('pred.bvh'):
      #   person_pred.append(os.path.join(base_dir, dir_name,file))
      if file.endswith('A.bvh'):
        person_a.append(os.path.join(base_dir, dir_name, file))
      elif file.endswith('B.bvh'):
        person_b.append(os.path.join(base_dir, dir_name,file))
  
    # assert len(person_gt) == len(person_ref) == len(person_pred), 'length of clips mismatch'
    assert len(person_a) == len(person_b), 'length of clips mismatch'
  
    # concat_clips(person_gt).export(os.path.join(output_dir, f'{dir_name}_person_gt.bvh'))
    # concat_clips(person_ref).export(os.path.join(output_dir, f'{dir_name}_person_ref.bvh'))
    # concat_clips(person_pred).export(os.path.join(output_dir, f'{dir_name}_person_pred.bvh'))
    concat_clips(person_a).export(os.path.join(output_dir, f'{dir_name}_person_a.bvh'))
    concat_clips(person_b).export(os.path.join(output_dir, f'{dir_name}_person_b.bvh'))
