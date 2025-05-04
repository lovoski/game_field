from utils.bvh_motion import Motion
from scipy.spatial.transform import Rotation
import numpy as np
import torch
from tqdm import tqdm
import os
from scipy.ndimage import gaussian_filter1d
import shutil

def combine_upper_lower(upper_motion_dir: str, lower_motion_dir: str, output_dir: str):
  os.makedirs(output_dir, exist_ok=True)
  upper_files = os.listdir(upper_motion_dir)
  # upper_files = [file for file in upper_files if file.endswith('test.bvh')]
  lower_files = os.listdir(lower_motion_dir)
  # lower_files = [file for file in lower_files if file.endswith('test.bvh')]

  # match upper motion and lower motion
  ul_pair = []
  for uf in upper_files:
    for lf in lower_files:
      if uf == lf:
        ul_pair.append((
          os.path.join(upper_motion_dir, uf),
          os.path.join(lower_motion_dir, lf)
        ))
        break

  for ufp, lfp in tqdm(ul_pair):
    upper = Motion.load_bvh(ufp)
    lower = Motion.load_bvh(lfp)
    # 1:52 -> upper rigs
    # 0,52: -> lower rigs
    nframes = min(upper.frame_num, lower.frame_num)
    motion = lower.copy()
    motion.frame_num = nframes
    motion.positions = motion.positions[:nframes]
    motion.rotations = np.zeros((nframes, motion.joint_num, 4))
    motion.rotations[:nframes,:,:] = lower.rotations[:nframes,:,:]
    motion.rotations[:nframes,1:52,:]=upper.rotations[:nframes,1:52,:]
    # apply smooth to the rotations
    for f in range(1,nframes):
      for j in range(motion.joint_num):
        if np.dot(motion.rotations[f-1,j], motion.rotations[f,j])<0:
          motion.rotations[f,j] *= -1
    for j in range(motion.joint_num):
      for c in range(4):
        motion.rotations[:,j,c] = gaussian_filter1d(motion.rotations[:,j,c], sigma=1)
    # apply smooth to root positions
    for c in range(3):
      motion.positions[:,0,c] = gaussian_filter1d(motion.positions[:,0,c], sigma=1)
    # export to dst dir
    filename = os.path.basename(ufp)
    motion.export(os.path.join(output_dir, filename))
    
if __name__ == '__main__':
  upper_motion_dir = r'arfriend_data/upper'
  lower_motion_dir = r'arfriend_data/lower'
  combined_motion_dir = r'arfriend_data/arfriend_data_combined'
  paired_motion_dir = r'arfriend_data/arfriend_data_paired'
  retarget_source_dir = r'arfriend_data/source'
  retarget_dst_dir = r'arfriend_data/retarget'
  retarget_processed_dir = r'arfriend_data/p_retarget'

  # combine_upper_lower(upper_motion_dir, lower_motion_dir, combined_motion_dir)
  
  # files = os.listdir(combined_motion_dir)
  # os.makedirs(paired_motion_dir, exist_ok=True)
  # for i in range(len(files)):
  #   for j in range(i, len(files)):
  #     if files[i].replace('.bvh', '_2.bvh') == files[j]:
  #       shutil.copy(os.path.join(combined_motion_dir, files[i]), os.path.join(paired_motion_dir, files[i]))
  #       shutil.copy(os.path.join(combined_motion_dir, files[j]), os.path.join(paired_motion_dir, files[j]))
  #       break
  
  # files = os.listdir(paired_motion_dir)
  # os.makedirs(retarget_source_dir, exist_ok=True)
  # fix_rotation = Rotation.from_euler('xyz', angles=[0, -90, 0], degrees=True)
  # for file in tqdm(files):
  #   filepath = os.path.join(paired_motion_dir, file)
  #   motion = Motion.load_bvh(filepath)
  #   # rotate the rest pose
  #   n_joints = motion.joint_num
  #   n_frames = motion.frame_num
  #   # scale & rotate rest pos
  #   for j in range(n_joints):
  #     motion.offsets[j, :] = fix_rotation.apply(motion.offsets[j, :])

  #   for fid in range(n_frames):
  #     # scale & rotate root pos
  #     motion.positions[fid, 0, :] = fix_rotation.apply(motion.positions[fid, 0, :])
  #     # repair loca rotation for each joint
  #     for j in range(n_joints):
  #       motion.rotations[fid, j, :] = (fix_rotation * Rotation.from_quat(motion.rotations[fid, j, :][[1, 2, 3, 0]]) * fix_rotation.inv()).as_quat()[[3, 0, 1, 2]]

  #   # set the first frame as tpose
  #   motion.rotations[0, :, 1:] = 0
  #   motion.rotations[0, :, 0] = 1

  #   # set the first frame on ground
  #   toe_joint_idx = []
  #   for joint_idx, joint_name in enumerate(motion.names):
  #     if joint_name.lower().find('toe') != -1:
  #       toe_joint_idx.append(joint_idx)
  #   global_pos = motion.get_T_pose()
  #   motion.positions[0, ...] = 0
  #   motion.positions[0, 0, 1] = global_pos[0, 1]-global_pos[toe_joint_idx[0], 1]
  #   motion.export(os.path.join(retarget_source_dir, file))
  
  files = os.listdir(retarget_dst_dir)
  os.makedirs(retarget_processed_dir, exist_ok=True)
  for file in tqdm(files):
    filepath = os.path.join(retarget_dst_dir, file)
    motion = Motion.load_bvh(filepath)
    motion.positions[0] = motion.positions[1]
    motion.rotations[0] = motion.rotations[1]
    motion.export(os.path.join(retarget_processed_dir, file))