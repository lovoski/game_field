from scipy.spatial.transform import Rotation as R
from utils.bvh_motion import Motion
from utils.motion_modules import remove_joints
import numpy as np
import os
from tqdm import tqdm

def format_motion_files_motion_persona(source_dir, target_dir, scale=1):
  if not os.path.exists(target_dir):
    os.mkdir(target_dir)
  raw_filenames = os.listdir(source_dir)

  fix_rotation = R.from_euler('xyz', angles=[-90, 180, 0], degrees=True)

  for raw_filename in tqdm(raw_filenames):
    motion = Motion.load_bvh(os.path.join(source_dir, raw_filename))
    # remove finger joints if exists
    motion = remove_joints(motion, ['middle', 'ring', 'pinky', 'index', 'thumb'])
    # resample motion to 30fps
    n_frames = int(np.floor(motion.frame_num * motion.frametime * 30))
    n_joints = motion.joint_num
    n_positions = np.zeros((n_frames, n_joints, 3), dtype=float)
    n_rotations = np.zeros((n_frames, n_joints, 4), dtype=float)
    # scale & rotate rest pos
    n_offsets = motion.offsets
    for j in range(n_joints):
      n_offsets[j, :] = scale * fix_rotation.apply(n_offsets[j, :])

    for fid in range(n_frames):
      frame = fid / 30 / motion.frametime
      first = int(np.floor(frame))
      last = first + 1
      alpha = frame - first

      n_positions[fid, 0, :] = motion.positions[first, 0, :] * (1-alpha) + motion.positions[last, 0, :] * alpha
      # scale & rotate root pos
      n_positions[fid, 0, :] = scale * fix_rotation.apply(n_positions[fid, 0, :])
      n_rotations[fid, :, :] = motion.rotations[first, :, :] * (1-alpha) + motion.rotations[last, :, :] * alpha
      # normalization
      n_rotations[fid, :, :] /= np.repeat(np.linalg.norm(n_rotations[fid, :, :], axis=1).reshape(n_joints, 1), repeats=4, axis=1)
      # repair loca rotation for each joint
      for j in range(n_joints):
        n_rotations[fid, j, :] = (fix_rotation * R.from_quat(n_rotations[fid, j, :][[1, 2, 3, 0]]) * fix_rotation.inv()).as_quat()[[3, 0, 1, 2]]

    motion.end_offsets = np.zeros_like(motion.end_offsets)
    motion.offsets = n_offsets
    motion.frame_num = n_frames
    motion.frametime = 1/30
    motion.positions = n_positions
    motion.rotations = n_rotations

    motion.export(os.path.join(target_dir, raw_filename))

if __name__ == '__main__':
    raw_dir = r'D:\Dataset\motion_nn\Charlie'
    dst_dir = r'./motion_persona'
    
    format_motion_files_motion_persona(raw_dir, dst_dir)
