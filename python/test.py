import numpy as np
from utils import bvh, bvh_motion
import os
from scipy.spatial.transform import Rotation

def forward_kinematics_trans(data):
  nframes, njoints = data['positions'].shape[:2]
  local_trans = np.zeros((nframes, njoints, 4, 4)).astype(np.float64)
  global_trans = np.zeros((nframes, njoints, 4, 4)).astype(np.float64)
  for frame in range(nframes):
    for joint in range(njoints):
      # construct T*R homogeneous transform matrix
      local_trans[frame, joint, 3, 3] = 1
      local_trans[frame, joint, :3, :3] = Rotation.from_quat(data['rot_quats'][frame, joint, [1,2,3,0]]).as_matrix()
      local_trans[frame, joint, :3, 3] = data['positions'][frame, joint]
    for joint_idx, parent_idx in enumerate(data['parents']):
      # build up transform matrix chain
      if parent_idx == -1:
        global_trans[frame, joint_idx] = local_trans[frame, joint_idx]
      else:
        global_trans[frame, joint_idx] = global_trans[frame, parent_idx] @ local_trans[frame, joint_idx]
  return global_trans, local_trans

def forward_kinematics_direct(data):
  nframes, njoints = data['positions'].shape[:2]
  gpos = np.zeros((nframes, njoints, 3)).astype(np.float64)
  grot = np.zeros((nframes, njoints, 4)).astype(np.float64)
  grot[...,0] = 1
  for frame in range(nframes):
    for joint_idx, parent_idx in enumerate(data['parents']):
      if parent_idx == -1:
        grot[frame, joint_idx] = data['rot_quats'][frame, joint_idx]
        gpos[frame, joint_idx] = data['positions'][frame, joint_idx]
      else:
        grot[frame, joint_idx] = (Rotation.from_quat(grot[frame,parent_idx,[1,2,3,0]])*Rotation.from_quat(data['rot_quats'][frame,joint_idx,[1,2,3,0]])).as_quat()[3,0,1,2]
        gpos[frame, joint_idx] = gpos[frame, parent_idx] + Rotation.from_quat(grot[frame, joint_idx, [1,2,3,0]]).apply(data['positions'][frame, joint_idx])
  return gpos, grot

def decompose_transform_trs(trans):
  pos = trans[...,:3,3]
  scl = np.array([
    np.linalg.norm(trans[...,:3,0], axis=-1),
    np.linalg.norm(trans[...,:3,1], axis=-1),
    np.linalg.norm(trans[...,:3,2], axis=-1)])
  rot = trans[...,:3,:3]
  rot[:,0] /= scl[0]
  rot[:,1] /= scl[1]
  rot[:,2] /= scl[2]
  return pos, rot, scl

def make_first_frame_rest_pose(data):
  gpos, grot = forward_kinematics_direct(data)
  
  

if __name__ == '__main__':
  base_dir = '/mnt/d/repo/GenoViewPython-MotionMatching/resources/lafan_motion'
  # base_dir = '/mnt/d/repo/GenoViewPython-MotionMatching/resources/persona_motion'

  for filename in os.listdir(base_dir):
    if not filename.endswith('.bvh'):
      continue
    data = bvh.load(os.path.join(base_dir, filename))
    data = make_first_frame_rest_pose(data)
    bvh.save('test.bvh', data)
    break