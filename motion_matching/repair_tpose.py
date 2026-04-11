import os
from scipy.spatial.transform import Rotation
import numpy as np
import bvh

FILE_PATH = os.path.dirname(os.path.realpath(__file__))

def migrate_tpose(tpose0, tpose1, base_dir, output_dir):
  """
  tpose0 and tpose1 should have the same joint names and hierarchy and fk joint positions, but rest pose are allowed to be different.
  
  migrate motion of base_dir from the rest pose of tpose0 to the rest pose of tpose1, and save the repaired motion to output_dir.
  """
  
  os.makedirs(output_dir, exist_ok=True)
  repair_rot = []
  grot0,_ = bvh.fk(tpose0['rotations'], tpose0['positions'], tpose0['parents'])
  grot1,_ = bvh.fk(tpose1['rotations'], tpose1['positions'], tpose1['parents'])
  for i in range(len(tpose0['names'])):
    repair_rot.append(Rotation.from_quat(grot0[0,i,[1,2,3,0]]).inv()*Rotation.from_quat(grot1[0,i,[1,2,3,0]]))

  for file in os.listdir(base_dir):
    if file.endswith('.bvh'):
      data = bvh.load(os.path.join(base_dir, file))
      nframes, njoints = data['rotations'].shape[:2]
      grot,_ = bvh.fk(data['rotations'], data['positions'], data['parents'])
      repaired_ori = np.zeros_like(data['rotations'])
      repaired_ori[:,:,0] = 1
      for f in range(nframes):
        for i in range(njoints):
          repaired_ori[f,i] = (Rotation.from_quat(grot[f,i,[1,2,3,0]])*repair_rot[i]).as_quat()[[3,0,1,2]]
          if i == 0:
            data['rotations'][f,i] = repaired_ori[f,i]
          else:
            data['rotations'][f,i] = (Rotation.from_quat(repaired_ori[f,data['parents'][i],[1,2,3,0]]).inv()*Rotation.from_quat(repaired_ori[f,i,[1,2,3,0]])).as_quat()[[3,0,1,2]]
      data['offsets'] = tpose1['offsets']
      print(f"save repaired motion to {os.path.join(output_dir, file)}")
      bvh.save(os.path.join(output_dir, file), data)

def bake_tpose(filepath, output_path):
  """
  bake the first frame of the bvh file to the rest pose, and save the repaired motion to output_path.
  """
  
  data = bvh.load(filepath)
  grot, gpos = bvh.fk(data['rotations'], data['positions'], data['parents'])
  baked_rot = np.zeros_like(data['rotations'][0:1])
  baked_pos = np.zeros_like(data['positions'][0:1])
  baked_rot[:,:,0] = 1
  baked_offset = np.zeros_like(data['offsets'])
  for i in range(len(data['names'])):
    if data['parents'][i] == -1:
      baked_offset[i] = 0
      baked_pos[0,i] = gpos[0,i]
    else:
      baked_offset[i] = gpos[0,i] - gpos[0,data['parents'][i]]
      baked_pos[0,i] = baked_offset[i]
  data['rotations'] = baked_rot
  data['positions'] = baked_pos
  data['offsets'] = baked_offset
  bvh.save(output_path, data)

if __name__ == "__main__":
  base_dir = os.path.join(FILE_PATH, "data")
  
  bake_tpose(os.path.join(base_dir, "lafan1_tpose.bvh"), os.path.join(base_dir, "lafan1_tpose_baked.bvh"))
  
  migrate_tpose(
    bvh.load(os.path.join(base_dir, "lafan1_tpose.bvh")), 
    bvh.load(os.path.join(base_dir, "lafan1_tpose_baked.bvh")), 
    os.path.join(base_dir, '_lafan1_loco'), 
    os.path.join(base_dir, '_lafan1_loco_repaired'))

  # migrate_tpose(
  #   bvh.load(os.path.join(base_dir, "pfnn_tpose.bvh")), 
  #   bvh.load(os.path.join(base_dir, "pfnn_tpose_baked.bvh")), 
  #   os.path.join(base_dir, 'pfnn_processed'), 
  #   os.path.join(base_dir, 'pfnn_processed_repaired'))
