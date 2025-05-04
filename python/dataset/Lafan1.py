from scipy.spatial.transform import Rotation as R
from utils.bvh_motion import Motion
import numpy as np
from tqdm import tqdm
from utils.motion_from_fk import motion_from_fk_with_rest_pose
from utils.motion_modules import scaling

def process_template(filepath):
  """
  rotate around +x axis for -90 degrees, use first frame as rest pose
  """
  rot = R.from_euler('xyz', [-90, 0, 0], degrees=True)
  motion = Motion.load_bvh(filepath)
  positions = motion.get_joint_positions()
  rest_pos = rot.apply(positions[0])
  root_pos = rest_pos[0]
  rest_pos -= root_pos
  return rest_pos

def conc_task(dst_filepath, src_filepath):
  motion = Motion.load_bvh(src_filepath)
  p_motion = scaling(motion_from_fk_with_rest_pose(
    motion.get_joint_positions(),
    rest_pos,
    motion.parents,
    names=motion.names
  ))
  p_motion.export(dst_filepath, save_ori_scal=False)

if __name__ == '__main__':
  import os

  rest_pos = process_template(r'lafan/template.bvh')
  src_dir = r'lafan/raw'
  dst_dir = r'lafan/processed'
  if not os.path.exists(dst_dir):
    os.makedirs(dst_dir)
  files = os.listdir(src_dir)
  for file in tqdm(files):
    if not file.endswith('.bvh'):
      continue
    conc_task(
      os.path.join(dst_dir, file),
      os.path.join(src_dir, file)
    )
  print(f'all tasks finished')