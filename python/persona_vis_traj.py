from utils.bvh_motion import Motion
import numpy as np
from PIL import Image
from tqdm import tqdm
import os

def write_colorbar(filepath, width=200, height=10):
  color = np.zeros((height, width, 3), dtype=np.uint8)
  for i in range(width):
    ratio = i/(width-1)
    color[:,i] = np.array([255,0,0])*ratio+np.array([0,0,255])*(1-ratio)
  Image.fromarray(color, mode='RGB').save(filepath)

def produce_textured_traj(filepath, output_dir):
  motion = Motion.load_bvh(filepath)
  pos = motion.get_joint_positions()[:,0] * 0.01
  pos[:,1] = 0
  vel = np.linalg.norm(pos[1:]-pos[:-1], axis=1)
  vel = np.append(vel, vel[-1])
  max_v, min_v = np.max(vel), np.min(vel)
  vel = (vel-min_v)/max_v
  nframes = pos.shape[0]

  if not os.path.exists(os.path.join(output_dir, 'colorbar_mat.mtl')):
    with open(os.path.join(output_dir, 'colorbar_mat.mtl'), 'w') as f:
      f.write("newmtl colorbar_mat\nKa 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\nmap_Kd colorbar.png")

  with open(os.path.join(output_dir, os.path.basename(filepath).split('.')[0] + '.obj'), 'w') as f:
    f.write("mtllib colorbar_mat.mtl\nusemtl colorbar_mat\n")
    for i in range(nframes):
      f.write(f'v {pos[i,0]} {pos[i,1]} {pos[i,2]}\n')
      f.write(f'vt {vel[i]} 0\n')
    line_indices = " ".join(str(i+1) for i in range(nframes))
    f.write(f"l {line_indices}\n")

if __name__ == '__main__':
  """
  After writing these .obj files, 
  import them into blender, 
  right click convert to curve objects, 
  setup width to render.
  """
  base_dir = r'D:\repo\persona_unity\processed_eval_output'
  output_dir = r'D:\repo\persona_unity\traj_output'
  os.makedirs(output_dir, exist_ok=True)
  write_colorbar(os.path.join(output_dir, 'colorbar.png'))
  
  files = os.listdir(base_dir)
  for file in tqdm(files):
    produce_textured_traj(os.path.join(base_dir, file), output_dir)