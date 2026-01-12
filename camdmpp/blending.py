from scipy.spatial.transform import Rotation
import utils.bvh as bvh
import numpy as np
import os

clip_len = 40
frametime = 1/60
inertia_halflife = 0.06
blend_wnd = 15
inertia_lambda = np.log(2)/(inertia_halflife*np.log(np.e))
base_dir = '100style_pf'
export_dir = '100style_pf_blending'
os.makedirs(export_dir, exist_ok=True)

for file in sorted(os.listdir(base_dir)):
  data = bvh.load(os.path.join(base_dir, file))
  rotations = data['rotations'].copy() # xyzw
  positions = data['positions'].copy()
  nframes, njoints, nfeats = rotations.shape
  blended_rotations = rotations.copy()
  blended_positions = positions.copy()

  for clip_start_idx in range(clip_len, nframes, clip_len):
    # blend this clip with the end frame of last clip
    from_rot = rotations[clip_start_idx-1]
    to_rot = rotations[clip_start_idx]
    for j in range(njoints):
      if np.dot(from_rot[j], to_rot[j]) < 0.0:
        to_rot[j] = -to_rot[j]
    off_rot = (Rotation.from_quat(from_rot)*(Rotation.from_quat(to_rot).inv())).as_rotvec()
    # estimate angular velocity
    from0 = Rotation.from_quat(rotations[clip_start_idx-2])
    from1 = Rotation.from_quat(rotations[clip_start_idx-1])
    from_ang = (from1*(from0.inv())).as_rotvec()/frametime
    to0 = Rotation.from_quat(rotations[clip_start_idx])
    to1 = Rotation.from_quat(rotations[clip_start_idx+1])
    to_ang = (to1*(to0.inv())).as_rotvec()/frametime
    off_ang = from_ang-to_ang

    for bf in range(blend_wnd):
      dt = (bf+1)*frametime
      r = off_rot
      av = off_ang
      r = (r+(av+inertia_lambda*r)*dt)*np.exp(-inertia_lambda*dt)
      blended_rotations[clip_start_idx+bf] = (Rotation.from_rotvec(r) * Rotation.from_quat(rotations[clip_start_idx+bf])).as_quat()

    from_pos = positions[clip_start_idx-1,0].copy()
    to_pos = positions[clip_start_idx,0].copy()
    off_pos = from_pos-to_pos
    from_vel = positions[clip_start_idx-1,0]-positions[clip_start_idx-2,0]
    to_vel = positions[clip_start_idx+1,0]-positions[clip_start_idx,0]
    off_vel = from_vel-to_vel
    
    for bf in range(blend_wnd):
      dt = (bf+1)*frametime
      x = off_pos
      v = off_vel
      x = (x+(v+inertia_lambda*x)*dt)*np.exp(-inertia_lambda*dt)
      blended_positions[clip_start_idx+bf,0] = x+positions[clip_start_idx+bf,0]

  data['rotations'] = blended_rotations
  data['positions'] = blended_positions
  bvh.save(os.path.join(export_dir, file), data)
