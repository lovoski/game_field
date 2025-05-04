from utils.bvh_motion import Motion
import numpy as np

def resample_motion(motion: Motion, fps: int):
    duration = motion.frame_num * motion.frametime
    nframes = int(np.floor(duration * fps))
    sampling = np.linspace(0, duration, nframes)
    n_positions = np.zeros((nframes, motion.joint_num, 3), dtype=np.float32)
    n_rotations = np.zeros((nframes, motion.joint_num, 4), dtype=np.float32)
    
    positions = motion.get_joint_positions()
    for f in range(nframes):
        sample_time = sampling[f]
        a = int(np.floor(sample_time/motion.frametime))
        b = a+1
        alpha = sample_time/motion.frametime-a
        if b >= positions.shape[0]:
            n_positions[f] = positions[-1]
            n_rotations[f] = motion.rotations[-1]
            break
        n_positions[f] = (1-alpha)*positions[a]+alpha*positions[b]
        n_rotations[f] = (1-alpha)*motion.rotations[a]+alpha*motion.rotations[b]
        rot_norm = np.linalg.norm(n_rotations[f], axis=1)
        n_rotations[f] /= rot_norm.reshape(-1, 1)

    n_motion = motion.copy()
    n_motion.frame_num = nframes
    n_motion.frametime = 1/nframes
    n_motion.positions = n_positions
    n_motion.rotations = n_rotations
    return n_motion

if __name__ == '__main__':
  motion = resample_motion(Motion.load_bvh('20231119_001Sky_051.bvh'), 10)
  print(f'{motion.joint_num}, {motion.frametime}, {motion.frame_num}')

  motion.export('aaa.bvh')