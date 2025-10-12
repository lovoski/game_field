import smplx
from smplx.joint_names import JOINT_NAMES
import os
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import utils.bvh as bvh

def smpl_to_bvh(filepath, model, betas, rots, trans):
    """
    betas: (1, 10)
    rots: (nframes, njoints, 3)
    trans: (nframes, 3)
    """
    njoints = model.parents.detach().shape[0]
    parents = model.parents.detach().numpy()
    output = model(betas=betas, body_pose=rots)
    rest = output.joints.detach().numpy()[0, :njoints]
    offsets = rest.copy()
    names = []
    for j in range(njoints):
        names.append(JOINT_NAMES[j])
        if parents[j] != -1:
            offsets[j] -= rest[parents[j]]
    offsets[0] = 0

    rotations = np.zeros((1, njoints, 4))
    rotations[:, :, 0] = 1
    positions = np.zeros((1, njoints, 3))
    positions[0] = offsets
    bvh.save(
        filepath,
        {
            "rotations": rotations,
            "positions": positions,
            "offsets": offsets,
            "parents": parents,
            "names": names,
            "frametime": 1.0 / 30.0,
        },
        True,
    )


if __name__ == "__main__":
    smpl_model_base_dir = "data/models"
    model = smplx.create(smpl_model_base_dir, model_type="smplx", gender="NEUTRAL")
    smpl_to_bvh('template.bvh', model, torch.zeros((1, 10)))
