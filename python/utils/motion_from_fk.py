from scipy.spatial.transform import Rotation as R
from utils.bvh_motion import Motion
import numpy as np
import ipdb


def from_to_rot(f, t):
    rot, _ = R.align_vectors(t, f)
    return rot


def normalize(arr):
    return arr/np.linalg.norm(arr)


def from2_to2_rot(f0: np.ndarray, f1: np.ndarray, t0: np.ndarray, t1: np.ndarray):
    rot0, _ = R.align_vectors(t0, f0)
    t0_normed = t0 / np.linalg.norm(t0)
    t1_now = rot0.apply(f1)
    t1_now_proj = t1_now - np.dot(t1_now, t0_normed) * t0_normed
    t1_proj = t1 - np.dot(t1, t0_normed) * t0_normed
    costheta = np.clip(
        np.dot(t1_now_proj, t1_proj)
        / (np.linalg.norm(t1_now_proj) * np.linalg.norm(t1_proj)),
        -1,
        1,
    )
    theta = np.arccos(costheta)
    cos_half_theta = np.cos(theta / 2)
    cross_vec = np.cross(
        t1_now_proj / np.linalg.norm(t1_now_proj), t1_proj / np.linalg.norm(t1_proj)
    )
    sin_half_theta = np.sin(theta / 2)
    if np.dot(cross_vec, t0_normed) < 0:
        sin_half_theta *= -1
    rot_add_quat = np.zeros(4)
    rot_add_quat[0:3] = sin_half_theta * t0_normed
    rot_add_quat[3] = cos_half_theta
    return R.from_quat(rot_add_quat) * rot0


def decompose_axis_rot(rot, axis):
    """
    Decompose a rotation to two rotations rot = q2*q1, q1 along the given axis
    """
    axis = normalize(axis)
    axis_rotated = normalize(rot.apply(axis))
    q2 = from_to_rot(axis, axis_rotated)
    q1 = q2.inv() * rot
    return q1, q2


def find_joint_chains(parents):
    njoints = len(parents)
    joint_children = {}
    for j in range(njoints):
        joint_children[j] = []
        if parents[j] != -1:
            joint_children[parents[j]].append(j)
    joint_chains = []
    for j in range(njoints):
        if len(joint_children[j]) > 1:
            for chain_joint in joint_children[j]:
                tmp_joint_chain = []
                current_joint = chain_joint
                while len(joint_children[current_joint]) == 1:
                    tmp_joint_chain.append(current_joint)
                    current_joint = joint_children[current_joint][0]
                joint_chains.append(tmp_joint_chain)
    return joint_chains


def motion_from_fk(positions, parents, initial_facing=np.array([0, 0, 1]), names=None):
    rest_positions = positions[0, ...]
    return motion_from_fk_with_rest_pose(
        positions, rest_positions, parents, initial_facing, names
    )


def motion_from_fk_with_rest_pose(
    positions, rest_positions, parents, initial_facing=np.array([0, 0, 1]), names=None
):
    """
    rest_positions should be in tpose
    """
    nframes, njoints, nfeats = positions.shape
    if names == None:
        joint_names = ["joint_" + str(i) for i in range(njoints)]
    else:
        joint_names = names
    joint_parents = parents
    motion_rotations = np.zeros((nframes, njoints, 4))
    motion_rotations[:, :, 0] = 1
    motion_orientations = np.zeros((nframes, njoints, 4))
    motion_orientations[:, :, 3] = 1
    motion_positions = positions
    # take the first frame as rest pose
    motion_offsets = np.zeros((njoints, 3))
    joint_children = {}
    for i in range(njoints):
        joint_children[i] = []
        if joint_parents[i] != -1:
            joint_children[joint_parents[i]].append(i)
            motion_offsets[i, :] = (
                rest_positions[i, :] - rest_positions[joint_parents[i], :]
            )
        else:
            motion_offsets[i, :] = np.squeeze(rest_positions[i, :])

    # construct rotation from relative positions
    end_effector_count = 0
    facing_dir = np.zeros((nframes, njoints, 3))
    facing_dir[:, :, ...] = initial_facing
    for j in range(njoints):
        if len(joint_children[j]) == 0:
            # end effector
            end_effector_count += 1

        if joint_parents[j] == -1:
            # root joint
            continue
        else:
            parent_ind = joint_parents[j]
            parent_actual_position = motion_positions[:, parent_ind, :]
            if len(joint_children[parent_ind]) == 1:
                # parent with only one child
                # the parent could have a twisting freedom
                joint_actual_position = motion_positions[:, j, :]
                actual_vector = joint_actual_position - parent_actual_position
                rest_vector = rest_positions[j] - rest_positions[parent_ind]
                for f in range(1, nframes, 1):
                    parent_orientation = from_to_rot(
                        rest_vector, np.squeeze(actual_vector[f, ...])
                    )

                    if len(joint_children[joint_parents[parent_ind]]) == 1:
                        # parent's parent with only one child
                        pp_ind = joint_parents[parent_ind]
                        pp_ori = R.from_quat(motion_orientations[f, pp_ind, :])
                        supposed_p_c_vec = normalize(pp_ori.apply(rest_vector))
                        actual_p_c_vec = normalize(np.squeeze(actual_vector[f, ...]))
                        change_rot = from_to_rot(supposed_p_c_vec, actual_p_c_vec)
                        parent_orientation = change_rot * pp_ori
                    else:
                        # parent's parent with more than one child
                        actual_facing_dir = normalize(parent_orientation.apply(initial_facing))
                        supposed_facing_dir = normalize(facing_dir[f, 0, :])
                        rotate_axis = normalize(actual_vector[f, :])
                        actual_facing_dir = normalize(actual_facing_dir - np.dot(rotate_axis, actual_facing_dir) * rotate_axis)
                        supposed_facing_dir = normalize(supposed_facing_dir - np.dot(rotate_axis, supposed_facing_dir) * rotate_axis)
                        costheta = np.clip(np.dot(actual_facing_dir, supposed_facing_dir), -1, 1)
                        theta = np.arccos(costheta)
                        if np.dot(np.cross(actual_facing_dir, supposed_facing_dir), rotate_axis) < 0:
                            theta *= -1
                        twist_fix = np.zeros(4)
                        twist_fix[0:3] = np.sin(theta/2) * rotate_axis
                        twist_fix[3] = np.cos(theta/2)
                        parent_orientation = R.from_quat(twist_fix) * parent_orientation

                    facing_dir[f, parent_ind, :] = parent_orientation.apply(initial_facing)
                    motion_orientations[f, parent_ind, :] = parent_orientation.as_quat()
            else:
                # parent with more than one child
                # the parent won't have additional twisting freedom
                child0_index = joint_children[parent_ind][0]
                child1_index = joint_children[parent_ind][1]
                child0_actual_position = motion_positions[:, child0_index, :]
                child1_actual_position = motion_positions[:, child1_index, :]
                child0_actual_vector = child0_actual_position - parent_actual_position
                child1_actual_vector = child1_actual_position - parent_actual_position
                child0_rest_vector = (
                    rest_positions[child0_index, :] - rest_positions[parent_ind, :]
                )
                child1_rest_vector = (
                    rest_positions[child1_index, :] - rest_positions[parent_ind, :]
                )
                for f in range(1, nframes, 1):
                    parent_orientation = from2_to2_rot(
                        child0_rest_vector,
                        child1_rest_vector,
                        np.squeeze(child0_actual_vector[f, ...]),
                        np.squeeze(child1_actual_vector[f, ...]),
                    )
                    facing_dir[f, parent_ind, :] = parent_orientation.apply(initial_facing)
                    motion_orientations[f, parent_ind, :] = parent_orientation.as_quat()

    # set the orientation for each end effector as its parent
    for j in range(njoints):
        if len(joint_children[j]) == 0:
            motion_orientations[:, j, :] = motion_orientations[:, joint_parents[j], :]

    # convert global rotation to local rotation
    for j in range(njoints):
        if joint_parents[j] != -1:
            parent_ind = joint_parents[j]
            for f in range(1, nframes, 1):
                parent_ori = R.from_quat(motion_orientations[f, parent_ind, :])
                self_ori = R.from_quat(motion_orientations[f, j, :])
                motion_rotations[f, j, [1, 2, 3, 0]] = (
                    parent_ori.inv() * self_ori
                ).as_quat()
        else:
            motion_rotations[:, j, [1, 2, 3, 0]] = motion_orientations[:, j, :]

    return Motion(
        motion_rotations,
        motion_positions,
        motion_offsets,
        joint_parents,
        joint_names,
        1 / 30,
        np.zeros((end_effector_count, 3)),
    )


def bake_first_frame_as_rest_pose(motion: Motion, initial_facing=np.array([0, 0, 1])):
    positions = motion.get_joint_positions()
    rest_positions = np.squeeze(positions[0, ...])
    return motion_from_fk(
        positions[1:, ...], rest_positions, motion.parents, initial_facing, motion.names
    )


def load_positions(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        tmp = lines[0].strip().split(' ')
        # nframes, njoints, 3
        data = np.ndarray((len(lines), int(len(tmp)/3), 3), dtype=np.float32)
        for fid, line in enumerate(lines):
            line_data = line.strip().split(' ')
            njoints = int(len(line_data)/3)
            for jid in range(njoints):
                data[fid, jid, 0] = np.float32(line_data[3 * jid + 0])
                data[fid, jid, 1] = np.float32(line_data[3 * jid + 1])
                data[fid, jid, 2] = np.float32(line_data[3 * jid + 2])
        return data

if __name__ == "__main__":
    import os
    from tqdm import tqdm
    
    # base_dir = '25_lp'
    # files = os.listdir(os.path.join(base_dir, 'pos'))
    # tmp_skeleton = Motion.load_bvh(os.path.join(base_dir, 'skel.bvh'))
    # for file in tqdm(files):
    #     filepath = os.path.join(base_dir, 'pos', file)
    #     positions = load_positions(filepath)
    #     motion_from_fk_with_rest_pose(
    #         positions=positions,
    #         rest_positions=tmp_skeleton.get_T_pose(),
    #         parents=tmp_skeleton.parents,
    #         names=tmp_skeleton.names
    #     ).export(os.path.join(base_dir, file.split('.')[0]+'.bvh'))

    base_dir = 'offset_lp'
    files = os.listdir(base_dir)
    for file in tqdm(files):
        if not file.endswith('.pos'):
            continue
        filepath = os.path.join(base_dir, file)
        tmp_skeleton = Motion.load_bvh(os.path.join(base_dir, file.replace('.pos', '.bvh')))
        
        positions = load_positions(filepath)
        motion_from_fk_with_rest_pose(
            positions=positions,
            rest_positions=tmp_skeleton.get_T_pose(),
            parents=tmp_skeleton.parents,
            names=tmp_skeleton.names
        ).export(os.path.join(base_dir, file.split('.')[0]+'.output.bvh'))
