"""
Thanks to:
    https://github.com/orangeduck/GenoViewPython
"""
import re, os, ntpath
import numpy as np

channelmap = {
    'Xrotation': 'x',
    'Yrotation': 'y',
    'Zrotation': 'z'
}

channelmap_inv = {
    'x': 'Xrotation',
    'y': 'Yrotation',
    'z': 'Zrotation',
}

ordermap = {
    'x': 0,
    'y': 1,
    'z': 2,
}

def _fast_cross(a, b):
    o = np.empty(np.broadcast(a, b).shape)
    o[...,0] = a[...,1]*b[...,2] - a[...,2]*b[...,1]
    o[...,1] = a[...,2]*b[...,0] - a[...,0]*b[...,2]
    o[...,2] = a[...,0]*b[...,1] - a[...,1]*b[...,0]
    return o

def eye(shape, dtype=np.float32):
    return np.ones(list(shape) + [4], dtype=dtype) * np.asarray([1, 0, 0, 0], dtype=dtype)

def length(x):
    return np.sqrt(np.sum(x * x, axis=-1))

def normalize(x, eps=1e-8):
    return x / (length(x)[...,np.newaxis] + eps)

def abs(x):
    return np.where(x[...,0:1] > 0.0, x, -x)

def from_angle_axis(angle, axis):
    c = np.cos(angle / 2.0)[..., np.newaxis]
    s = np.sin(angle / 2.0)[..., np.newaxis]
    q = np.concatenate([c, s * axis], axis=-1)
    return q

def to_xform(x):
    
    out = np.empty(list(x.shape[:-1]) + [3, 3])
    out[...,0,0] = 1.0 - (x[...,2] * 2 * x[...,2] + x[...,3] * 2 * x[...,3])
    out[...,0,1] = x[...,1] * 2 * x[...,2] - x[...,0] * 2 * x[...,3]
    out[...,0,2] = x[...,1] * 2 * x[...,3] + x[...,0] * 2 * x[...,2]
    out[...,1,0] = x[...,1] * 2 * x[...,2] + x[...,0] * 2 * x[...,3]
    out[...,1,1] = 1.0 - (x[...,1] * 2 * x[...,1] + x[...,3] * 2 * x[...,3])
    out[...,1,2] = x[...,2] * 2 * x[...,3] - x[...,0] * 2 * x[...,1]
    out[...,2,0] = x[...,1] * 2 * x[...,3] - x[...,0] * 2 * x[...,2]
    out[...,2,1] = x[...,2] * 2 * x[...,3] + x[...,0] * 2 * x[...,1]
    out[...,2,2] = 1.0 - (x[...,1] * 2 * x[...,1] + x[...,2] * 2 * x[...,2])
    return out
    
def to_xform_xy(x):

    qw, qx, qy, qz = x[...,0:1], x[...,1:2], x[...,2:3], x[...,3:4]
    
    x2, y2, z2 = qx + qx, qy + qy, qz + qz
    xx, yy, wx = qx * x2, qy * y2, qw * x2
    xy, yz, wy = qx * y2, qy * z2, qw * y2
    xz, zz, wz = qx * z2, qz * z2, qw * z2
    
    return np.concatenate([
        np.concatenate([1.0 - (yy + zz), xy - wz], axis=-1)[...,np.newaxis,:],
        np.concatenate([xy + wz, 1.0 - (xx + zz)], axis=-1)[...,np.newaxis,:],
        np.concatenate([xz - wy, yz + wx], axis=-1)[...,np.newaxis,:],
    ], axis=-2)

def from_euler(e, order='zyx'):
    axis = {
        'x': np.asarray([1, 0, 0], dtype=np.float32),
        'y': np.asarray([0, 1, 0], dtype=np.float32),
        'z': np.asarray([0, 0, 1], dtype=np.float32)}

    q0 = from_angle_axis(e[..., 0], axis[order[0]])
    q1 = from_angle_axis(e[..., 1], axis[order[1]])
    q2 = from_angle_axis(e[..., 2], axis[order[2]])

    return mul(q0, mul(q1, q2))

def from_xform(ts):
    
    return normalize(
        np.where((ts[...,2,2] < 0.0)[...,np.newaxis],
            np.where((ts[...,0,0] >  ts[...,1,1])[...,np.newaxis],
                np.concatenate([
                    (ts[...,2,1]-ts[...,1,2])[...,np.newaxis], 
                    (1.0 + ts[...,0,0] - ts[...,1,1] - ts[...,2,2])[...,np.newaxis], 
                    (ts[...,1,0]+ts[...,0,1])[...,np.newaxis], 
                    (ts[...,0,2]+ts[...,2,0])[...,np.newaxis]], axis=-1),
                np.concatenate([
                    (ts[...,0,2]-ts[...,2,0])[...,np.newaxis], 
                    (ts[...,1,0]+ts[...,0,1])[...,np.newaxis], 
                    (1.0 - ts[...,0,0] + ts[...,1,1] - ts[...,2,2])[...,np.newaxis], 
                    (ts[...,2,1]+ts[...,1,2])[...,np.newaxis]], axis=-1)),
            np.where((ts[...,0,0] < -ts[...,1,1])[...,np.newaxis],
                np.concatenate([
                    (ts[...,1,0]-ts[...,0,1])[...,np.newaxis], 
                    (ts[...,0,2]+ts[...,2,0])[...,np.newaxis], 
                    (ts[...,2,1]+ts[...,1,2])[...,np.newaxis], 
                    (1.0 - ts[...,0,0] - ts[...,1,1] + ts[...,2,2])[...,np.newaxis]], axis=-1),
                np.concatenate([
                    (1.0 + ts[...,0,0] + ts[...,1,1] + ts[...,2,2])[...,np.newaxis], 
                    (ts[...,2,1]-ts[...,1,2])[...,np.newaxis], 
                    (ts[...,0,2]-ts[...,2,0])[...,np.newaxis], 
                    (ts[...,1,0]-ts[...,0,1])[...,np.newaxis]], axis=-1))))

    
def from_xform_xy(x):

    c2 = _fast_cross(x[...,0], x[...,1])
    c2 = c2 / np.sqrt(np.sum(np.square(c2), axis=-1))[...,np.newaxis]
    c1 = _fast_cross(c2, x[...,0])
    c1 = c1 / np.sqrt(np.sum(np.square(c1), axis=-1))[...,np.newaxis]
    c0 = x[...,0]
    
    return from_xform(np.concatenate([
        c0[...,np.newaxis], 
        c1[...,np.newaxis], 
        c2[...,np.newaxis]], axis=-1))

def inv(q):
    return np.asarray([1, -1, -1, -1], dtype=np.float32) * q

def mul(x, y):
    x0, x1, x2, x3 = x[...,0], x[...,1], x[...,2], x[...,3]
    y0, y1, y2, y3 = y[...,0], y[...,1], y[...,2], y[...,3]
    
    o = np.empty(np.broadcast(x, y).shape)
    o[...,0] = y0 * x0 - y1 * x1 - y2 * x2 - y3 * x3
    o[...,1] = y0 * x1 + y1 * x0 - y2 * x3 + y3 * x2
    o[...,2] = y0 * x2 + y1 * x3 + y2 * x0 - y3 * x1
    o[...,3] = y0 * x3 - y1 * x2 + y2 * x1 + y3 * x0
    return o

def inv_mul(x, y):
    return mul(inv(x), y)

def mul_inv(x, y):
    return mul(x, inv(y))

def mul_vec(q, x):
    t = 2.0 * _fast_cross(q[...,1:], x)
    return x + q[...,0][...,None] * t + _fast_cross(q[..., 1:], t)

def inv_mul_vec(q, x):
    return mul_vec(inv(q), x)

def unroll(x):
    y = x.copy()
    for i in range(1, len(x)):
        d0 = np.sum( y[i] * y[i-1], axis=-1)
        d1 = np.sum(-y[i] * y[i-1], axis=-1)
        y[i][d0 < d1] = -y[i][d0 < d1]
    return y

def between(x, y):
    return np.concatenate([
        np.sqrt(np.sum(x*x, axis=-1) * np.sum(y*y, axis=-1))[...,np.newaxis] + 
        np.sum(x * y, axis=-1)[...,np.newaxis], 
        _fast_cross(x, y)], axis=-1)
        
def log(x, eps=1e-5):
    length = np.sqrt(np.sum(np.square(x[...,1:]), axis=-1))[...,np.newaxis]
    halfangle = np.where(length < eps, np.ones_like(length), np.arctan2(length, x[...,0:1]) / length)
    return halfangle * x[...,1:]
    
def exp(x, eps=1e-5):
    halfangle = np.sqrt(np.sum(np.square(x), axis=-1))[...,np.newaxis]
    c = np.where(halfangle < eps, np.ones_like(halfangle), np.cos(halfangle))
    s = np.where(halfangle < eps, np.ones_like(halfangle), np.sinc(halfangle / np.pi))
    return np.concatenate([c, s * x], axis=-1)
    
def to_scaled_angle_axis(x, eps=1e-5):
    return 2.0 * log(x, eps)
    
def from_scaled_angle_axis(x, eps=1e-5):
    return exp(x / 2.0, eps)

def fk(lrot, lpos, parents):
    
    grot, gpos = lrot.copy(), lpos.copy()
    
    for i in range(1, len(parents)):
        p = parents[i]
        gpos[...,i,:] = mul_vec(grot[...,p,:], lpos[...,i,:]) + gpos[...,p,:]
        grot[...,i,:] = mul    (grot[...,p,:], lrot[...,i,:])
    
    return grot, gpos
    
def ik(grot, gpos, parents):
    lrot, lpos = grot.copy(), gpos.copy()
    lrot[...,1:,:] = mul(inv(grot[...,parents[1:],:]), grot[...,1:,:])
    lpos[...,1:,:] = mul_vec(inv(grot[...,parents[1:],:]), gpos[...,1:,:] - gpos[...,parents[1:],:])
    return lrot, lpos
    
def fk_vel(lrot, lpos, lvel, lang, parents):
    
    grot, gpos, gvel, gang = lrot.copy(), lpos.copy(), lvel.copy(), lang.copy()
    
    for i in range(1, len(parents)):
        p = parents[i]
        gpos[...,i,:] = mul_vec(grot[...,p,:], lpos[...,i,:]) + gpos[...,p,:]
        grot[...,i,:] = mul    (grot[...,p,:], lrot[...,i,:])
        gvel[...,i,:] = (mul_vec(grot[...,p,:], lvel[...,i,:]) + 
            _fast_cross(gang[...,p,:], mul_vec(grot[...,p,:], lpos[...,i,:])) +
            gvel[...,p,:])
        gang[...,i,:] = mul_vec(grot[...,p,:], lang[...,i,:]) + gang[...,p,:]
        
    return grot, gpos, gvel, gang
        
        
def to_euler(x, order='xyz'):
    """
    Converts a quaternion or batch of quaternions to Euler angles.

    This function supports all 12 standard Euler angle sequences.

    Args:
        x (np.ndarray): 
            Input array of quaternions with shape (..., 4).
            The quaternion format is assumed to be (w, x, y, z).
        order (str, optional): 
            The desired order of rotation axes. Defaults to 'xyz'.
            Supported orders are:
            - Tait-Bryan angles: 'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx'
            - Proper Euler angles: 'zxz', 'zyz', 'xyx', 'xzx', 'yxy', 'yzy'

    Returns:
        np.ndarray: 
            The corresponding Euler angles in radians with shape (..., 3).
            The order of the angles corresponds to the `order` string.

    Raises:
        NotImplementedError: If the provided `order` is not supported.
    """
    # Get rotation matrix from quaternion
    R = to_xform(x)
    
    # Extract Euler angles based on rotation order
    if order == 'xyz':
        y = np.arctan2(-R[..., 2, 0], np.sqrt(R[..., 0, 0]**2 + R[..., 1, 0]**2))
        x = np.arctan2(R[..., 2, 1], R[..., 2, 2])
        z = np.arctan2(R[..., 1, 0], R[..., 0, 0])
        return np.stack([x, y, z], axis=-1)
    
    elif order == 'zyx':
        y = np.arcsin(np.clip(R[..., 0, 2], -1, 1))
        x = np.arctan2(-R[..., 1, 2], R[..., 2, 2])
        z = np.arctan2(-R[..., 0, 1], R[..., 0, 0])
        return np.stack([z, y, x], axis=-1)
    
    elif order == 'zxy':
        x = np.arcsin(-np.clip(R[..., 1, 2], -1, 1))
        z = np.arctan2(R[..., 1, 0], R[..., 1, 1])
        y = np.arctan2(R[..., 0, 2], R[..., 2, 2])
        return np.stack([z, x, y], axis=-1)
    
    elif order == 'xzy':
        z = np.arcsin(-np.clip(R[..., 0, 1], -1, 1))
        x = np.arctan2(R[..., 2, 1], R[..., 1, 1])
        y = np.arctan2(R[..., 0, 2], R[..., 0, 0])
        return np.stack([x, z, y], axis=-1)
    
    elif order == 'yxz':
        x = np.arcsin(-np.clip(R[..., 2, 1], -1, 1))
        y = np.arctan2(R[..., 2, 0], R[..., 2, 2])
        z = np.arctan2(R[..., 0, 1], R[..., 1, 1])
        return np.stack([y, x, z], axis=-1)
    
    elif order == 'yzx':
        z = np.arcsin(np.clip(R[..., 1, 0], -1, 1))
        y = np.arctan2(-R[..., 2, 0], R[..., 0, 0])
        x = np.arctan2(-R[..., 1, 2], R[..., 1, 1])
        return np.stack([y, z, x], axis=-1)
    
    # Proper Euler angles
    elif order == 'zxz':
        x = np.arccos(np.clip(R[..., 2, 2], -1, 1))
        z1 = np.arctan2(R[..., 0, 2], -R[..., 1, 2])
        z2 = np.arctan2(R[..., 2, 0], R[..., 2, 1])
        return np.stack([z1, x, z2], axis=-1)
    
    elif order == 'zyz':
        y = np.arccos(np.clip(R[..., 2, 2], -1, 1))
        z1 = np.arctan2(R[..., 1, 2], R[..., 0, 2])
        z2 = np.arctan2(R[..., 2, 1], -R[..., 2, 0])
        return np.stack([z1, y, z2], axis=-1)
    
    elif order == 'yxy':
        x = np.arccos(np.clip(R[..., 1, 1], -1, 1))
        y1 = np.arctan2(R[..., 0, 1], R[..., 2, 1])
        y2 = np.arctan2(R[..., 1, 0], -R[..., 1, 2])
        return np.stack([y1, x, y2], axis=-1)
    
    elif order == 'yzy':
        z = np.arccos(np.clip(R[..., 1, 1], -1, 1))
        y1 = np.arctan2(R[..., 2, 1], -R[..., 0, 1])
        y2 = np.arctan2(R[..., 1, 2], R[..., 1, 0])
        return np.stack([y1, z, y2], axis=-1)
    
    elif order == 'xyx':
        y = np.arccos(np.clip(R[..., 0, 0], -1, 1))
        x1 = np.arctan2(R[..., 1, 0], -R[..., 2, 0])
        x2 = np.arctan2(R[..., 0, 1], R[..., 0, 2])
        return np.stack([x1, y, x2], axis=-1)
    
    elif order == 'xzx':
        z = np.arccos(np.clip(R[..., 0, 0], -1, 1))
        x1 = np.arctan2(R[..., 2, 0], R[..., 1, 0])
        x2 = np.arctan2(R[..., 0, 2], -R[..., 0, 1])
        return np.stack([x1, z, x2], axis=-1)
    
    else:
        raise NotImplementedError(f"Rotation order {order} not implemented")


def _end_site_mask(data):
    mask = np.zeros(len(data['parents']), dtype=bool)
    end_sites = data.get('end_sites')
    if end_sites is None:
        return mask

    end_sites = np.asarray(end_sites)
    if end_sites.shape == mask.shape:
        return end_sites.astype(bool, copy=True)

    if end_sites.ndim == 1 and np.issubdtype(end_sites.dtype, np.integer):
        mask[end_sites] = True
        return mask

    raise ValueError(
        "Unexpected end_sites shape %s for %i joints" % (end_sites.shape, len(data['parents'])))


def _children_by_parent(parents):
    children = [[] for _ in range(len(parents))]
    for joint_index in range(1, len(parents)):
        parent_index = parents[joint_index]
        if parent_index >= 0:
            children[parent_index].append(joint_index)
    return children


def _write_end_site(f, t, offset):
    f.write("%sEnd Site\n" % t)
    f.write("%s{\n" % t)
    t += '\t'
    f.write("%sOFFSET %f %f %f\n" % (t, offset[0], offset[1], offset[2]))
    t = t[:-1]
    f.write("%s}\n" % t)


def load(filename, order=None, load_end_sites=False):
    """
    Load bvh motion data file.

    Args:
        filename: BVH filepath.
        order: Optional shared rotation order override.
        load_end_sites: When true, each BVH End Site is loaded as an extra joint.

    Returns:
        rotations: rotation in wxyz quaternion, be reminded scipy rotation use xyzw order
        positions: one joint's local position to its parent
        offsets: one joint's local position defined in hierarchy section, use positions instead
        end_sites: boolean mask for joints that originated from BVH End Site blocks
        parents: numpy array for one joint's parent index
        names: list of string for joint names
        frametime: time duration of a frame
    """
    i = 0
    active = -1
    end_site = False

    names = []
    offsets = []
    parents = []
    joint_channels = []
    joint_orders = []
    end_sites = []

    def add_joint(name, parent, offset=None, is_end_site=False):
        names.append(name)
        offsets.append([0.0, 0.0, 0.0] if offset is None else list(offset))
        parents.append(parent)
        joint_channels.append(())
        joint_orders.append(None)
        end_sites.append(is_end_site)
        return len(parents) - 1

    def next_end_site_name(parent_index):
        base_name = "%s_EndSite" % names[parent_index]
        name = base_name
        suffix = 1
        while name in names:
            name = "%s_%i" % (base_name, suffix)
            suffix += 1
        return name

    with open(filename, "r") as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue

            if stripped == "HIERARCHY" or stripped == "MOTION":
                continue

            rmatch = re.match(r'^\s*ROOT\s+(.+?)(?:\s*\{)?\s*$', line)
            if rmatch:
                active = add_joint(rmatch.group(1).strip(), active)
                continue

            if "{" in line:
                continue

            if "}" in line:
                if end_site:
                    if load_end_sites and active >= 0 and end_sites[active]:
                        active = parents[active]
                    end_site = False
                else:
                    active = parents[active]
                continue

            offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
            if offmatch:
                parsed_offset = list(map(float, offmatch.groups()))
                if end_site:
                    if load_end_sites:
                        active = add_joint(next_end_site_name(active), active, parsed_offset, is_end_site=True)
                    continue

                offsets[active] = parsed_offset
                continue

            chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
            if chanmatch:
                channels = int(chanmatch.group(1))
                parts = tuple(line.split()[2:2 + channels])
                joint_channels[active] = parts
                if order is None:
                    rotation_parts = [channelmap[p] for p in parts if p in channelmap]
                    if rotation_parts:
                        joint_orders[active] = "".join(rotation_parts)
                continue

            jmatch = re.match(r'^\s*JOINT\s+(.+?)(?:\s*\{)?\s*$', line)
            if jmatch:
                active = add_joint(jmatch.group(1).strip(), active)
                continue

            if "End Site" in line or "End site" in line:
                end_site = True
                continue

            fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
            if fmatch:
                fnum = int(fmatch.group(1))
                offset_array = np.asarray(offsets, dtype=np.float64)
                positions = offset_array[np.newaxis].repeat(fnum, axis=0)
                rotations = np.zeros((fnum, len(offsets), 3), dtype=np.float64)
                continue

            fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
            if fmatch:
                frametime = float(fmatch.group(1))
                continue

            data_block = np.array(list(map(float, stripped.split())))
            fi = i
            expected_values = sum(len(joint_channel_names) for joint_channel_names in joint_channels)
            if len(data_block) != expected_values:
                raise ValueError(
                    "Unexpected number of motion values in %s at frame %i: expected %i, got %i" % (
                        filename, fi, expected_values, len(data_block)))

            cursor = 0
            for ji, joint_channel_names in enumerate(joint_channels):
                joint_values = data_block[cursor:cursor + len(joint_channel_names)]
                cursor += len(joint_channel_names)

                joint_position = np.zeros(3)
                joint_position_mask = np.zeros(3, dtype=bool)
                joint_scale = np.ones(3)
                joint_scale_mask = np.zeros(3, dtype=bool)
                joint_rotation = []

                for channel_name, channel_value in zip(joint_channel_names, joint_values):
                    axis = ordermap[channel_name[0].lower()]
                    if channel_name.endswith('position'):
                        joint_position[axis] = channel_value
                        joint_position_mask[axis] = True
                    elif channel_name.endswith('rotation'):
                        joint_rotation.append(channel_value)
                    elif channel_name.endswith('scale'):
                        joint_scale[axis] = channel_value
                        joint_scale_mask[axis] = True
                    else:
                        raise ValueError("Unsupported BVH channel %s in %s" % (channel_name, filename))

                if joint_position_mask.any():
                    if ji != 0 and joint_scale_mask.any():
                        positions[fi, ji] = np.asarray(offsets[ji]) + joint_position * joint_scale
                    else:
                        positions[fi, ji, joint_position_mask] = joint_position[joint_position_mask]

                if joint_rotation:
                    rotations[fi, ji, :len(joint_rotation)] = joint_rotation

            i += 1

    if order is None:
        quaternions = eye(rotations.shape[:-1], dtype=rotations.dtype)
        for ji, joint_order in enumerate(joint_orders):
            if joint_order is not None:
                quaternions[:, ji] = from_euler(np.radians(rotations[:, ji]), order=joint_order)
    else:
        quaternions = from_euler(np.radians(rotations), order=order)

    return {
        'rotations': unroll(quaternions),
        'positions': positions,
        'offsets': np.asarray(offsets, dtype=np.float64),
        'end_sites': np.asarray(end_sites, dtype=bool),
        'parents': np.asarray(parents, dtype=int),
        'names': names,
        'frametime': frametime
    }


def save_joint(f, names, offsets, children, end_sites, t, i, save_order, order='zyx', save_positions=False, save_end_sites=False):
    save_order.append(i)

    f.write("%sJOINT %s\n" % (t, names[i]))
    f.write("%s{\n" % t)
    t += '\t'

    f.write("%sOFFSET %f %f %f\n" % (t, offsets[i,0], offsets[i,1], offsets[i,2]))

    if save_positions:
        f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % (
            t, channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))
    else:
        f.write("%sCHANNELS 3 %s %s %s\n" % (
            t, channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))

    wrote_child = False
    for child in children[i]:
        if save_end_sites and end_sites[child]:
            if children[child]:
                raise ValueError("End Site joint %s cannot have children" % names[child])
            _write_end_site(f, t, offsets[child])
            wrote_child = True
            continue

        t = save_joint(
            f, names, offsets, children, end_sites, t, child, save_order,
            order=order, save_positions=save_positions, save_end_sites=save_end_sites)
        wrote_child = True

    if not wrote_child:
        _write_end_site(f, t, np.zeros(3, dtype=offsets.dtype))

    t = t[:-1]
    f.write("%s}\n" % t)

    return t


def save(filename, data, save_positions=False, save_end_sites=False):
    order = 'zyx'
    frametime = data['frametime']
    names = data['names']
    offsets = np.asarray(data['offsets'])
    positions = np.asarray(data['positions'])
    rotations = np.asarray(data['rotations'])
    parents = np.asarray(data['parents'], dtype=int)
    end_sites = _end_site_mask(data)
    children = _children_by_parent(parents)

    if save_end_sites and end_sites[0]:
        raise ValueError("Root joint cannot be saved as End Site")

    with open(filename, 'w') as f:
        t = ""
        f.write("%sHIERARCHY\n" % t)
        f.write("%sROOT %s\n" % (t, names[0]))
        f.write("%s{\n" % t)
        t += '\t'

        f.write("%sOFFSET %f %f %f\n" % (t, offsets[0,0], offsets[0,1], offsets[0,2]))
        f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % (
            t, channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))

        save_order = [0]
        wrote_root_child = False
        for child in children[0]:
            if save_end_sites and end_sites[child]:
                if children[child]:
                    raise ValueError("End Site joint %s cannot have children" % names[child])
                _write_end_site(f, t, offsets[child])
                wrote_root_child = True
                continue

            t = save_joint(
                f, names, offsets, children, end_sites, t, child, save_order,
                order=order, save_positions=save_positions, save_end_sites=save_end_sites)
            wrote_root_child = True

        if not wrote_root_child:
            _write_end_site(f, t, np.zeros(3, dtype=offsets.dtype))

        t = t[:-1]
        f.write("%s}\n" % t)

        rots = 180.0 / np.pi * to_euler(rotations, order[::-1])

        f.write("MOTION\n")
        f.write("Frames: %i\n" % len(rots))
        f.write("Frame Time: %f\n" % frametime)

        for i in range(rots.shape[0]):
            for j in save_order:
                if save_positions or j == 0:
                    f.write("%f %f %f %f %f %f " % (
                        positions[i,j,0], positions[i,j,1], positions[i,j,2],
                        rots[i,j,ordermap[order[0]]], rots[i,j,ordermap[order[1]]], rots[i,j,ordermap[order[2]]]))
                else:
                    f.write("%f %f %f " % (
                        rots[i,j,ordermap[order[0]]], rots[i,j,ordermap[order[1]]], rots[i,j,ordermap[order[2]]]))

            f.write("\n")


def mirrored_motion(data):
    mirror_bones = []
    pos = data['positions'].copy()
    rot = data['rotations'].copy()
    end_sites = _end_site_mask(data)
    for ni, n in enumerate(data["names"]):
        if "Right" in n and n.replace("Right", "Left") in data["names"]:
            mirror_bones.append(data["names"].index(n.replace("Right", "Left")))
        elif "Left" in n and n.replace("Left", "Right") in data["names"]:
            mirror_bones.append(data["names"].index(n.replace("Left", "Right")))
        else:
            mirror_bones.append(ni)
    mirror_bones = np.array(mirror_bones)
    gloRot, gloPos = fk(rot, pos, data["parents"])
    gloPos = np.array([-1, 1, 1]) * gloPos[:, mirror_bones]
    gloRot = np.array([1, 1, -1, -1]) * gloRot[:, mirror_bones]
    rot, pos = ik(gloRot, gloPos, data["parents"])

    m_data = data.copy()
    m_data['rotations'] = rot.copy()
    m_data['positions'] = pos.copy()
    m_data['end_sites'] = end_sites[mirror_bones].copy()
    return m_data


def remove_joint(data, joint_name):
    """
    Remove the given joint and all its children if exists
    """
    nframes = data['rotations'].shape[0]
    njoints = len(data['names'])
    end_sites = _end_site_mask(data)
    children = []
    for i in range(njoints):
        children.append([])
        if data['parents'][i] != -1:
            children[data['parents'][i]].append(i)
    joint_idx = data['names'].index(joint_name)
    joints_to_be_removed = []
    s = [joint_idx]
    while len(s) > 0:
        last = s.pop()
        joints_to_be_removed.append(last)
        for c in children[last]:
            s.append(c)
    old_id_to_new = {}
    for i in range(njoints):
        if joints_to_be_removed.count(i) == 0:
            old_id_to_new[i] = len(old_id_to_new.items())
    rotations = np.zeros((nframes, njoints-len(joints_to_be_removed), 4))
    positions = np.zeros((nframes, njoints-len(joints_to_be_removed), 3))
    offsets = np.zeros((njoints-len(joints_to_be_removed), 3))
    new_end_sites = np.zeros(njoints-len(joints_to_be_removed), dtype=bool)
    names, parents = [], []
    for k, v in old_id_to_new.items():
        rotations[:,v] = data['rotations'][:,k]
        positions[:,v] = data['positions'][:,k]
        offsets[v] = data['offsets'][k]
        new_end_sites[v] = end_sites[k]
        names.append(data['names'][k])
        if data['parents'][k] == -1:
            parents.append(-1)
        else:
            parents.append(old_id_to_new[data['parents'][k]])
    return {
        'rotations': rotations,
        'positions': positions,
        'offsets': offsets,
        'end_sites': new_end_sites,
        'names': names,
        'parents': parents,
        'frametime': data['frametime']
    }


def resample_motion(data, target_fps=60):
    nframes = data['rotations'].shape[0]
    njoints = len(data['parents'])
    duration = nframes * data['frametime']
    target_frametime = 1.0 / target_fps
    target_nframes = int(np.floor(duration / target_frametime))
    rotations = np.zeros((target_nframes, njoints, 4))
    positions = np.zeros((target_nframes, njoints, 3))
    for i in range(target_nframes):
        timestamp = i * target_frametime
        start = np.clip(int(np.floor(timestamp / data['frametime'])), 0, nframes - 1)
        end = np.clip(start + 1, 0, nframes - 1)
        blending_alpha = np.clip((timestamp - start * data['frametime']) / data['frametime'], 0.0, 1.0)
        positions[i,:] = data['positions'][start,:] * (1.0 - blending_alpha) + data['positions'][end,:] * blending_alpha
        for j in range(njoints):
            if np.dot(data['rotations'][start,j], data['rotations'][end,j]) < 0:
                rotations[i,j] = data['rotations'][start,j] * (1.0 - blending_alpha) - data['rotations'][end,j] * blending_alpha
            else:
                rotations[i,j] = data['rotations'][start,j] * (1.0 - blending_alpha) + data['rotations'][end,j] * blending_alpha
            rotations[i,j] /= (np.linalg.norm(rotations[i,j]) + 1e-8)
    return {
        'rotations': rotations,
        'positions': positions,
        'offsets': data['offsets'],
        'end_sites': _end_site_mask(data).copy(),
        'names': data['names'],
        'parents': data['parents'],
        'frametime': target_frametime
    }
