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


def load(filename, order=None):
    """
    Load bvh motion data file.

    returns:
        rotations: rotation in wxyz quaternion, be reminded scipy rotation use xyzw order
        positions: one joint's local position to its parent
        offsets: one joint's local position defined in hierarchy section, use positions instead
        parents: numpy array for one joint's parent index
        names: list of string for joint names
        order: order for euler angles
        frametime: time duration of a frame
    """
    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = np.array([]).reshape((0, 4))
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    # Parse the  file, line by line
    for line in f:
        
        if line == '\n':
            continue
        
        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            fnum = int(fmatch.group(1))
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        # split by all posible intervals
        dmatch = line.strip().split()
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()

    return {
        'rotations': unroll(from_euler(np.radians(rotations), order=order)),
        'positions': positions,
        'offsets': offsets,
        'parents': parents,
        'names': names,
        'frametime': frametime
    }
    
    
def save_joint(f, data, t, i, save_order, order='zyx', save_positions=False):
    
    save_order.append(i)
    
    f.write("%sJOINT %s\n" % (t, data['names'][i]))
    f.write("%s{\n" % t)
    t += '\t'
  
    f.write("%sOFFSET %f %f %f\n" % (t, data['offsets'][i,0], data['offsets'][i,1], data['offsets'][i,2]))
    
    if save_positions:
        f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % (t, 
            channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))
    else:
        f.write("%sCHANNELS 3 %s %s %s\n" % (t, 
            channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))
    
    end_site = True
    
    for j in range(len(data['parents'])):
        if data['parents'][j] == i:
            t = save_joint(f, data, t, j, save_order, order=order, save_positions=save_positions)
            end_site = False
    
    if end_site:
        f.write("%sEnd Site\n" % t)
        f.write("%s{\n" % t)
        t += '\t'
        f.write("%sOFFSET %f %f %f\n" % (t, 0.0, 0.0, 0.0))
        t = t[:-1]
        f.write("%s}\n" % t)
  
    t = t[:-1]
    f.write("%s}\n" % t)
    
    return t
    

def save(filename, data, save_positions=False):
    order = 'zyx'
    frametime = data['frametime']
    
    with open(filename, 'w') as f:

        t = ""
        f.write("%sHIERARCHY\n" % t)
        f.write("%sROOT %s\n" % (t, data['names'][0]))
        f.write("%s{\n" % t)
        t += '\t'

        f.write("%sOFFSET %f %f %f\n" % (t, data['offsets'][0,0], data['offsets'][0,1], data['offsets'][0,2]) )
        f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % 
            (t, channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))

        save_order = [0]
            
        for i in range(len(data['parents'])):
            if data['parents'][i] == 0:
                t = save_joint(f, data, t, i, save_order, order=order, save_positions=save_positions)
      
        t = t[:-1]
        f.write("%s}\n" % t)

        rots, poss = 180.0/np.pi*to_euler(data['rotations'], order[::-1]) , data['positions']

        f.write("MOTION\n")
        f.write("Frames: %i\n" % len(rots));
        f.write("Frame Time: %f\n" % frametime);
        
        for i in range(rots.shape[0]):
            for j in save_order:
                
                if save_positions or j == 0:
                
                    f.write("%f %f %f %f %f %f " % (
                        poss[i,j,0],                  poss[i,j,1],                  poss[i,j,2], 
                        rots[i,j,ordermap[order[0]]], rots[i,j,ordermap[order[1]]], rots[i,j,ordermap[order[2]]]))
                
                else:
                    
                    f.write("%f %f %f " % (
                        rots[i,j,ordermap[order[0]]], rots[i,j,ordermap[order[1]]], rots[i,j,ordermap[order[2]]]))

            f.write("\n")



