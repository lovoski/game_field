import os
import bvh
import numpy as np
import scipy.signal as signal


def build_motion_db(files):
    Ypos = []
    Yrot = []
    Yvel = []
    Yang = []
    YrangeStarts = []
    YrangeStops = []

    for filename, start, stop in files:

        for mirrored in [True, False]:

            bvhData = bvh.load(filename)

            pos = bvhData["positions"][start:stop].copy()
            rot = bvhData["rotations"][start:stop].copy()

            # First compute world space positions/rotations
            gloRot, gloPos = bvh.fk(rot, pos, bvhData["parents"])

            if mirrored:

                mirror_bones = []
                for ni, n in enumerate(bvhData["names"]):
                    if "Right" in n and n.replace("Right", "Left") in bvhData["names"]:
                        mirror_bones.append(
                            bvhData["names"].index(n.replace("Right", "Left"))
                        )
                    elif "Left" in n and n.replace("Left", "Right") in bvhData["names"]:
                        mirror_bones.append(
                            bvhData["names"].index(n.replace("Left", "Right"))
                        )
                    else:
                        mirror_bones.append(ni)

                mirror_bones = np.array(mirror_bones)

                gloRot, gloPos = bvh.fk(rot, pos, bvhData["parents"])
                gloPos = np.array([-1, 1, 1]) * gloPos[:, mirror_bones]
                gloRot = np.array([1, 1, -1, -1]) * gloRot[:, mirror_bones]
                rot, pos = bvh.ik(gloRot, gloPos, bvhData["parents"])

            # Specify joints to use for simulation bone
            simPosJoint = bvhData["names"].index("Spine1")
            simRotJoint = bvhData["names"].index("Hips")

            # Position comes from spine joint
            simPos = (
                np.array([1.0, 0.0, 1.0]) * gloPos[:, simPosJoint : simPosJoint + 1]
            )
            simPos = signal.savgol_filter(simPos, 31, 3, axis=0, mode="interp")

            # Direction comes from projected hip forward direction
            simDir = np.array([1.0, 0.0, 1.0]) * bvh.mul_vec(
                gloRot[:, simRotJoint : simRotJoint + 1], np.array([0.0, 0.0, 1.0])
            )

            # We need to re-normalize the direction after both projection and smoothing
            simDir = (
                simDir / np.sqrt(np.sum(np.square(simDir), axis=-1))[..., np.newaxis]
            )
            simDir = signal.savgol_filter(simDir, 61, 3, axis=0, mode="interp")
            simDir = simDir / np.sqrt(
                np.sum(np.square(simDir), axis=-1)[..., np.newaxis]
            )

            # Extract rotation from direction
            simRot = bvh.normalize(bvh.between(np.array([0, 0, 1]), simDir))

            # Transform first joints to be local to sim and append sim as root bone
            pos[:, 0:1] = bvh.mul_vec(bvh.inv(simRot), pos[:, 0:1] - simPos)
            rot[:, 0:1] = bvh.mul(bvh.inv(simRot), rot[:, 0:1])

            pos = np.concatenate([simPos, pos], axis=1)
            rot = np.concatenate([simRot, rot], axis=1)

            parents = np.concatenate([[-1], bvhData["parents"] + 1])
            names = np.array(["Simulation"] + bvhData["names"])

            # Compute velocities via central difference
            # vel = np.empty_like(pos)
            # vel[1:-1] = (
            #     0.5 * (pos[2:] - pos[1:-1]) * 60.0 + 0.5 * (pos[1:-1] - pos[:-2]) * 60.0
            # )
            # vel[0] = vel[1] - (vel[3] - vel[2])
            # vel[-1] = vel[-2] + (vel[-2] - vel[-3])

            # directly use position difference as velocity
            vel = np.empty_like(pos)
            vel[:-1] = pos[1:]-pos[:-1]
            vel[-1] = vel[-2]
            vel *= 60

            # Same for angular velocities
            ang = np.zeros_like(pos)
            ang[1:-1] = (
                0.5
                * bvh.to_scaled_angle_axis(bvh.abs(bvh.mul_inv(rot[2:], rot[1:-1])))
                * 60.0
                + 0.5
                * bvh.to_scaled_angle_axis(bvh.abs(bvh.mul_inv(rot[1:-1], rot[:-2])))
                * 60.0
            )
            ang[0] = ang[1] - (ang[3] - ang[2])
            ang[-1] = ang[-2] + (ang[-2] - ang[-3])

            # Append to Database

            Ypos.append(pos)
            Yvel.append(vel)
            Yrot.append(rot)
            Yang.append(ang)

            offset = 0 if len(YrangeStarts) == 0 else YrangeStops[-1]

            YrangeStarts.append(offset)
            YrangeStops.append(offset + len(pos))

    Ypos = np.concatenate(Ypos, axis=0)
    Yrot = np.concatenate(Yrot, axis=0)
    Yvel = np.concatenate(Yvel, axis=0)
    Yang = np.concatenate(Yang, axis=0)

    YrangeStarts = np.array(YrangeStarts)
    YrangeStops = np.array(YrangeStops)

    return (
        Ypos,
        Yrot,
        Yvel,
        Yang,
        YrangeStarts.astype(np.int32),
        YrangeStops.astype(np.int32),
        parents.astype(np.int32),
        names.tolist(),
    )


def compute_db_features(
    Ypos, Yrot, Yvel, Yang, YrangeStarts, YrangeStops, parents, names
):
    posJoints = np.array([names.index(n) for n in ["LeftToeBase", "RightToeBase"]])
    velJoints = np.array(
        [names.index(n) for n in ["LeftToeBase", "RightToeBase", "Hips"]]
    )

    YgloRot, YgloPos, YgloAng, YgloVel = bvh.fk_vel(Yrot, Ypos, Yang, Yvel, parents)
    YrootDir = bvh.mul_vec(YgloRot[:, 0], np.array([0, 0, 1]))

    Xpos = bvh.inv_mul_vec(
        YgloRot[:, 0:1], YgloPos[:, posJoints] - YgloPos[:, 0:1]
    ).reshape([len(Ypos), 6])
    Xvel = bvh.inv_mul_vec(YgloRot[:, 0:1], YgloVel[:, velJoints]).reshape(
        [len(Ypos), 9]
    )

    XtrajPos = np.zeros([len(Ypos), 6])
    XtrajDir = np.zeros([len(Ypos), 6])
    for rs, re in zip(YrangeStarts, YrangeStops):
        ft0 = np.clip(np.arange(rs, re) + 20, rs, re - 1)
        ft1 = np.clip(np.arange(rs, re) + 40, rs, re - 1)
        ft2 = np.clip(np.arange(rs, re) + 60, rs, re - 1)

        XtrajPos[rs:re, 0:2] = bvh.inv_mul_vec(
            YgloRot[rs:re, 0], YgloPos[ft0, 0] - YgloPos[rs:re, 0]
        )[:, np.array([0, 2])]
        XtrajPos[rs:re, 2:4] = bvh.inv_mul_vec(
            YgloRot[rs:re, 0], YgloPos[ft1, 0] - YgloPos[rs:re, 0]
        )[:, np.array([0, 2])]
        XtrajPos[rs:re, 4:6] = bvh.inv_mul_vec(
            YgloRot[rs:re, 0], YgloPos[ft2, 0] - YgloPos[rs:re, 0]
        )[:, np.array([0, 2])]

        XtrajDir[rs:re, 0:2] = bvh.inv_mul_vec(YgloRot[rs:re, 0], YrootDir[ft0])[
            :, np.array([0, 2])
        ]
        XtrajDir[rs:re, 2:4] = bvh.inv_mul_vec(YgloRot[rs:re, 0], YrootDir[ft1])[
            :, np.array([0, 2])
        ]
        XtrajDir[rs:re, 4:6] = bvh.inv_mul_vec(YgloRot[rs:re, 0], YrootDir[ft2])[
            :, np.array([0, 2])
        ]

    X = np.concatenate([Xpos, Xvel, XtrajPos, XtrajDir], axis=-1)

    Xoffset = X.mean(axis=0)

    Xscale = np.concatenate(
        [
            Xpos.std(axis=0).mean().repeat(Xpos.shape[1]),
            Xvel.std(axis=0).mean().repeat(Xvel.shape[1]),
            XtrajPos.std(axis=0).mean().repeat(XtrajPos.shape[1]),
            XtrajDir.std(axis=0).mean().repeat(XtrajDir.shape[1]),
        ],
        axis=-1,
    )

    X = (X - Xoffset) / Xscale

    return X, Xoffset, Xscale


def create_mm_db(base_dir):
    files = []
    for fn in os.listdir(base_dir):
        if not fn.endswith(".bvh"):
            continue
        files.append((os.path.join(base_dir, fn), 0, -1))
    Ypos, Yrot, Yvel, Yang, YrangeStarts, YrangeStops, parents, names = build_motion_db(
        files
    )
    X, Xoffset, Xscale = compute_db_features(
        Ypos, Yrot, Yvel, Yang, YrangeStarts, YrangeStops, parents, names
    )
    with open("mapping.json", "w") as f:
        import json

        mapping = {}
        for idx, name in enumerate(names):
            mapping[name] = idx
        json.dump(mapping, f)

    np.savez(
        "db.npz",
        Ypos=Ypos.astype(np.float32),
        Yrot=Yrot.astype(np.float32),
        Yvel=Yvel.astype(np.float32),
        Yang=Yang.astype(np.float32),
        YrangeStarts=YrangeStarts,
        YrangeStops=YrangeStops,
        parents=parents,
        X=X.astype(np.float32),
        Xoffset=Xoffset.astype(np.float32),
        Xscale=Xscale.astype(np.float32),
    )


def process_interact_data(base_dir, output_dir):
    for file in os.listdir(base_dir):
        if not file.endswith(".bvh"):
            continue
        data = bvh.load(os.path.join(base_dir, file))
        data["frametime"] = 1.0 / 60.0
        data["positions"] = 0.01 * data["positions"]
        data["offsets"] = 0.01 * data["offsets"]
        nframes, njoints = data["rotations"].shape[0:2]
        rotations = np.zeros((nframes * 2, njoints, 4), dtype=data["rotations"].dtype)
        positions = np.zeros((nframes * 2, njoints, 3), dtype=data["positions"].dtype)

        for f in range(1, nframes):
            rotations[2 * f - 2] = data["rotations"][f - 1]
            for j in range(njoints):
                start_rot, end_rot = (
                    data["rotations"][f - 1, j],
                    data["rotations"][f, j],
                )
                if np.dot(start_rot, end_rot) < 0.0:
                    end_rot *= -1
                rotations[2 * f - 1, j] = 0.5 * start_rot + 0.5 * end_rot
                rotations[2 * f - 1, j] /= np.linalg.norm(rotations[2 * f - 1, j])
            rotations[2 * f] = data["rotations"][f]

            positions[2 * f - 2] = data["positions"][f - 1]
            positions[2 * f - 1] = (
                data["positions"][f - 1] * 0.5 + data["positions"][f] * 0.5
            )
            positions[2 * f] = data["positions"][f]

        positions[1] = positions[2]
        rotations[1] = rotations[2]

        data["offsets"][0] = 0
        data["rotations"] = rotations
        data["positions"] = positions

        bvh.save(os.path.join(output_dir, file), data)


def process_lafan1_data():
    files = [
        (
            "motion_matching/data/lafan1_loco/pushAndStumble1_subject5.bvh",
            397,
            706,
        ),
        (
            "motion_matching/data/lafan1_loco/run1_subject5.bvh",
            172,
            14136,
        ),
        (
            "motion_matching/data/lafan1_loco/walk1_subject5.bvh",
            160,
            15518,
        ),
    ]
    os.makedirs('motion_matching/data/_lafan1_loco', exist_ok=True)
    for file, start, end in files:
        if not file.endswith(".bvh"):
            continue
        data = bvh.resample_motion(bvh.load(file), 60.0)
        data["positions"] = 0.01 * data["positions"]
        data["offsets"] = 0.01 * data["offsets"]
        data['offsets'][0] = 0
        data['positions'] = data['positions'][start:end]
        data['rotations'] = data['rotations'][start:end]
        bvh.save(os.path.join('motion_matching/data/_lafan1_loco', os.path.basename(file)), data)


def extract_motion_trajectory(base_dir, output_dir):
    from scipy.spatial.transform import Rotation
    from tqdm import tqdm

    for file in tqdm(os.listdir(base_dir)):
        if not file.endswith(".bvh"):
            continue
        data = bvh.load(os.path.join(base_dir, file))
        grot, gpos = bvh.fk(
            data["positions"], data["rotations"], data["parents"]
        )
        root_pos = gpos[:, 0]
        root_rot = grot[:, 0]
        nframes = root_pos.shape[0]

        # we need:
        # 1. the direction at each frame as the simulated input for left joystick
        # 2. the root facing direction as the simulated input for right joystick

        root_pos = np.array([1, 0, 1]) * root_pos
        root_facing = np.zeros((nframes, 3))
        for f in range(nframes):
            facing_dir = Rotation.from_quat(root_rot[f, [1, 2, 3, 0]]).apply(
                np.array([0, 0, 1])
            )
            facing_dir[1] = 0
            facing_dir /= (np.linalg.norm(facing_dir) + 1e-5)
            root_facing[f] = facing_dir
        root_pos = signal.savgol_filter(root_pos, 31, 3, axis=0, mode='interp')
        root_facing = signal.savgol_filter(root_facing, 61, 3, axis=0, mode='interp')
        root_facing = root_facing / np.linalg.norm(root_facing, axis=1, keepdims=True)

        with open(os.path.join(output_dir, file.replace('.bvh', '.txt')), 'w') as f:
            for frame in range(root_pos.shape[0]):
                f.write(f'{root_pos[frame, 0]:.3f} {root_pos[frame, 1]:.3f} {root_pos[frame, 2]:.3f} {root_facing[frame, 0]:.3f} {root_facing[frame, 1]:.3f} {root_facing[frame, 2]:.3f}\n')


if __name__ == "__main__":
    process_lafan1_data()
