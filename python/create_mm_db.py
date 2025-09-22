import numpy as np
from utils import bvh
import scipy.signal as signal
from test import forward_kinematics_direct


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
            vel = np.empty_like(pos)
            vel[1:-1] = (
                0.5 * (pos[2:] - pos[1:-1]) * 60.0 + 0.5 * (pos[1:-1] - pos[:-2]) * 60.0
            )
            vel[0] = vel[1] - (vel[3] - vel[2])
            vel[-1] = vel[-2] + (vel[-2] - vel[-3])

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
    # files = [
    #     (
    #         r"/mnt/d/repo/GenoViewPython-MotionMatching/resources/lafan_retarget_to_smpl_processed/pushAndStumble1_subject5.bvh",
    #         397,
    #         706,
    #     ),
    #     (
    #         r"/mnt/d/repo/GenoViewPython-MotionMatching/resources/lafan_retarget_to_smpl_processed/run1_subject5.bvh",
    #         172,
    #         14136,
    #     ),
    #     (
    #         r"/mnt/d/repo/GenoViewPython-MotionMatching/resources/lafan_retarget_to_smpl_processed/walk1_subject5.bvh",
    #         160,
    #         15518,
    #     ),
    # ]
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


def extract_motion_trajectory(base_dir, output_dir):
    from scipy.spatial.transform import Rotation
    from tqdm import tqdm

    for file in tqdm(os.listdir(base_dir)):
        if not file.endswith(".bvh"):
            continue
        data = bvh.load(os.path.join(base_dir, file))
        gpos, grot = forward_kinematics_direct(
            data["positions"], data["rotations"], data["parents"]
        )
        root_pos = gpos[:, 0]
        root_rot = grot[:, 0]
        nframes = root_pos.shape[0]

        # we need:
        # 1. the direction at each frame as the simulated input for left joystick
        # 2. the root facing direction as the simulated input for right joystick

        root_pos = root_pos[1:-1].copy()
        root_pos[:, 1] = 0
        init_root_pos = root_pos[0].copy()
        root_pos -= init_root_pos
        root_vel = np.zeros_like(root_pos)
        root_vel[1:-1] = (
            0.5 * (root_pos[2:] - root_pos[1:-1]) * 60
            + 0.5 * (root_pos[1:-1] - root_pos[:-2]) * 60
        )
        root_vel[0] = root_vel[1] - (root_vel[3] - root_vel[2])
        root_vel[-1] = root_vel[-2] + (root_vel[-2] - root_vel[-3])
        root_facing = np.zeros((nframes - 2, 3))
        for f in range(1, nframes - 1):
            facing_dir = Rotation.from_quat(root_rot[f, [1, 2, 3, 0]]).apply(
                np.array([0, 0, 1])
            )
            facing_dir[1] = 0
            facing_dir /= np.linalg.norm(facing_dir) + 1e-5
            root_facing[f - 1] = facing_dir

        np.savez(
            os.path.join(output_dir, file.replace(".bvh", ".npz")),
            pos=root_pos.astype(np.float32),
            vel=root_vel.astype(np.float32),
            facing=root_facing.astype(np.float32),
        )


if __name__ == "__main__":
    import os

    # # base_dir = '/mnt/d/repo/GenoViewPython-MotionMatching/resources/persona_to_smpl_processed'
    # base_dir = 'data/InterAct/bvh'
    # mm_processed_dir = 'data/InterAct/mm_processed'
    # traj_output_dir = 'data/InterAct/extracted_traj'
    # os.makedirs(mm_processed_dir, exist_ok=True)
    # os.makedirs(traj_output_dir, exist_ok=True)
    # # process_interact_data(base_dir, mm_processed_dir)
    # # create_mm_db(mm_processed_dir)

    # extract_motion_trajectory(mm_processed_dir, traj_output_dir)

    base_dir = "data/lafan1/bvh"
    processed_dir = "data/lafan1/processed_bvh"
    traj_output_dir = "data/lafan1/traj_output"
    os.makedirs(traj_output_dir, exist_ok=True)
    extract_motion_trajectory(processed_dir, traj_output_dir)
    # os.makedirs(processed_dir, exist_ok=True)
    # from tqdm import tqdm
    # for file in tqdm(os.listdir(base_dir)):
    #     data = bvh.load(os.path.join(base_dir, file))
    #     data['offsets'][0] = 0
    #     data['offsets'] *= 0.01
    #     data['positions'] *= 0.01
    #     bvh.save(os.path.join(processed_dir, file), data)
