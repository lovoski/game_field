import numpy as np
from scipy.spatial.transform import Rotation
from utils.bvh_motion import Motion


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

            pos = 0.01 * bvhData["positions"][start:stop].copy().astype(np.float32)
            rot = quat.unroll(
                quat.from_euler(
                    np.radians(bvhData["rotations"][start:stop]), order=bvhData["order"]
                )
            )

            # First compute world space positions/rotations
            gloRot, gloPos = quat.fk(rot, pos, bvhData["parents"])

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

                gloRot, gloPos = quat.fk(rot, pos, bvhData["parents"])
                gloPos = np.array([-1, 1, 1]) * gloPos[:, mirror_bones]
                gloRot = np.array([1, 1, -1, -1]) * gloRot[:, mirror_bones]
                rot, pos = quat.ik(gloRot, gloPos, bvhData["parents"])

            # Specify joints to use for simulation bone
            simPosJoint = bvhData["names"].index("Spine2")
            simRotJoint = bvhData["names"].index("Hips")

            # Position comes from spine joint
            simPos = (
                np.array([1.0, 0.0, 1.0]) * gloPos[:, simPosJoint : simPosJoint + 1]
            )
            simPos = signal.savgol_filter(simPos, 31, 3, axis=0, mode="interp")

            # Direction comes from projected hip forward direction
            simDir = np.array([1.0, 0.0, 1.0]) * quat.mul_vec(
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
            simRot = quat.normalize(quat.between(np.array([0, 0, 1]), simDir))

            # Transform first joints to be local to sim and append sim as root bone
            pos[:, 0:1] = quat.mul_vec(quat.inv(simRot), pos[:, 0:1] - simPos)
            rot[:, 0:1] = quat.mul(quat.inv(simRot), rot[:, 0:1])

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
                * quat.to_scaled_angle_axis(quat.abs(quat.mul_inv(rot[2:], rot[1:-1])))
                * 60.0
                + 0.5
                * quat.to_scaled_angle_axis(quat.abs(quat.mul_inv(rot[1:-1], rot[:-2])))
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

    return dict(
        Ypos=Ypos,
        Yrot=Yrot,
        Yvel=Yvel,
        Yang=Yang,
        YrangeStarts=YrangeStarts,
        YrangeStops=YrangeStops,
        parents=parents,
        names=names,
    )


def compute_db_features(Y):
    Ypos = Y["Ypos"]
    Yrot = Y["Yrot"]
    Yvel = Y["Yvel"]
    Yang = Y["Yang"]
    YrangeStarts = Y["YrangeStarts"]
    YrangeStops = Y["YrangeStops"]
    parents = Y["parents"]
    names = Y["names"].tolist()

    posJoints = np.array([names.index(n) for n in ["LeftToeBase", "RightToeBase"]])
    velJoints = np.array(
        [names.index(n) for n in ["LeftToeBase", "RightToeBase", "Hips"]]
    )

    YgloRot, YgloPos, YgloAng, YgloVel = quat.fk_vel(Yrot, Ypos, Yang, Yvel, parents)
    YrootDir = quat.mul_vec(YgloRot[:, 0], np.array([0, 0, 1]))

    Xpos = quat.inv_mul_vec(
        YgloRot[:, 0:1], YgloPos[:, posJoints] - YgloPos[:, 0:1]
    ).reshape([len(Ypos), 6])
    Xvel = quat.inv_mul_vec(YgloRot[:, 0:1], YgloVel[:, velJoints]).reshape(
        [len(Ypos), 9]
    )

    XtrajPos = np.zeros([len(Ypos), 6])
    XtrajDir = np.zeros([len(Ypos), 6])
    for rs, re in zip(YrangeStarts, YrangeStops):
        ft0 = np.clip(np.arange(rs, re) + 20, rs, re - 1)
        ft1 = np.clip(np.arange(rs, re) + 40, rs, re - 1)
        ft2 = np.clip(np.arange(rs, re) + 60, rs, re - 1)

        XtrajPos[rs:re, 0:2] = quat.inv_mul_vec(
            YgloRot[rs:re, 0], YgloPos[ft0, 0] - YgloPos[rs:re, 0]
        )[:, np.array([0, 2])]
        XtrajPos[rs:re, 2:4] = quat.inv_mul_vec(
            YgloRot[rs:re, 0], YgloPos[ft1, 0] - YgloPos[rs:re, 0]
        )[:, np.array([0, 2])]
        XtrajPos[rs:re, 4:6] = quat.inv_mul_vec(
            YgloRot[rs:re, 0], YgloPos[ft2, 0] - YgloPos[rs:re, 0]
        )[:, np.array([0, 2])]

        XtrajDir[rs:re, 0:2] = quat.inv_mul_vec(YgloRot[rs:re, 0], YrootDir[ft0])[
            :, np.array([0, 2])
        ]
        XtrajDir[rs:re, 2:4] = quat.inv_mul_vec(YgloRot[rs:re, 0], YrootDir[ft1])[
            :, np.array([0, 2])
        ]
        XtrajDir[rs:re, 4:6] = quat.inv_mul_vec(YgloRot[rs:re, 0], YrootDir[ft2])[
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


if __name__ == "__main__":
    build_motion_db(
        [
            r"D:\repo\GenoViewPython-MotionMatching\resources\lafan_motion",
        ]
    )
