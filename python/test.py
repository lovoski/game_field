import numpy as np
import matplotlib.pyplot as plt
from dataset.MotionPersona import format_motion_files_motion_persona
from utils.bvh_motion import Motion

src_dir = 'data/motion_persona/ZhiqingMom'
dst_dir = 'data/motion_persona/ZhiqingMom_processed'
format_motion_files_motion_persona(src_dir, dst_dir, 0.026524)

# motion = Motion.load_bvh(
#     "data/motion_persona/ZhiqingMom_processed/ZhiqingMom_Neutral_BR.bvh"
# )
# # for j in range(motion.joint_num):
# #     print(f'"{motion.names[j]}",')

# smpl_parents = [
#     -1,
#     0,
#     0,
#     0,
#     1,
#     2,
#     3,
#     4,
#     5,
#     6,
#     7,
#     8,
#     9,
#     9,
#     9,
#     12,
#     13,
#     14,
#     16,
#     17,
#     18,
#     19,
#     20,
#     21,
# ]
# smpl_names = [
#     "pelvis",
#     "left_hip",
#     "right_hip",
#     "spine1",
#     "left_knee",
#     "right_knee",
#     "spine2",
#     "left_ankle",
#     "right_ankle",
#     "spine3",
#     "left_foot",
#     "right_foot",
#     "neck",
#     "left_collar",
#     "right_collar",
#     "head",
#     "left_shoulder",
#     "right_shoulder",
#     "left_elbow",
#     "right_elbow",
#     "left_wrist",
#     "right_wrist",
#     "left_hand",
#     "right_hand",
# ]
# persona_parents = [
#     -1,
#     0,
#     1,
#     2,
#     3,
#     4,
#     5,
#     6,
#     4,
#     8,
#     9,
#     10,
#     4,
#     12,
#     13,
#     14,
#     0,
#     16,
#     17,
#     18,
#     0,
#     20,
#     21,
#     22,
# ]
# persona_names = [
#     "Hips",
#     "Spine",
#     "Spine1",
#     "Spine2",
#     "Spine3",
#     "Neck",
#     "Neck1",
#     "Head",
#     "RightShoulder",
#     "RightArm",
#     "RightForeArm",
#     "RightHand",
#     "LeftShoulder",
#     "LeftArm",
#     "LeftForeArm",
#     "LeftHand",
#     "RightUpLeg",
#     "RightLeg",
#     "RightFoot",
#     "RightToeBase",
#     "LeftUpLeg",
#     "LeftLeg",
#     "LeftFoot",
#     "LeftToeBase",
# ]

# # from smpl index to persona index
# smpl_to_persona = {}

# def collect_children(parents):
#   children = []
#   for j in range(len(parents)):
#     children.append([])
#     if parents[j] != -1:
#       children[parents[j]].append(j)
#   return children

# smpl_children = collect_children(smpl_parents)
# # print(smpl_children)
# # for j, children in enumerate(smpl_children):
# #   if len(children) == 0:
# #     print(f'smpl: {smpl_names[j]}, {j}')
# persona_children = collect_children(persona_parents)
# # print(persona_children)
# # for j, children in enumerate(persona_children):
# #   if len(children) == 0:
# #     print(f'persona: {persona_names[j]}, {j}')

# ee_pairs = [(10,23),(11,19),(15,7),(22,15),(23,11)]
# for smpl_ee, persona_ee in ee_pairs:
#   cur0, cur1 = smpl_ee, persona_ee
#   while cur0 != -1 and cur1 != -1:
#     smpl_to_persona[cur0] = cur1
#     cur0 = smpl_parents[cur0]
#     cur1 = persona_parents[cur1]

# for smpl_j, persona_j in smpl_to_persona.items():
#   print(f'{smpl_names[smpl_j]}, {persona_names[persona_j]}')
# print(smpl_to_persona)