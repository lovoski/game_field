"""FBX → BVH converter for the toolkit engine.

Conventions required by toolkit/loaders/bvh.cpp and toolkit/loaders/motion.cpp:
  • +Y up, meters
  • Root joint : CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation
  • Other joints: CHANNELS 3 Zrotation Yrotation Xrotation
  • End-effectors named <parentName>_End (or "End Site")
  • Parent index always < child index (standard BVH tree order)
"""

import os
import sys

import bpy # type: ignore


def convert(fbx_path: str, bvh_path: str, scale: float = 1.0) -> None:
    # ── 1. Fresh empty scene ───────────────────────────────────────────────
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # ── 2. Import FBX ──────────────────────────────────────────────────────
    # axis_forward/axis_up map the FBX coordinate frame (+Y up, -Z forward)
    # into Blender's internal +Z-up frame.  bake_space_transform ensures the
    # coordinate-system correction is baked into every bone channel so the
    # subsequent BVH export sees clean Y-up data.
    bpy.ops.import_scene.fbx(
        filepath=fbx_path,
        global_scale=scale,
        axis_forward="-Z",
        axis_up="Y",
        bake_space_transform=True,
        use_anim=True,
        use_custom_normals=False,
        use_image_search=False,
    )

    # ── 3. Locate the armature ─────────────────────────────────────────────
    arm_obj = next(
        (o for o in bpy.context.scene.objects if o.type == "ARMATURE"), None
    )
    if arm_obj is None:
        raise RuntimeError("No ARMATURE found in the imported FBX.")

    bpy.ops.object.select_all(action="DESELECT")
    arm_obj.select_set(True)
    bpy.context.view_layer.objects.active = arm_obj

    # ── 4. Set scene frame range from the action ───────────────────────────
    scene = bpy.context.scene
    if arm_obj.animation_data and arm_obj.animation_data.action:
        action = arm_obj.animation_data.action
        scene.frame_start = int(action.frame_range[0])
        scene.frame_end = int(action.frame_range[1])

    # ── 5. Export BVH ──────────────────────────────────────────────────────
    # rotate_mode='ZYX'  → CHANNELS … Zrotation Yrotation Xrotation
    #   matches the order the toolkit save() function writes and load()
    #   accepts (see toolkit/loaders/motion.cpp).
    # axis_forward='-Z', axis_up='Y'  → output is +Y-up, which is the
    #   standard BVH convention and what this engine assumes.
    # global_scale=1.0 because scale was already applied at import time.
    out_dir = os.path.dirname(os.path.abspath(bvh_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    bpy.ops.export_anim.bvh(
        filepath=bvh_path,
        global_scale=1.0,
        frame_start=scene.frame_start,
        frame_end=scene.frame_end,
        rotate_mode="ZYX",
        root_transform_only=False,
        axis_forward="-Z",
        axis_up="Y",
    )
    n_frames = scene.frame_end - scene.frame_start + 1
    print(f"[fbx_to_bvh] wrote '{bvh_path}'  ({n_frames} frames, scale={scale})")


if __name__ == "__main__":
    convert(
        r'',
        r'',
        scale=1.0,
    )