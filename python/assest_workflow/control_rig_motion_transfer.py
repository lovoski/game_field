import bpy
from mathutils import Matrix

# ============================================================
# CONFIGURATION
# ============================================================
SOURCE_ARMATURE = "Armature"       # Control rig with animations
TARGET_ARMATURE = "GameRig"        # Your manually repaired game rig
FRAME_START = 1
FRAME_END = 250                    # Adjust to your animation range
ACTION_NAME = "GameRig_Action"     # Name for the new baked action

# ============================================================
# STEP 1: Sample world-space bone transforms from the source rig
# ============================================================
def sample_source_transforms(source_obj, frame_start, frame_end):
    """
    For every deform bone on the source rig, evaluate its final
    world-space matrix at every frame. This captures the result of
    ALL constraints, drivers, IK, etc. — the actual visual pose.

    Returns:
        dict[bone_name] -> list[(frame, world_matrix)]
    """
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    samples = {}

    # Collect deform bone names
    deform_names = [
        b.name for b in source_obj.pose.bones
        if source_obj.data.bones[b.name].use_deform
    ]

    print(f"  Sampling {len(deform_names)} bones over frames {frame_start}-{frame_end}...")

    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        depsgraph.update()

        # Get the evaluated (post-constraint) armature
        source_eval = source_obj.evaluated_get(depsgraph)

        for bone_name in deform_names:
            pbone = source_eval.pose.bones[bone_name]

            # World-space matrix = object transform * pose bone matrix
            world_matrix = source_eval.matrix_world @ pbone.matrix

            samples.setdefault(bone_name, []).append((frame, world_matrix.copy()))

    print(f"  Sampled {len(samples)} bones × {frame_end - frame_start + 1} frames.")
    return samples


# ============================================================
# STEP 2: Apply world-space transforms to the target rig
# ============================================================
def apply_to_target(target_obj, samples, frame_start, frame_end):
    """
    For each frame, convert the sampled world-space matrix into
    the target bone's LOCAL space (respecting the game rig's
    potentially different hierarchy) and set a keyframe.

    This is where the hierarchy difference is handled:
    same world-space result, different local-space decomposition.
    """
    scene = bpy.context.scene
    bpy.context.view_layer.objects.active = target_obj

    # Create a new action
    action = bpy.data.actions.new(name=ACTION_NAME)
    if not target_obj.animation_data:
        target_obj.animation_data_create()
    target_obj.animation_data.action = action

    target_world_inv = target_obj.matrix_world.inverted()

    # Build rest-pose matrices for the target rig
    # (needed to convert from pose-space to bone-local-space)
    rest_matrices = {}
    for pbone in target_obj.pose.bones:
        rest_matrices[pbone.name] = pbone.bone.matrix_local.copy()

    matched = 0
    skipped = []

    for bone_name, frame_data in samples.items():
        # Find matching bone on target rig
        pbone = target_obj.pose.bones.get(bone_name)
        if not pbone:
            skipped.append(bone_name)
            continue

        matched += 1
        rest_local = rest_matrices[bone_name]
        rest_local_inv = rest_local.inverted()

        # Parent's rest matrix in armature space (identity if root)
        if pbone.parent:
            parent_rest = rest_matrices[pbone.parent.name]
        else:
            parent_rest = Matrix.Identity(4)
        parent_rest_inv = parent_rest.inverted()

        for frame, world_matrix in frame_data:
            scene.frame_set(frame)

            # World -> armature space (object-local)
            armature_space = target_world_inv @ world_matrix

            # Armature space -> bone local space
            # local = rest_local_inv @ parent_rest @ parent_rest_inv @ armature_space
            # Simplified: factor out the parent's rest contribution
            #
            # The pose matrix in Blender is:
            #   armature_space = parent_rest @ parent_pose @ rest_offset
            # So to recover the pose:
            #   pose = parent_rest_inv @ armature_space @ rest_local_inv^-1
            # But we need it relative to the parent's CURRENT pose, not rest.
            #
            # Simplest correct approach: set the matrix directly and let
            # Blender decompose it.

            pbone.matrix = armature_space
            bpy.context.view_layer.update()

            # Now keyframe the resulting loc/rot/scale
            pbone.keyframe_insert(data_path="location", frame=frame)
            pbone.keyframe_insert(data_path="rotation_quaternion", frame=frame)
            pbone.keyframe_insert(data_path="rotation_euler", frame=frame)
            pbone.keyframe_insert(data_path="scale", frame=frame)

    if skipped:
        print(f"\n  ⚠ Bones on source but not on target ({len(skipped)}):")
        for name in skipped[:10]:
            print(f"      {name}")
        if len(skipped) > 10:
            print(f"      ... and {len(skipped) - 10} more")

    print(f"  Transferred animation to {matched} bones.")
    return action


# ============================================================
# STEP 3: Clean up the baked curves
# ============================================================
def clean_curves(action, threshold=0.001):
    """
    Remove redundant keyframes where the value doesn't change.
    Keeps the animation data lean for game engines.
    """
    removed = 0
    for fcurve in action.fcurves:
        keyframes = fcurve.keyframe_points
        if len(keyframes) < 3:
            continue

        # Walk backwards so removal doesn't shift indices
        for i in range(len(keyframes) - 2, 0, -1):
            prev_val = keyframes[i - 1].co[1]
            curr_val = keyframes[i].co[1]
            next_val = keyframes[i + 1].co[1]

            # If this key is between two keys with the same value, remove it
            if abs(curr_val - prev_val) < threshold and abs(curr_val - next_val) < threshold:
                keyframes.remove(keyframes[i])
                removed += 1

    print(f"  Cleaned {removed} redundant keyframes.")


# ============================================================
# MAIN
# ============================================================
def main():
    print("\n" + "=" * 50)
    print("  ANIMATION TRANSFER: Control Rig → Game Rig")
    print("=" * 50 + "\n")

    source = bpy.data.objects[SOURCE_ARMATURE]
    target = bpy.data.objects[TARGET_ARMATURE]

    # Ensure both are visible and selectable
    source.hide_set(False)
    target.hide_set(False)

    print("[1/3] Sampling source rig transforms...")
    samples = sample_source_transforms(source, FRAME_START, FRAME_END)

    print("\n[2/3] Applying to game rig...")
    action = apply_to_target(target, samples, FRAME_START, FRAME_END)

    print("\n[3/3] Cleaning curves...")
    clean_curves(action)

    print(f"\n✓ Done. Action '{ACTION_NAME}' is ready on '{TARGET_ARMATURE}'.\n")


if __name__ == "__main__":
    main()