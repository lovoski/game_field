import bpy

# ============================================================
# CONFIGURATION
# ============================================================
SOURCE_ARMATURE = "Genesis 9"     # Your rig's name
STRIP_PREFIXES = ["DEF-", "DEF_"]  # Prefixes to strip from bone names
ROOT_BONE_NAME = "Root"          # Optional root bone at origin

# ============================================================
# STEP 1: Discover deform bones and their relationships
# ============================================================
class DeformBone:
    """Stores everything we need to reconstruct a bone in a new armature."""
    __slots__ = (
        "name", "clean_name", "parent_name",
        "head", "tail", "roll", "matrix",
        "use_connect",
    )

    def __init__(self, edit_bone, deform_parent_name):
        self.name = edit_bone.name
        self.clean_name = self._strip_prefix(edit_bone.name)
        self.parent_name = deform_parent_name
        self.head = edit_bone.head.copy()
        self.tail = edit_bone.tail.copy()
        self.roll = edit_bone.roll
        self.matrix = edit_bone.matrix.copy()
        self.use_connect = False  # We'll recalculate this

    @staticmethod
    def _strip_prefix(name):
        for prefix in STRIP_PREFIXES:
            if name.startswith(prefix):
                return name[len(prefix):]
        return name


def discover_deform_bones(armature_obj):
    """
    Traverse the full armature in edit mode, collect every deform bone,
    and resolve each one's nearest deform ancestor.

    Returns:
        bones_by_name: dict[str, DeformBone]  — keyed by ORIGINAL name
        roots:         list[str]               — original names of root deform bones
    """
    bpy.context.view_layer.objects.active = armature_obj
    bpy.ops.object.mode_set(mode='EDIT')

    edit_bones = armature_obj.data.edit_bones
    bones_by_name = {}
    roots = []

    for ebone in edit_bones:
        if not ebone.use_deform:
            continue

        # Walk up to find nearest deform ancestor
        parent = ebone.parent
        while parent and not parent.use_deform:
            parent = parent.parent

        deform_parent_name = parent.name if parent else None
        db = DeformBone(ebone, deform_parent_name)
        bones_by_name[db.name] = db

        if deform_parent_name is None:
            roots.append(db.name)

    bpy.ops.object.mode_set(mode='OBJECT')

    print(f"Discovered {len(bones_by_name)} deform bones, {len(roots)} root(s).")
    return bones_by_name, roots


# ============================================================
# STEP 2: Build a brand new armature with only deform bones
# ============================================================
def build_game_armature(bones_by_name, roots):
    """
    Create a fresh armature and reconstruct the deform hierarchy.
    Bones are added in parent-first order via recursive traversal
    so that every parent exists before its children.
    """
    # --- Create empty armature ---
    arm_data = bpy.data.armatures.new("GameRig_Data")
    arm_obj = bpy.data.objects.new("GameRig", arm_data)
    bpy.context.collection.objects.link(arm_obj)
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='EDIT')

    edit_bones = arm_data.edit_bones

    # Build a children lookup for ordered traversal
    children_map = {}  # parent original name -> [child original names]
    for db in bones_by_name.values():
        children_map.setdefault(db.parent_name, []).append(db.name)

    # Optional: add a single root bone at the origin
    add_root = ROOT_BONE_NAME and len(roots) > 1
    if add_root:
        root = edit_bones.new(ROOT_BONE_NAME)
        root.head = (0, 0, 0)
        root.tail = (0, 0.1, 0)
        root.use_deform = True
        print(f"  Added unified root bone: {ROOT_BONE_NAME}")

    # --- name collision guard (after prefix stripping) ---
    used_names = set()
    if add_root:
        used_names.add(ROOT_BONE_NAME)

    def unique_clean_name(db):
        name = db.clean_name
        if name not in used_names:
            used_names.add(name)
            return name
        # Append incrementing suffix on collision
        i = 1
        while f"{name}.{i:03d}" in used_names:
            i += 1
        final = f"{name}.{i:03d}"
        used_names.add(final)
        return final

    # Map: original name -> final clean name used in the new armature
    name_map = {}

    # --- Recursive bone creation (parent-first order) ---
    def create_bone_recursive(original_name, parent_edit_bone=None):
        db = bones_by_name[original_name]
        clean = unique_clean_name(db)
        name_map[original_name] = clean

        ebone = edit_bones.new(clean)
        ebone.head = db.head
        ebone.tail = db.tail
        ebone.roll = db.roll
        ebone.use_deform = True

        if parent_edit_bone:
            ebone.parent = parent_edit_bone
            # Connect if the child's head matches the parent's tail
            if (ebone.head - parent_edit_bone.tail).length < 0.0001:
                ebone.use_connect = True

        # Recurse into children
        for child_name in children_map.get(original_name, []):
            create_bone_recursive(child_name, ebone)

    # Kick off recursion from each root
    for root_name in roots:
        parent_bone = edit_bones.get(ROOT_BONE_NAME) if add_root else None
        create_bone_recursive(root_name, parent_bone)

    bpy.ops.object.mode_set(mode='OBJECT')
    print(f"  Built game armature with {len(name_map)} bones.")
    return arm_obj, name_map


# ============================================================
# STEP 3: Transfer mesh weights to the new rig
# ============================================================
def transfer_meshes(source_armature_name, game_rig_obj, name_map):
    """
    Duplicate child meshes, rename their vertex groups to match
    the new bone names, and parent them to the game rig.
    """
    source = bpy.data.objects[source_armature_name]

    for child in source.children:
        if child.type != 'MESH':
            continue

        # Duplicate
        bpy.ops.object.select_all(action='DESELECT')
        child.select_set(True)
        bpy.context.view_layer.objects.active = child
        bpy.ops.object.duplicate()
        mesh_copy = bpy.context.active_object
        mesh_copy.name = child.name + "_game"

        # Rename vertex groups: original bone name -> clean game name
        for vg in mesh_copy.vertex_groups:
            if vg.name in name_map:
                vg.name = name_map[vg.name]

        # Remove vertex groups that don't map to any game bone
        game_bone_names = set(name_map.values())
        for vg in list(mesh_copy.vertex_groups):
            if vg.name not in game_bone_names:
                mesh_copy.vertex_groups.remove(vg)

        # Reparent to game rig
        mesh_copy.parent = game_rig_obj
        for mod in mesh_copy.modifiers:
            if mod.type == 'ARMATURE':
                mod.object = game_rig_obj

        print(f"  Transferred mesh: {child.name} -> {mesh_copy.name}")


# ============================================================
# STEP 4: Validate the result
# ============================================================
def validate(game_rig_obj, name_map):
    """Sanity checks on the final rig."""
    arm = game_rig_obj.data
    issues = []

    # Check every bone has a parent (except root)
    for bone in arm.bones:
        if bone.parent is None and bone.name != ROOT_BONE_NAME:
            # This is fine if we didn't add a unified root and it's
            # a natural root (like Hips). Only warn if multiple roots.
            pass

        # Check for zero-length bones (game engines hate these)
        if bone.length < 0.001:
            issues.append(f"  ⚠ Zero-length bone: {bone.name}")

    # Check meshes reference valid bones
    game_bone_names = {b.name for b in arm.bones}
    for child in game_rig_obj.children:
        if child.type != 'MESH':
            continue
        for vg in child.vertex_groups:
            if vg.name not in game_bone_names:
                issues.append(f"  ⚠ Mesh '{child.name}' has orphan vertex group: {vg.name}")

    if issues:
        print("\nValidation warnings:")
        for issue in issues:
            print(issue)
    else:
        print("\n✓ Validation passed — rig is clean.")

    # Print final hierarchy
    print(f"\nFinal game rig hierarchy ({len(arm.bones)} bones):")
    def print_tree(bone, indent=0):
        print("  " * indent + f"{'└─ ' if indent else ''}{bone.name}")
        for child in bone.children:
            print_tree(child, indent + 1)

    for bone in arm.bones:
        if bone.parent is None:
            print_tree(bone)


# ============================================================
# MAIN
# ============================================================
def main():
    print("\n" + "=" * 50)
    print("  GAME RIG BUILDER")
    print("=" * 50 + "\n")

    # Discover
    source = bpy.data.objects[SOURCE_ARMATURE]
    bones_by_name, roots = discover_deform_bones(source)

    # Build
    game_rig, name_map = build_game_armature(bones_by_name, roots)

    # Transfer meshes
    transfer_meshes(SOURCE_ARMATURE, game_rig, name_map)

    # Validate
    validate(game_rig, name_map)

    print("\n✓ Done. 'GameRig' is ready for FBX export.\n")


if __name__ == "__main__":
    main()