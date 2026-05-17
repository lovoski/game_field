import bpy  # type: ignore

NEW_ARMATURE_NAME = "Deform_Only_Armature"

source_armature = bpy.context.object
if source_armature is None or source_armature.type != "ARMATURE":
    raise Exception("Select the original armature, then run this script.")

if bpy.ops.object.mode_set.poll():
    bpy.ops.object.mode_set(mode="OBJECT")

deform_bones = [bone for bone in source_armature.data.bones if bone.use_deform]
deform_names = {bone.name for bone in deform_bones}

if not deform_bones:
    raise Exception(f"'{source_armature.name}' has no deform bones.")

bpy.ops.object.armature_add(enter_editmode=False)
new_armature = bpy.context.object
new_armature.name = NEW_ARMATURE_NAME
new_armature.data.name = NEW_ARMATURE_NAME
new_armature.matrix_world = source_armature.matrix_world.copy()
new_armature.show_in_front = True

bpy.ops.object.mode_set(mode="EDIT")

for bone in list(new_armature.data.edit_bones):
    new_armature.data.edit_bones.remove(bone)

for source_bone in deform_bones:
    new_bone = new_armature.data.edit_bones.new(source_bone.name)
    new_bone.head = source_bone.head_local
    new_bone.tail = source_bone.tail_local
    new_bone.use_deform = True

for source_bone in deform_bones:
    parent = source_bone.parent
    while parent and parent.name not in deform_names:
        parent = parent.parent

    if parent:
        new_bone = new_armature.data.edit_bones[source_bone.name]
        new_bone.parent = new_armature.data.edit_bones[parent.name]
        new_bone.use_connect = source_bone.use_connect and source_bone.parent == parent

bpy.ops.object.mode_set(mode="OBJECT")

print(f"Created '{new_armature.name}' with {len(deform_bones)} deform bones.")