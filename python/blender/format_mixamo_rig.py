import bpy
import mathutils
import numpy

def midpoint(coordinates, mode):

    if len(coordinates) > 0:

        if mode == "BOUNDING_BOX":

            x= []
            y= []
            z= []

            for coordinate in coordinates:
                x.append(coordinate[0])
                y.append(coordinate[1])
                z.append(coordinate[2])

            range_x = (max(x), min(x))
            range_y = (max(y), min(y))
            range_z = (max(z), min(z))

            bounding_box_coordinate = []

            for a in range_x:
                for b in range_y:
                    for c in range_z:
                        bounding_box_coordinate.append((a, b, c))

            return mathutils.Vector(numpy.array(bounding_box_coordinate).mean(axis=0))

        if mode == "CENTER":
            return mathutils.Vector(numpy.array(coordinates).mean(axis=0))
    else:
        return None


def altitude(A, B, V):

    a = B-A
    b = B-B
    v = B-V

#    a = A
#    b = B
#    v = V

    x1 = a[0]
    y1 = a[1]
    z1 = a[2]

    x2 = b[0]
    y2 = b[1]
    z2 = b[2]

    x3 = v[0]
    y3 = v[1]
    z3 = v[2]

    w = (((x1 - x2) * (x3 - x2)) + ((y1 - y2) * (y3 - y2)) + ((z1 - z2) * (z3 - z2))) / (pow((x1-x2), 2) + pow((y1-y2), 2) + pow((z1-z2), 2))

    x = x2 + (w * (x2 - x1))
    y = y2 + (w * (y2 - y1))
    z = z2 + (w * (z2 - z1))

    a = mathutils.Vector((x, y, z))

    return a + B

def Calculate_Poll_Position(upper_bone, lower_bone, distance):


    up = upper_bone.head
    mid = lower_bone.head
    down = lower_bone.tail

    Altitude_Co = -altitude(up, down, mid)
    Vec = mathutils.Vector(mid) + Altitude_Co
    Mat = mathutils.Matrix.Translation(-Altitude_Co)

    pole_position = Mat @ (Vec*(distance + 1.5))


    return pole_position

def Copy_Rig(object, name):

    copy_rig = object.copy()
    copy_rig.display_type = "SOLID"
    copy_rig.show_in_front = True
    copy_rig.name = name
    copy_rig.data = object.data.copy()
    bpy.context.collection.objects.link(copy_rig)

    return copy_rig

def Extract_Rig(context, object, name ,Extract_Mode = "SELECTED", remove_animation_data=True, move_to_layer_1=True, flatten_hierarchy=False, disconnect_Bone=True, remove_custom_properties=True, remove_bbone = True, set_inherit_rotation = True, set_local_location=True, set_inherit_scale_full=True, remove_bone_shape = True, unlock_transform=True, Clear_Constraints=True):

    copy_rig = Copy_Rig(object, name)

    bpy.ops.object.mode_set(mode = 'OBJECT')

    bpy.ops.object.select_all(action='DESELECT')
    copy_rig.select_set(True)
    context.view_layer.objects.active = copy_rig
    bpy.ops.object.mode_set(mode = 'EDIT')

    Edit_Bones = copy_rig.data.edit_bones

    if remove_animation_data:

        copy_rig.animation_data_clear()
        copy_rig.data.animation_data_clear()

    if move_to_layer_1:

        for i, layer in enumerate(copy_rig.data.layers):
            if i == 0:
                copy_rig.data.layers[i] = True
            else:
                copy_rig.data.layers[i] = False

    for bone in Edit_Bones:

        if flatten_hierarchy:
            bone.parent = None
        if disconnect_Bone:
            bone.use_connect = False

        if remove_custom_properties:
            if bone.get("_RNA_UI"):
                for property in bone["_RNA_UI"]:
                    del bone[property]

        if remove_bbone:
            bone.bbone_segments = 0

        if set_inherit_rotation:
            bone.use_inherit_rotation = True

        if set_local_location:
             bone.use_local_location = True

        if set_inherit_scale_full:
             bone.inherit_scale = "FULL"

        if move_to_layer_1:
            for i, layer in enumerate(bone.layers):
                if i == 0:
                    bone.layers[i] = True
                else:
                    bone.layers[i] = False


        if Extract_Mode == "SELECTED":
            if not bone.select:
                Edit_Bones.remove(bone)

        if Extract_Mode == "DEFORM":
            if not bone.use_deform:
                Edit_Bones.remove(bone)

        if Extract_Mode == "SELECTED_DEFORM":
            if not bone.select:
                if not bone.use_deform:
                    Edit_Bones.remove(bone)

        if Extract_Mode == "DEFORM_AND_SELECTED":
            if not bone.use_deform and not bone.select:
                Edit_Bones.remove(bone)


    bpy.ops.object.mode_set(mode = 'POSE')
    copy_rig.data.bones.update()

    if remove_custom_properties:
        if copy_rig.get("_RNA_UI"):
            for property in copy_rig["_RNA_UI"]:
                del copy_rig[property]

        if copy_rig.data.get("_RNA_UI"):
            for property in copy_rig.data["_RNA_UI"]:
                del copy_rig.data[property]

    Pose_Bones = copy_rig.pose.bones

    for bone in Pose_Bones:

        if remove_custom_properties:
            if bone.get("_RNA_UI"):
                for property in bone["_RNA_UI"]:
                    del bone[property]

        if remove_bone_shape:
            bone.custom_shape = None

        if unlock_transform:
            bone.lock_location[0] = False
            bone.lock_location[1] = False
            bone.lock_location[2] = False

            bone.lock_scale[0] = False
            bone.lock_scale[1] = False
            bone.lock_scale[2] = False

            bone.lock_rotation_w = False
            bone.lock_rotation[0] = False
            bone.lock_rotation[1] = False
            bone.lock_rotation[2] = False

        if Clear_Constraints:
            for constraint in bone.constraints:
                bone.constraints.remove(constraint)

        # if self.Constraint_Type == "TRANSFORM":

    for bone in object.pose.bones:
        if copy_rig:

            if copy_rig.data.bones.get(bone.name):

                constraint = bone.constraints.new("COPY_TRANSFORMS")
                constraint.target = copy_rig
                constraint.subtarget = copy_rig.data.bones.get(bone.name).name

        # if self.Constraint_Type == "LOTROT":
        #     constraint = bone.constraints.new("COPY_LOCATION")
        #     constraint.target = object
        #     constraint.subtarget = object.data.bones.get(bone.name).name
        #
        #     constraint = bone.constraints.new("COPY_ROTATION")
        #     constraint.target = object
        #     constraint.subtarget = object.data.bones.get(bone.name).name
        #
        # if self.Constraint_Type == "NONE":
        #     pass

    bpy.ops.object.mode_set(mode = 'OBJECT')


def hang_tail(object, snap_this, snap_to):
    bones = object.data.edit_bones
    bone_snap_this = bones.get(snap_this)
    bone_snap_to = bones.get(snap_to)

    if bone_snap_this:
        bone_snap_this.use_connect = False
    
    if bone_snap_to:
        bone_snap_to.use_connect = False

    if bone_snap_this:
        distance = numpy.linalg.norm(bone_snap_this.head.xyz - bone_snap_this.tail.xyz)
        bone_snap_this.tail.x = bone_snap_this.head.x
        bone_snap_this.tail.y = bone_snap_this.head.y + distance
        bone_snap_this.tail.z = bone_snap_this.head.z 


def fix_roll(bones, bone_name, vector):

    bone = bones.get(bone_name)
    if bone:
        bone.align_roll(vector)


def execute(context):
    object = context.object
    context.view_layer.update()
    if object.type == "ARMATURE":
        bpy.ops.object.mode_set(mode='EDIT')
        bones = object.data.edit_bones
        pref = ''
        #detecting the prefix (all prefixes are followed by a semicolon)
        semicolon = bones[0].name.find(":")
        if semicolon != -1:
            pref = bones[0].name[0:semicolon+1]
        #removes the prefix. In the final tool this could be optional
        #although I think removing it doesn't have any downsides
        #Some characters only work if the prefix is removed
        if True:
            for bone in bones:
                bone.name = bone.name[semicolon+1:]
                pref = ''
        #left leg
        hang_tail(object, pref + 'LeftUpLeg', pref + 'LeftLeg')
        hang_tail(object, pref + 'LeftLeg', pref + 'LeftFoot')
        hang_tail(object, pref + 'LeftFoot', pref + 'LeftToeBase')
        hang_tail(object, pref + 'LeftToeBase', pref + 'LeftToe_End')
        hang_tail(object, pref + 'LeftToeBase', pref + 'LeftFootToeBase_End')
        hang_tail(object, pref + 'LeftToe_End', pref + 'LeftToe_End')
        #right leg
        hang_tail(object, pref + 'RightUpLeg', pref + 'RightLeg')
        hang_tail(object, pref + 'RightLeg', pref + 'RightFoot')
        hang_tail(object, pref + 'RightFoot', pref + 'RightToeBase')
        hang_tail(object, pref + 'RightToeBase', pref + 'RightToe_End')
        hang_tail(object, pref + 'RightToeBase', pref + 'RightFootToeBase_End')
        hang_tail(object, pref + 'RightToe_End', pref + 'RightToe_End')
        
        #fix hips bone to point up
        Hip_Bone = bones.get(pref + "Hips")
        if Hip_Bone:
            Spine_Bone = bones.get(pref + "Spine")
            if Spine_Bone:
                Hip_Bone.tail.y = Spine_Bone.head.y
            Hip_Bone.tail.z = Hip_Bone.head.z
            Hip_Bone.tail.x = 0
        
        #spine
        hang_tail(object, pref + 'Spine', pref + 'Spine1')
        hang_tail(object, pref + 'Spine1', pref + 'Spine2')
        #some rigs have a second neck bone 'Neck1' and some don't
        if bones.find('Neck1') != -1:
            hang_tail(object, pref + 'Neck', pref + 'Neck1')
            hang_tail(object, pref + 'Neck1', pref + 'Head')
        else:
            hang_tail(object, pref + 'Neck', pref + 'Head')
        hang_tail(object, pref + 'Head', pref + 'HeadTop_End')
        #left arm
        hang_tail(object, pref + 'LeftShoulder', pref + 'LeftArm')
        hang_tail(object, pref + 'LeftArm', pref + 'LeftForeArm')
        hang_tail(object, pref + 'LeftForeArm', pref + 'LeftHand')
        #Right arm
        hang_tail(object, pref + 'RightShoulder', pref + 'RightArm')
        hang_tail(object, pref + 'RightArm', pref + 'RightForeArm')
        hang_tail(object, pref + 'RightForeArm', pref + 'RightHand')
        
        #left arm finers
        hang_tail(object, pref + 'LeftHand', pref + 'LeftHandPinky1')
        hang_tail(object, pref + 'LeftHandPinky1', pref + 'LeftHandPinky2')
        hang_tail(object, pref + 'LeftHandPinky2', pref + 'LeftHandPinky3')
        hang_tail(object, pref + 'LeftHandPinky3', pref + 'LeftHandPinky4')
        hang_tail(object, pref + 'LeftHandRing1', pref + 'LeftHandRing2')
        hang_tail(object, pref + 'LeftHandRing2', pref + 'LeftHandRing3')
        hang_tail(object, pref + 'LeftHandRing3', pref + 'LeftHandRing4')
        hang_tail(object, pref + 'LeftHandMiddle1', pref + 'LeftHandMiddle2')
        hang_tail(object, pref + 'LeftHandMiddle2', pref + 'LeftHandMiddle3')
        hang_tail(object, pref + 'LeftHandMiddle3', pref + 'LeftHandMiddle4')
        hang_tail(object, pref + 'LeftHandIndex1', pref + 'LeftHandIndex2')
        hang_tail(object, pref + 'LeftHandIndex2', pref + 'LeftHandIndex3')
        hang_tail(object, pref + 'LeftHandIndex3', pref + 'LeftHandIndex4')
        hang_tail(object, pref + 'LeftHandThumb1', pref + 'LeftHandThumb2')
        hang_tail(object, pref + 'LeftHandThumb2', pref + 'LeftHandThumb3')
        hang_tail(object, pref + 'LeftHandThumb3', pref + 'LeftHandThumb4')
        #Right arm finers
        hang_tail(object, pref + 'RightHand', pref + 'RightHandPinky1')
        hang_tail(object, pref + 'RightHandPinky1', pref + 'RightHandPinky2')
        hang_tail(object, pref + 'RightHandPinky2', pref + 'RightHandPinky3')
        hang_tail(object, pref + 'RightHandPinky3', pref + 'RightHandPinky4')
        hang_tail(object, pref + 'RightHandRing1', pref + 'RightHandRing2')
        hang_tail(object, pref + 'RightHandRing2', pref + 'RightHandRing3')
        hang_tail(object, pref + 'RightHandRing3', pref + 'RightHandRing4')
        hang_tail(object, pref + 'RightHandMiddle1', pref + 'RightHandMiddle2')
        hang_tail(object, pref + 'RightHandMiddle2', pref + 'RightHandMiddle3')
        hang_tail(object, pref + 'RightHandMiddle3', pref + 'RightHandMiddle4')
        hang_tail(object, pref + 'RightHandIndex1', pref + 'RightHandIndex2')
        hang_tail(object, pref + 'RightHandIndex2', pref + 'RightHandIndex3')
        hang_tail(object, pref + 'RightHandIndex3', pref + 'RightHandIndex4')
        hang_tail(object, pref + 'RightHandThumb1', pref + 'RightHandThumb2')
        hang_tail(object, pref + 'RightHandThumb2', pref + 'RightHandThumb3')
        hang_tail(object, pref + 'RightHandThumb3', pref + 'RightHandThumb4')
        fix_roll(bones, pref + 'LeftUpLeg', [0,0,1])
        fix_roll(bones, pref + 'LeftLeg', [0,0,1])
        fix_roll(bones, pref + 'LeftFoot', [0,0,1])
        fix_roll(bones, pref + 'LeftToeBase', [0,1,0])
        fix_roll(bones, pref + 'LeftToe_End', [0,1,0])
        fix_roll(bones, pref + 'RightUpLeg', [0,0,1])
        fix_roll(bones, pref + 'RightLeg', [0,0,1])
        fix_roll(bones, pref + 'RightFoot', [0,0,1])
        fix_roll(bones, pref + 'RightToeBase', [0,1,0])
        fix_roll(bones, pref + 'RightToe_End', [0,1,0])
        fix_roll(bones, pref + 'LeftShoulder', [0,-1,0])
        fix_roll(bones, pref + 'LeftArm', [0,-1,0])
        fix_roll(bones, pref + 'LeftForeArm', [0,-1,0])
        fix_roll(bones, pref + 'LeftHand', [0,-1,0])
        fix_roll(bones, pref + 'RightShoulder', [0,-1,0])
        fix_roll(bones, pref + 'RightArm', [0,-1,0])
        fix_roll(bones, pref + 'RightForeArm', [0,-1,0])
        fix_roll(bones, pref + 'RightHand', [0,-1,0])
        #FIX ROLL FINGERS
        #Left
        fix_roll(bones, pref + 'LeftHandPinky1', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandPinky2', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandPinky3', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandPinky4', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandRing1', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandRing2', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandRing3', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandRing4', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandMiddle1', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandMiddle2', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandMiddle3', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandMiddle4', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandIndex1', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandIndex2', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandIndex3', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandIndex4', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandThumb1', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandThumb2', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandThumb3', [0,-1,0])
        fix_roll(bones, pref + 'LeftHandThumb4', [0,-1,0])
        #Right
        fix_roll(bones, pref + 'RightHandPinky1', [0,-1,0])
        fix_roll(bones, pref + 'RightHandPinky2', [0,-1,0])
        fix_roll(bones, pref + 'RightHandPinky3', [0,-1,0])
        fix_roll(bones, pref + 'RightHandPinky4', [0,-1,0])
        fix_roll(bones, pref + 'RightHandRing1', [0,-1,0])
        fix_roll(bones, pref + 'RightHandRing2', [0,-1,0])
        fix_roll(bones, pref + 'RightHandRing3', [0,-1,0])
        fix_roll(bones, pref + 'RightHandRing4', [0,-1,0])
        fix_roll(bones, pref + 'RightHandMiddle1', [0,-1,0])
        fix_roll(bones, pref + 'RightHandMiddle2', [0,-1,0])
        fix_roll(bones, pref + 'RightHandMiddle3', [0,-1,0])
        fix_roll(bones, pref + 'RightHandMiddle4', [0,-1,0])
        fix_roll(bones, pref + 'RightHandIndex1', [0,-1,0])
        fix_roll(bones, pref + 'RightHandIndex2', [0,-1,0])
        fix_roll(bones, pref + 'RightHandIndex3', [0,-1,0])
        fix_roll(bones, pref + 'RightHandIndex4', [0,-1,0])
        fix_roll(bones, pref + 'RightHandThumb1', [0,-1,0])
        fix_roll(bones, pref + 'RightHandThumb2', [0,-1,0])
        fix_roll(bones, pref + 'RightHandThumb3', [0,-1,0])
        fix_roll(bones, pref + 'RightHandThumb4', [0,-1,0])
        fix_roll(bones, pref + 'Hips', [0,0,1])

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.object.mode_set(mode='OBJECT')
    

execute(bpy.context)