import pickle
import numpy as np
import bpy
import json
import os

from bpy_extras.io_utils import (
    axis_conversion,
)
from mathutils import Matrix, Euler
from math import degrees


def add_floor(size):
    bpy.ops.mesh.primitive_plane_add(size=size, enter_editmode=False, location=(0, 0, -0.05))
    floor = bpy.context.object
    floor.name = 'floor'

    floor_mat = bpy.data.materials.new(name="floorMaterial")
    floor_mat.use_nodes = True
    bsdf = floor_mat.node_tree.nodes["Principled BSDF"]
    floor_text = floor_mat.node_tree.nodes.new("ShaderNodeTexChecker")
    floor_text.inputs[3].default_value = 100
    floor_mat.node_tree.links.new(bsdf.inputs['Base Color'], floor_text.outputs['Color'])
    floor_mat.node_tree.nodes["Checker Texture"].inputs[1].default_value = (0.637593, 0.597203, 0.558341, 1)
    floor_mat.node_tree.nodes["Checker Texture"].inputs[2].default_value = (0.287439, 0.274678, 0.254152, 1)
    floor.data.materials.append(floor_mat)
    return floor


def add_camera(location, rotation):
    bpy.ops.object.camera_add(enter_editmode=False, align='VIEW', location=location, rotation=rotation)
    camera = bpy.context.object
    camera.data.lens = 50
    return camera


def add_point_light(location):
    bpy.ops.object.light_add(type='POINT', location=location, rotation=(0, 0, 0))
    sun = bpy.context.object
    sun.data.energy = 100
    return sun

def add_area_light(location, rotation):
    bpy.ops.object.light_add(type='AREA', location=location, rotation=rotation)
    light = bpy.context.object
    light.data.energy = 300
    light.data.size = 10
    return light

def clean_blocks():
    bpy.ops.ptcache.free_bake_all()
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    bpy.ops.outliner.orphans_purge()
    bpy.data.batch_remove(bpy.data.cache_files)
    for c in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.unlink(c)
    for c in bpy.data.collections:
        bpy.data.collections.remove(c)
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in bpy.data.materials:
        if block.users == 0:
            bpy.data.materials.remove(block)
    for block in bpy.data.textures:
        if block.users == 0:
            bpy.data.textures.remove(block)
    for block in bpy.data.images:
        if block.users == 0:
            bpy.data.images.remove(block)


shape_folder = r'D:\0tasks\000MotionPersona\user_study\conditions'
bvh_folder = r'D:\0tasks\000MotionPersona\user_study\lmp_processed'
output_folder = r'D:\0tasks\000MotionPersona\user_study\lmp_render'

bvh_files = sorted(os.listdir(bvh_folder))

for bvh_file in bvh_files[:50]:
    clean_blocks()
    
    if not bvh_file.endswith('.bvh'):
        continue

    bvh_file_path = os.path.join(bvh_folder, bvh_file)
    test_name = bvh_file.split('.')[0]
    shape_file_path = os.path.join(shape_folder, test_name, 'shape_%s.json' % test_name.split('_')[-1])
    output_path = os.path.join(output_folder, bvh_file.split('.')[0] + '_')
    
    shape_latent = json.load(open(shape_file_path))['latent']

    # import smplx
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.window_managers["WinMan"].smplx_tool.smplx_gender = 'neutral'
    bpy.data.window_managers["WinMan"].smplx_tool.smplx_handpose = 'flat'
    bpy.ops.scene.smplx_add_gender()
    bpy.ops.object.smplx_set_handpose()

    obj = bpy.context.object
    obj.data.shape_keys.key_blocks['Shape000'].slider_min = -10
    obj.data.shape_keys.key_blocks['Shape000'].slider_max = 10

    for key_block in obj.data.shape_keys.key_blocks:
        if key_block.name.startswith("Pose"):
            key_block.value = 0

    bpy.ops.object.mode_set(mode='OBJECT')

    for i in range(0, 9):
        obj.data.shape_keys.key_blocks['Shape%03d'%i].value = float(shape_latent[i])

    bpy.ops.object.smplx_update_joint_locations('EXEC_DEFAULT')
    bpy.ops.object.shape_key_remove(all=True, apply_mix=True)
#    bpy.ops.object.smplx_snap_ground_plane()
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
      
    target_armature = bpy.data.objects["SMPLX-neutral"]
    bpy.context.scene.target_rig = target_armature.name

    # import bvh
    bpy.ops.import_anim.bvh(filepath=bvh_file_path, filter_glob="*.bvh", target='ARMATURE', global_scale=0.01, frame_start=1, use_fps_scale=True, update_scene_fps=True, use_cyclic=False, rotate_mode='NATIVE', axis_forward='-Z', axis_up='Y')
    source_armature = bpy.context.object
    bpy.context.scene.source_rig = source_armature.name
    
    bpy.ops.arp.import_config_preset(preset_name='smpl2smpl')
    bpy.ops.arp.auto_scale()
    bpy.ops.arp.retarget(frame_start=0, frame_end=3600)
    
    # add light, floor, camera
    floor = add_floor(100)
    light = add_point_light((3, 0, 3))
    area_light = add_area_light((0, -4.68536, 5.65595), (0.7, 0, 0))

    camera = add_camera((-1, 0, 2), (4.45, 3.1415, 1.5707))
    bpy.ops.object.select_all(action='DESELECT')
    
    # Make the camera follow the character
    # Get the SMPLX character
    character = bpy.data.objects["SMPLX-neutral"]

    # Create an empty object to serve as a target for the camera
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0, 0, 0))
    target = bpy.context.object
    target.name = "CameraTarget"
    
    # Parent the empty to the character's pelvis bone
    target.parent = character
    target.parent_type = 'BONE'
    target.parent_bone = 'pelvis'

    # Position the camera at a fixed distance from the parent
    camera.location = (0, -3.0, 2)  # 3.0 units behind the parent
    
    # Make the camera look at the target
    camera.select_set(True)
    bpy.context.view_layer.objects.active = camera

    copy_loc_constraint = light.constraints.new(type='COPY_LOCATION')
    copy_loc_constraint.name = "FollowTargetXY_PointLight"
    copy_loc_constraint.target = target
    copy_loc_constraint.use_x = True
    copy_loc_constraint.use_y = True
    copy_loc_constraint.use_z = False # Light maintains its own Z
    copy_loc_constraint.use_offset = True

    copy_loc_constraint = area_light.constraints.new(type='COPY_LOCATION')
    copy_loc_constraint.name = "FollowTargetXY_AreaLight"
    copy_loc_constraint.target = target
    copy_loc_constraint.use_x = True
    copy_loc_constraint.use_y = True
    copy_loc_constraint.use_z = False # Light maintains its own Z
    copy_loc_constraint.use_offset = True

    copy_loc_constraint = camera.constraints.new(type='COPY_LOCATION')
    copy_loc_constraint.name = "FollowTargetXY"
    copy_loc_constraint.target = target
    copy_loc_constraint.use_x = True  # Follow target's X
    copy_loc_constraint.use_y = True  # Follow target's Y
    copy_loc_constraint.use_z = False # Do NOT follow target's Z. Camera maintains its own Z.
    copy_loc_constraint.use_offset = True
    
    # Add a Track To constraint to the camera
    track_constraint = camera.constraints.new('TRACK_TO')
    track_constraint.target = target
    track_constraint.track_axis = 'TRACK_NEGATIVE_Z'
    track_constraint.up_axis = 'UP_Y'
    
    # Deselect all objects
    bpy.ops.object.select_all(action='DESELECT')
    
    scene = bpy.context.scene
    bpy.context.scene.render.ffmpeg.format = 'MPEG4'
    bpy.context.scene.render.ffmpeg.constant_rate_factor = 'HIGH'
    bpy.context.scene.eevee.taa_render_samples = 16
    bpy.context.scene.render.ffmpeg.audio_codec = 'AAC'
    scene.render.resolution_x = 512
    scene.render.resolution_y = 512
    scene.camera = camera
    scene.frame_start = 600
    scene.frame_end = 1200
    scene.render.fps = 30
    scene.render.image_settings.file_format = 'FFMPEG'
    
    scene.render.filepath = output_path
    
    bpy.ops.render.render(animation=True, use_viewport=True)
