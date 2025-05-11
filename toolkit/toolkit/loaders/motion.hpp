/**
 * Data structure related to motion data and joints hierarchy,
 * currently, only bvh and fbx motion files are tested.
 *
 * BVH file stores only the offsets of skeleton joints, the initial rotation
 * of all joints are identity rotations. The motion represents the
 * local rotations of joints and translation of root joint.
 *
 * FBX file stores the offsets and rotations of skeleton joints, so we need to
 * apply the initial rotations of a joint to export to bvh, but the motion
 * still represents the local rotation of joints. So we can use the same
 * scheme to construct the skeleton entities and get global positions.
 *
 * Keep in mind, its an easy practice to get local rotation after global
 * rotation.
 */

#pragma once

#include "toolkit/math.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/utils.hpp"

namespace toolkit::assets {

struct skeleton;
struct pose;
struct motion;

// The parent joint has a lower index than all its children
struct skeleton {
  std::string name;
  std::vector<std::string> joint_names;
  // local position to joints' parent
  std::vector<toolkit::math::vector3> joint_offset;
  // local rotation to joints' parent
  std::vector<toolkit::math::quat> joint_rotation;
  // local scale to joints' parent
  std::vector<toolkit::math::vector3> joint_scale;
  // offset matrics for skinning
  std::vector<toolkit::math::matrix4> offset_matrices;
  std::vector<int> joint_parent;
  std::vector<std::vector<int>> joint_children;

  // file containing the data
  std::string path;

  // Reset current skeleton as an empty skeleton with desired number of joints.
  void as_empty(int jointNum);

  // Get the default pose decribed with above information.
  pose get_rest_pose();

  // fbx skeleton need rotation to make up rest post, but bvh don't,
  // handle fbx-style skeleton and bvh-style skeleton export.
  // When a joint with no children is not named following the convention
  // f"{parentName}_End", if the parameter `keep_joint_names` is set to true,
  // an `End Site` with offset `0 0 0` will be automatically added.
  // Otherwise, this joint itself will be renamed to `End Site`.
  void export_as_bvh(std::string filepath, bool keep_joint_names = true);

  const int get_num_joints() { return joint_names.size(); }
};
REFLECT(skeleton, name, joint_names, joint_offset, joint_rotation, joint_scale,
        offset_matrices, joint_parent, joint_children, path)

// By default, the up direction is y-axis (0, 1, 0)
struct pose {
  pose() = default;
  ~pose() {
    skeleton = nullptr;
    joint_local_rot.clear();
    root_local_pos = toolkit::math::vector3::Zero();
  }

  skeleton *skeleton = nullptr;

  // // local positions of all joints
  // std::vector<toolkit::math::Vector3> jointPositions;

  // local position for root joint only
  toolkit::math::vector3 root_local_pos;
  // local rotations of all joints in quaternion
  std::vector<toolkit::math::quat> joint_local_rot;

  // Extract the facing direction projected to xz plane,
  toolkit::math::vector3 get_facing_dir(toolkit::math::vector3 rest_facing_dir =
                                            toolkit::math::vector3(0.0f, 0.0f,
                                                                   1.0f));

  // Perform fk to get global positions for all joints.
  // Mind that `self.ori = parent.ori * self.rot`
  // where `ori` means global rotation, `rot` means local rotation.
  // `self.pos = parent.pos + parent.ori * self.offset`
  // this function works for both fbx-style and bvh-style skeleton
  std::vector<toolkit::math::vector3> fk();

  // Same as `fk`, but the reference for orientations will be
  // automatically set.
  std::vector<toolkit::math::vector3>
  fk(std::vector<toolkit::math::quat> &orientations);

  // // Get the facing direction on XZ plane
  // glm::vec2 GetFacingDirection();
};

struct motion {
  motion() {}
  ~motion() {}
  int fps;
  skeleton skeleton;
  std::vector<pose> poses;
  // file containing the data
  std::string path;

  // We assume that the root's position channels always has the order XYZ,
  // while the rotation channels can have arbitrary orders,
  // the rotation channels can only follow behind the position channels.
  // Be aware that the euler angles in bvh file should
  // be parsed in reversed order, xyz rotation should be quaternion qx*qy*qz.
  bool load_from_bvh(std::string filename, float scale = 1.0f);
  // The saved bvh file's position channels will always be XYZ,
  // the rotation channels will be ZYX and
  // can only follow behind the position channels.
  // Only the root joint has 6 dofs, the rest joints only have 3 dofs.
  // The skeleton and motion will be flatten to offset-only manner
  // automatically.
  // When a joint with no children is not named following the convention
  // f"{parentName}_End", if the parameter `keep_joint_names` is set to true,
  // an `End Site` with offset `0 0 0` will be automatically added.
  // Otherwise, this joint itself will be renamed to `End Site`.
  bool save_to_bvh(std::string filename, bool keep_joint_names = true,
                   float scale = 1.0f);

  // Takes a float value as paramter, returns the slerp interpolated value.
  // If the frame is not valid (out of [0, nframe) range), returns the first
  // frame or last frame respectively.
  pose at(float frame);
};

}; // namespace toolkit::assets