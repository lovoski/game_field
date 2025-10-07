#include "toolkit/loaders/motion.hpp"

#include <filesystem>
#include <fstream>
#include <queue>
#include <set>
#include <stack>

namespace toolkit::assets {

using toolkit::math::quat;
using toolkit::math::vector3;
using std::queue;
using std::set;
using std::stack;
using std::string;
using std::vector;

inline bool IsWhiteSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

// split a string by white space
vector<string> SplitByWhiteSpace(string str) {
  int last = 0, cur = 0;
  vector<string> result;
  while (cur < str.size() && IsWhiteSpace(str[cur]))
    cur++;
  last = cur;
  while (cur <= str.size()) {
    if (cur == str.size() || IsWhiteSpace(str[cur])) {
      if (cur > last)
        result.push_back(str.substr(last, cur - last));
      while (cur < str.size() && IsWhiteSpace(str[cur]))
        cur++;
      last = cur;
    }
    cur++;
  }
  return result;
}

std::string ConcatStr(std::vector<std::string> str, int start) {
  std::string result = "";
  for (int i = start; i < str.size(); i++) {
    result = result + str[i];
  }
  return result;
}

#define EULER_XYZ 12
#define EULER_XZY 21
#define EULER_YXZ 102
#define EULER_YZX 120
#define EULER_ZXY 201
#define EULER_ZYX 210

quat QuatFromEulers(vector3 angles, int order) {
  auto qx = toolkit::math::angle_axis(angles.x(), vector3(1.0f, 0.0f, 0.0f));
  auto qy = toolkit::math::angle_axis(angles.y(), vector3(0.0f, 1.0f, 0.0f));
  auto qz = toolkit::math::angle_axis(angles.z(), vector3(0.0f, 0.0f, 1.0f));
  if (order == EULER_XYZ) {
    return qx * qy * qz;
  } else if (order == EULER_XZY) {
    return qx * qz * qy;
  } else if (order == EULER_YXZ) {
    return qy * qx * qz;
  } else if (order == EULER_YZX) {
    return qy * qz * qx;
  } else if (order == EULER_ZXY) {
    return qz * qx * qy;
  } else if (order == EULER_ZYX) {
    return qz * qy * qx;
  } else
    return quat::Identity();
}

bool bvh_motion::load(string filename, float scale) {
  std::ifstream fileInput(filename);
  if (!fileInput.is_open()) {
    printf("failed to open file %s\n", filename.c_str());
    fileInput.close();
    return false;
  } else {
    vector<int> jointChannels;
    vector<int> jointChannelsOrder;
    string line;
    getline(fileInput, line);
    if (line == "HIERARCHY") {
      getline(fileInput, line);
      auto lineSeg = SplitByWhiteSpace(line);
      if (lineSeg[0] == "ROOT") {
        int currentJoint = 0, parentJoint = -1;
        stack<int> s;
        s.push(currentJoint);
        skel.joint_names.push_back(ConcatStr(lineSeg, 1)); // the name
        skel.joint_parent.push_back(parentJoint);          // the parent
        while (!s.empty()) {
          getline(fileInput, line);
          lineSeg = SplitByWhiteSpace(line);
          if (lineSeg.size() == 0)
            continue; // skip blank lines
          if (lineSeg[0] == "{") {
            currentJoint++;
          } else if (lineSeg[0] == "}") {
            s.pop();
          } else if (lineSeg[0] == "OFFSET") {
            float xOffset = std::stof(lineSeg[1]) * scale;
            float yOffset = std::stof(lineSeg[2]) * scale;
            float zOffset = std::stof(lineSeg[3]) * scale;
            skel.joint_offset.push_back(vector3(xOffset, yOffset, zOffset));
            // ready to recieve children
            skel.joint_children.push_back(vector<int>());
            getline(fileInput, line);
            lineSeg = SplitByWhiteSpace(line);
            if (lineSeg[0] == "CHANNELS") {
              int numChannels = std::stoi(lineSeg[1]);
              jointChannels.push_back(numChannels);
              // process the channels, assume that the rotation channels
              // are always behind the position channels
              if (numChannels == 3 || numChannels == 6) {
                // only rotation channels
                int a = lineSeg[numChannels - 1][0] - 'X';
                int b = lineSeg[numChannels][0] - 'X';
                int c = lineSeg[numChannels + 1][0] - 'X';
                jointChannelsOrder.push_back(a * 100 + b * 10 + c);
              } else
                throw std::runtime_error(
                    "number of channels in bvh must be either 3 or 6");
            } else if (lineSeg[0] == "}") {
              jointChannels.push_back(0);
              jointChannelsOrder.push_back(0);
              s.pop(); // this is a end effector
            } else
              throw std::runtime_error(
                  ("this label should be CHANNELS or `}` instead of " +
                   lineSeg[0])
                      .c_str());
          } else if (lineSeg[0] == "JOINT") {
            parentJoint = s.top();
            skel.joint_parent.push_back(parentJoint);
            skel.joint_children[parentJoint].push_back(currentJoint);
            skel.joint_names.push_back(
                ConcatStr(lineSeg, 1)); // name of this joint
            s.push(currentJoint);       // keep record of this child joint
          } else if (lineSeg[0] == "End") {
            parentJoint = s.top();
            skel.joint_parent.push_back(parentJoint);
            skel.joint_children[parentJoint].push_back(currentJoint);
            skel.joint_names.push_back(skel.joint_names[parentJoint] +
                                          "_End"); // the end effector's name
            getline(fileInput, line);              // {
            getline(fileInput, line);              // OFFSET
            lineSeg = SplitByWhiteSpace(line);
            if (lineSeg[0] == "OFFSET") {
              float xOffset = std::stof(lineSeg[1]) * scale;
              float yOffset = std::stof(lineSeg[2]) * scale;
              float zOffset = std::stof(lineSeg[3]) * scale;
              skel.joint_offset.push_back(
                  vector3(xOffset, yOffset, zOffset));
              skel.joint_children.push_back(vector<int>());
              jointChannels.push_back(0); // 0 for end effector
              jointChannelsOrder.push_back(0);
            } else
              throw std::runtime_error(
                  "the label should be OFFSET for end effector");
            getline(fileInput, line); // }
            currentJoint++;           // move to the next joint index
          }
        }

        // initialize joint_rotation and joint_scale
        // bvh format don't store localRotation and localScale
        // of a skeleton hierarchy, initialize to identity transform
        skel.joint_rotation =
            std::vector<quat>(skel.get_num_joints(), quat::Identity());
        skel.joint_scale =
            std::vector<vector3>(skel.get_num_joints(), vector3::Ones());

        // parse pose data
        while (getline(fileInput, line)) {
          lineSeg = SplitByWhiteSpace(line);
          if (lineSeg[0] == "MOTION")
            break;
        }
        if (lineSeg[0] == "MOTION") {
          getline(fileInput, line);
          lineSeg = SplitByWhiteSpace(line);
          if (lineSeg[0] == "Frames:") {
            poses.resize(std::stoi(lineSeg[1]));
            getline(fileInput, line);
            lineSeg = SplitByWhiteSpace(line);
            if (lineSeg[0] == "Frame" && lineSeg[1] == "Time:") {
              float timePerFrame = std::stof(lineSeg[2]);
              fps = std::round(1.0f / timePerFrame);
              const int jointNumber = skel.get_num_joints();
              for (int frameInd = 0; frameInd < poses.size(); ++frameInd) {
                getline(fileInput, line);
                lineSeg = SplitByWhiteSpace(line);
                vector<vector3> jointPositions = skel.joint_offset;
                poses[frameInd].skel = &this->skel;
                poses[frameInd].joint_local_rot.resize(jointNumber,
                                                      quat::Identity());
                int segInd = 0;
                for (int jointInd = 0; jointInd < jointNumber; ++jointInd) {
                  if (jointChannels[jointInd] == 6) {
                    // set up the positions if exists
                    float x = std::stof(lineSeg[segInd++]) * scale,
                          y = std::stof(lineSeg[segInd++]) * scale,
                          z = std::stof(lineSeg[segInd++]) * scale;
                    jointPositions[jointInd] = vector3(x, y, z);
                  }
                  if (jointChannels[jointInd] != 0) {
                    // set up rotations
                    vector3 v = vector3::Zero();
                    int rotationOrder = jointChannelsOrder[jointInd];
                    int o1 = rotationOrder / 100;
                    int o2 = (rotationOrder - o1 * 100) / 10;
                    int o3 = rotationOrder - o1 * 100 - o2 * 10;
                    v[o1] = std::stof(lineSeg[segInd++]);
                    v[o2] = std::stof(lineSeg[segInd++]);
                    v[o3] = std::stof(lineSeg[segInd++]);
                    poses[frameInd].joint_local_rot[jointInd] = QuatFromEulers(
                        toolkit::math::deg_to_rad(v), rotationOrder);
                  } else {
                    // set the rotation of end effectors to normal quaternion
                    poses[frameInd].joint_local_rot[jointInd] = quat::Identity();
                  }
                }
                // setup the root translation only
                poses[frameInd].root_local_pos = jointPositions[0];
              }
            }
          } else
            throw std::runtime_error("number of frames must be specified");
        } else
          throw std::runtime_error(
              "pose data should start with a MOTION label");
        fileInput.close();
        return true;
      } else
        throw std::runtime_error("the label should be ROOT instead of " +
                                 lineSeg[0]);
    } else
      throw std::runtime_error("bvh file should start with HIERARCHY");
    fileInput.close();
    return false;
  }
}

inline void BVHPadding(std::ostream &out, int depth) {
  for (int i = 0; i < depth; ++i)
    out << "\t";
}

bool bvh_motion::save(string filename, bool keepJointNames, float scale) {
  // apply the initial rotations of skeleton joints
  // the motion data remains unchanged
  auto restPose = skel.get_rest_pose();
  int jointNumber = skel.get_num_joints();
  vector<quat> globalJointOrien;
  auto globalJointPositions = restPose.fk(globalJointOrien);
  auto flattenJointOffset = globalJointPositions;
  for (int i = 0; i < skel.get_num_joints(); ++i) {
    int parentInd = skel.joint_parent[i];
    if (parentInd != -1) {
      flattenJointOffset[i] -= globalJointPositions[parentInd];
    }
  }
  // output the skeleton and motions
  std::ofstream fileOutput(filename);
  if (!fileOutput.is_open()) {
    printf("failed to open file %s\n", filename.c_str());
    fileOutput.close();
    return false;
  } else {
    fileOutput << "HIERARCHY" << std::endl;
    // write the skeleton hierarchy
    std::set<int> incorrectNamedEE;
    int depth = 0;
    for (int jointInd = 0; jointInd < skel.get_num_joints(); ++jointInd) {
      BVHPadding(fileOutput, depth);
      if (skel.joint_children[jointInd].size() == 0) {
        if (endswith(skel.joint_names[jointInd], "_End") ||
            !keepJointNames) {
          // force rename end effector
          fileOutput << "End Site\n";
        } else {
          incorrectNamedEE.insert(jointInd);
          fileOutput << "JOINT "
                     << replace(skel.joint_names[jointInd], " ", "_")
                     << "\n";
        }
        BVHPadding(fileOutput, depth++);
        fileOutput << "{\n";
        BVHPadding(fileOutput, depth);
        fileOutput << "OFFSET " << flattenJointOffset[jointInd].x() * scale
                   << " " << flattenJointOffset[jointInd].y() * scale << " "
                   << flattenJointOffset[jointInd].z() * scale << "\n";
        if (incorrectNamedEE.count(jointInd) != 0) {
          // add an end effector with no offset to the end
          BVHPadding(fileOutput, depth);
          fileOutput << "CHANNELS 3 Zrotation Yrotation Xrotation\n";
          BVHPadding(fileOutput, depth);
          fileOutput << "End Site\n";
          BVHPadding(fileOutput, depth);
          fileOutput << "{\n";
          BVHPadding(fileOutput, depth + 1);
          fileOutput << "OFFSET 0 0 0\n";
          BVHPadding(fileOutput, depth);
          fileOutput << "}\n";
        }
        BVHPadding(fileOutput, --depth);
        fileOutput << "}\n";
        // find the direct parent of the next joint (if exists)
        if (jointInd == skel.get_num_joints() - 1) {
          // flush until depth is 0
          while (depth > 0) {
            BVHPadding(fileOutput, --depth);
            fileOutput << "}\n";
          }
        } else {
          int parentOfNext = skel.joint_parent[jointInd + 1];
          int cur = skel.joint_parent[jointInd];
          while (cur != parentOfNext) {
            BVHPadding(fileOutput, --depth);
            fileOutput << "}\n";
            cur = skel.joint_parent[cur];
          }
        }
      } else {
        if (skel.joint_parent[jointInd] == -1)
          fileOutput << "ROOT "
                     << replace(skel.joint_names[jointInd], " ", "_")
                     << "\n";
        else
          fileOutput << "JOINT "
                     << replace(skel.joint_names[jointInd], " ", "_")
                     << "\n";
        BVHPadding(fileOutput, depth++);
        fileOutput << "{\n";
        BVHPadding(fileOutput, depth);
        fileOutput << "OFFSET " << flattenJointOffset[jointInd].x() * scale
                   << " " << flattenJointOffset[jointInd].y() * scale << " "
                   << flattenJointOffset[jointInd].z() * scale << "\n";
        BVHPadding(fileOutput, depth);
        if (skel.joint_parent[jointInd] != -1) {
          fileOutput << "CHANNELS 3 Zrotation Yrotation Xrotation\n";
        } else {
          fileOutput << "CHANNELS 6 Xposition Yposition Zposition Zrotation "
                        "Yrotation Xrotation\n";
        }
      }
    }
    fileOutput.flush();

    // write the pose data
    fileOutput << "MOTION\nFrames: " << poses.size() << "\n"
               << "Frame Time: " << 1.0f / fps << "\n";
    for (int frameInd = 0; frameInd < poses.size(); ++frameInd) {
      fileOutput << poses[frameInd].root_local_pos.x() * scale << " "
                 << poses[frameInd].root_local_pos.y() * scale << " "
                 << poses[frameInd].root_local_pos.z() * scale << " ";
      vector<quat> frameJointRot(jointNumber, quat::Identity());
      vector<quat> newOrien(jointNumber, quat::Identity());
      vector<quat> oldOrien;
      // oldOrien is the ground truth rotation we need
      poses[frameInd].fk(oldOrien);
      for (int jointInd = 0; jointInd < jointNumber; ++jointInd) {
        int parentInd = skel.joint_parent[jointInd];
        // `newOrien` is the delta rotation in each frame
        // oldOrien = newOrien * globalJointOrien
        // meaning that we can get the ground truth rotation with the skeleton
        // initial rotation and a delta rotation stored in each frame,
        // we will get local rotation of each joint from this delta global
        // rotation
        newOrien[jointInd] =
            oldOrien[jointInd] * globalJointOrien[jointInd].inverse();
        if (parentInd == -1) {
          frameJointRot[jointInd] = newOrien[jointInd];
        } else {
          frameJointRot[jointInd] =
              newOrien[parentInd].inverse() * newOrien[jointInd];
        }
      }
      for (int jointInd = 0; jointInd < jointNumber; ++jointInd) {
        if (skel.joint_children[jointInd].size() != 0) {
          vector3 euler =
              toolkit::math::quat_to_euler(frameJointRot[jointInd]);
          vector3 eulerDegree = toolkit::math::rad_to_deg(euler);
          fileOutput << eulerDegree.z() << " " << eulerDegree.y() << " "
                     << eulerDegree.x() << " ";
        } else {
          if (incorrectNamedEE.count(jointInd) != 0) {
            // if this joint is a incorrectly named ee
            fileOutput << "0 0 0 ";
          }
        }
      }
      fileOutput << "\n";
    }
    fileOutput.close();
    return true;
  }
}

vector<vector3> pose::fk(vector<quat> &orientations) {
  orientations.clear();
  int jointNum = skel->get_num_joints();
  orientations = vector<quat>(jointNum, quat::Identity());
  vector<vector3> positions(jointNum, vector3::Zero());
  if (jointNum != joint_local_rot.size()) {
    throw std::runtime_error(
        "inconsistent joint number between skeleton and pose data");
    return positions;
  }
  quat parentOrientation;
  vector3 parentPosition;
  // start traversaling the joints
  for (int curJoint = 0; curJoint < jointNum; ++curJoint) {
    // compute the orientation
    int parentInd = skel->joint_parent[curJoint];
    if (parentInd == -1) {
      parentOrientation = orientations[0];
      parentPosition = vector3::Zero();
    } else {
      parentOrientation = orientations[parentInd];
      parentPosition = positions[parentInd];
    }
    orientations[curJoint] = parentOrientation * joint_local_rot[curJoint];
    positions[curJoint] =
        parentPosition + parentOrientation * skel->joint_offset[curJoint];
    if (parentInd == -1)
      positions[curJoint] = root_local_pos;
  }
  return positions;
}

vector<vector3> pose::fk() {
  vector<quat> orientations;
  return fk(orientations);
}

pose bvh_motion::at(float frame) {
  if (frame <= 0.0f)
    return poses[0];
  if (frame >= poses.size() - 1)
    return poses[poses.size() - 1];
  unsigned int start = (unsigned int)frame;
  unsigned int end = start + 1;
  float alpha = frame - start;
  pose result;
  result.skel = &skel;
  result.joint_local_rot =
      vector<quat>(skel.get_num_joints(), quat::Identity());
  result.root_local_pos = poses[start].root_local_pos * (1.0f - alpha) +
                             poses[end].root_local_pos * alpha;
  for (auto jointInd = 0; jointInd < skel.get_num_joints(); ++jointInd) {
    result.joint_local_rot[jointInd] =
        poses[start].joint_local_rot[jointInd].slerp(
            alpha, poses[end].joint_local_rot[jointInd]);
  }
  return result;
}

pose skeleton::get_rest_pose() {
  pose p;
  p.skel = this;
  p.root_local_pos = joint_offset[0];
  int jointNum = get_num_joints();
  p.joint_local_rot.resize(jointNum, quat::Identity());
  for (int jointInd = 0; jointInd < jointNum; ++jointInd) {
    p.joint_local_rot[jointInd] = joint_rotation[jointInd];
  }
  return p;
}

void skeleton::export_as_bvh(std::string filepath, bool keep_joint_names) {
  int jointNum = get_num_joints();
  pose emptyPose = get_rest_pose();
  emptyPose.skel = this;
  bvh_motion tmpMotion;
  tmpMotion.skel = *this;
  tmpMotion.fps = 30;
  tmpMotion.poses.push_back(emptyPose);
  tmpMotion.save(filepath, keep_joint_names);
}

vector3 pose::get_facing_dir(vector3 restFacing) {
  auto q = joint_local_rot[0];
  auto xzRot = q * vector3(0.0f, 1.0f, 0.0f);
  auto qxz = quat::FromTwoVectors(vector3(0.0f, 1.0f, 0.0f), xzRot);
  auto qy = qxz.inverse() * q;
  return qy * restFacing;
}

void skeleton::as_empty(int jointNum) {
  joint_names.resize(jointNum, "");
  joint_offset.resize(jointNum, math::vector3::Zero());
  joint_rotation.resize(jointNum, math::quat::Identity());
  joint_scale.resize(jointNum, math::vector3::Ones());
  joint_parent.resize(jointNum, -1);
  joint_children.resize(jointNum, std::vector<int>());
}

}; // namespace toolkit::Assets