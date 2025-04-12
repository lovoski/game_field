#include "toolkit/loaders/mesh.hpp"
#include "toolkit/loaders/imp.hpp"
#include <fstream>
#include <iostream>
#include <stack>

#include <ufbx.h>

namespace toolkit::assets {

using toolkit::math::matrix4;
using toolkit::math::quat;
using toolkit::math::vector3;
using toolkit::math::vector4;

// ----------------------------- FBX file loading -----------------------------

namespace FBX {

using MathScalarType = float;
vector3 Convert(ufbx_vec3 v) {
  return {(MathScalarType)v.x, (MathScalarType)v.y, (MathScalarType)v.z};
}
vector4 Convert(ufbx_vec4 v) {
  return {(MathScalarType)v.x, (MathScalarType)v.y, (MathScalarType)v.z,
          (MathScalarType)v.w};
}
quat Convert(ufbx_quat q) {
  return {(MathScalarType)q.w, (MathScalarType)q.x, (MathScalarType)q.y,
          (MathScalarType)q.z};
}
struct TmpVertexFBXLoading {
  vector4 Position;
  vector4 normal;
  vector4 tex_coords;
  vector4 color;
  int BoneId[MAX_BONES_PER_MESH];
  float BoneWeight[MAX_BONES_PER_MESH];
  vector3 BlendShapeOffset[MAX_BLEND_SHAPES_PER_MESH];
  vector3 BlendShapeNormal[MAX_BLEND_SHAPES_PER_MESH];
};
mesh processMesh(ufbx_mesh *mesh_instance, ufbx_mesh_part &part,
                 std::map<std::string, std::size_t> &boneMapping,
                 std::string model_path) {
  std::vector<TmpVertexFBXLoading> vertices;
  std::vector<unsigned int> indices(mesh_instance->max_face_triangles * 3);
  std::map<std::string, std::size_t> blendShapeMapping;
  std::vector<blend_shape> blend_shapes;
  for (auto faceInd : part.face_indices) {
    ufbx_face face = mesh_instance->faces[faceInd];
    auto numTriangles =
        ufbx_triangulate_face(indices.data(), indices.size(), mesh_instance, face);
    for (auto i = 0; i < numTriangles * 3; ++i) {
      auto index = indices[i];
      TmpVertexFBXLoading v;
      v.Position.x() = mesh_instance->vertex_position[index].x;
      v.Position.y() = mesh_instance->vertex_position[index].y;
      v.Position.z() = mesh_instance->vertex_position[index].z;
      v.Position.w() = 1.0f;

      v.normal.x() = mesh_instance->vertex_normal[index].x;
      v.normal.y() = mesh_instance->vertex_normal[index].y;
      v.normal.z() = mesh_instance->vertex_normal[index].z;
      v.normal.w() = 0.0f;

      // for multiple sets of uv, refer to mesh->uv_sets
      // this is by default the first uv set
      if (mesh_instance->uv_sets.count > 0) {
        v.tex_coords.x() = mesh_instance->vertex_uv[index].x;
        v.tex_coords.y() = mesh_instance->vertex_uv[index].y;
        if (mesh_instance->uv_sets.count > 1) {
          v.tex_coords.z() = mesh_instance->uv_sets[1].vertex_uv[index].x;
          v.tex_coords.w() = mesh_instance->uv_sets[1].vertex_uv[index].y;
        } else {
          v.tex_coords.z() = 0.0f;
          v.tex_coords.w() = 0.0f;
        }
      } else
        v.tex_coords = vector4::Zero();

      if (mesh_instance->vertex_color.exists) {
        v.color.x() = mesh_instance->vertex_color[index].x;
        v.color.y() = mesh_instance->vertex_color[index].y;
        v.color.z() = mesh_instance->vertex_color[index].z;
        v.color.w() = mesh_instance->vertex_color[index].w;
      } else
        v.color = vector4::Ones();

      for (int boneCounter = 0; boneCounter < MAX_BONES_PER_MESH;
           ++boneCounter) {
        v.BoneId[boneCounter] = 0;
        v.BoneWeight[boneCounter] = 0.0f;
      }
      // setup skin deformers
      // if there's skin_deformers, the boneMapping won't be empty
      for (auto skin : mesh_instance->skin_deformers) {
        auto vertex = mesh_instance->vertex_indices[index];
        auto skinVertex = skin->vertices[vertex];
        auto numWeights = skinVertex.num_weights;
        if (numWeights > MAX_BONES_PER_MESH)
          numWeights = MAX_BONES_PER_MESH;
        float totalWeight = 0.0f;
        for (auto k = 0; k < numWeights; ++k) {
          auto skinWeight = skin->weights[skinVertex.weight_begin + k];
          std::string clusterName =
              skin->clusters[skinWeight.cluster_index]->bone_node->name.data;
          auto boneMapIt = boneMapping.find(clusterName);
          int mappedBoneInd = 0;
          if (boneMapIt == boneMapping.end()) {
            printf("cluster named %s not in boneMapping", clusterName.c_str());
          } else
            mappedBoneInd = boneMapIt->second;
          v.BoneId[k] = mappedBoneInd;
          totalWeight += (float)skinWeight.weight;
          v.BoneWeight[k] = (float)skinWeight.weight;
        }
        // normalize the skin weights
        if (totalWeight != 0.0f) {
          for (auto k = 0; k < numWeights; ++k)
            v.BoneWeight[k] /= totalWeight;
        } else {
          // parent the vertex to root if its not bind
          v.BoneId[0] = 0;
          v.BoneWeight[0] = 1.0f;
        }
      }
      // blend shapes
      for (auto deformer : mesh_instance->blend_deformers) {
        uint32_t vertex = mesh_instance->vertex_indices[index];

        size_t num_blends = deformer->channels.count;
        if (num_blends > MAX_BLEND_SHAPES_PER_MESH) {
          num_blends = MAX_BLEND_SHAPES_PER_MESH;
        }

        for (size_t i = 0; i < num_blends; i++) {
          ufbx_blend_channel *channel = deformer->channels[i];
          ufbx_blend_shape *shape = channel->target_shape;
          auto it = blendShapeMapping.find(shape->name.data);
          int blendShapeIndex = blendShapeMapping.size();
          if (it == blendShapeMapping.end()) {
            blendShapeMapping[shape->name.data] = blendShapeIndex;
            blend_shapes.push_back(blend_shape());
          } else {
            blendShapeIndex = it->second;
          }
          assert(shape); // In theory this could be missing in broken files
          auto &bs = blend_shapes[blendShapeIndex];
          bs.name = shape->name.data;
          v.BlendShapeOffset[blendShapeIndex] =
              Convert(ufbx_get_blend_shape_vertex_offset(shape, vertex));
        }
      }
      vertices.push_back(v);
    }
  }

  if (vertices.size() != part.num_triangles * 3)
    printf("vertices number inconsistent with part's triangle number");

  ufbx_vertex_stream stream[] = {
      {vertices.data(), vertices.size(), sizeof(TmpVertexFBXLoading)}};
  indices.resize(part.num_triangles * 3);
  auto numVertices = ufbx_generate_indices(stream, 1, indices.data(),
                                           indices.size(), nullptr, nullptr);
  vertices.resize(numVertices);

  // create the final mesh
  std::vector<mesh_vertex> actualVertices(vertices.size());
  for (int i = 0; i < numVertices; i++) {
    actualVertices[i].position = vertices[i].Position;
    actualVertices[i].normal = vertices[i].normal;
    actualVertices[i].tex_coords = vertices[i].tex_coords;
    actualVertices[i].color = vertices[i].color;
    for (int j = 0; j < MAX_BONES_PER_MESH; j++) {
      actualVertices[i].bond_indices[j] = vertices[i].BoneId[j];
      actualVertices[i].bone_weights[j] = vertices[i].BoneWeight[j];
    }
  }
  mesh result;
  result.vertices = actualVertices;
  result.indices = indices;
  result.mesh_name = mesh_instance->name.data;
  result.model_path = model_path;

  if (blendShapeMapping.size() > 0) {
    // update blend shapes
    for (const auto &bsm : blendShapeMapping) {
      blend_shapes[bsm.second].data.resize(vertices.size());
      for (int vertId = 0; vertId < vertices.size(); ++vertId) {
        blend_shapes[bsm.second].data[vertId].offset_pos
            << vertices[vertId].BlendShapeOffset[bsm.second],
            0.0;
      }
    }
    result.blend_shapes = blend_shapes;
  }

  return result;
}

struct BoneInfo {
  std::string boneName;
  ufbx_node *node;
  vector3 localPosition = vector3::Zero();
  quat localRotation = quat::Identity();
  vector3 localScale = vector3::Ones();
  matrix4 offsetMatrix = matrix4::Identity();
  int parentIndex =
      -1; // Index of the parent bone in the hierarchy (-1 if root)
  std::vector<int> children = std::vector<int>(); // Indices of child bones
};

struct KeyFrame {
  vector3 localPosition;
  quat localRotation;
  vector3 localScale;
};

}; // namespace FBX

fbx_package load_fbx(std::string path) {
  fbx_package package;
  package.motion = nullptr;

  ufbx_load_opts opts = {};
  opts.target_axes = ufbx_axes_right_handed_y_up;
  opts.target_unit_meters = 1.0f;
  ufbx_error error;
  ufbx_scene *scene = ufbx_load_file(path.c_str(), &opts, &error);
  if (!scene) {
    printf("failed to load fbx scene from %s", error.description.data);
    return package;
  }

  package.model_path = path;
  std::vector<FBX::BoneInfo> globalBones;
  std::map<std::string, std::size_t> boneMapping;
  std::map<int, int> oldBoneInd2NewInd;
  std::stack<ufbx_node *> s;
  s.push(scene->root_node);
  while (!s.empty()) {
    auto cur = s.top();
    s.pop();
    if (cur->bone) {
      // if this is a bone node
      std::string boneName = cur->name.data;
      if (boneMapping.find(boneName) == boneMapping.end()) {
        int currentBoneInd = globalBones.size();
        boneMapping.insert(std::make_pair(boneName, currentBoneInd));
        FBX::BoneInfo bi;
        bi.boneName = boneName;
        bi.node = cur;
        auto localTransform = cur->local_transform;
        bi.localScale = FBX::Convert(localTransform.scale);
        bi.localPosition = FBX::Convert(localTransform.translation);
        bi.localRotation = FBX::Convert(localTransform.rotation);
        // find a parent bone
        auto curParent = cur->parent;
        while (curParent) {
          if (curParent->bone) {
            auto parentIt = boneMapping.find(curParent->name.data);
            if (parentIt == boneMapping.end()) {
              printf("parent bone don't exist in boneMapping");
            } else {
              bi.parentIndex = parentIt->second;
              globalBones[parentIt->second].children.push_back(currentBoneInd);
              break;
            }
          }
          curParent = curParent->parent;
        }
        globalBones.push_back(bi);
      }
    }
    for (auto child : cur->children) {
      s.push(child);
    }
  }

  if (globalBones.size() > 1 && boneMapping.size() > 1) {
    int animStackCount = scene->anim_stacks.count;
    // find the longest animation to import
    double longestDuration = -1.0;
    int longestAnimInd = -1;
    for (int animInd = 0; animInd < animStackCount; ++animInd) {
      auto tmpAnim = scene->anim_stacks[animInd]->anim;
      auto duration = tmpAnim->time_end - tmpAnim->time_begin;
      if (duration > longestDuration) {
        duration = longestDuration;
        longestAnimInd = animInd;
      }
    }
    std::vector<std::vector<FBX::KeyFrame>> animationPerJoint(
        globalBones.size(), std::vector<FBX::KeyFrame>());
    if (longestAnimInd != -1) {
      auto anim = scene->anim_stacks[longestAnimInd]
                      ->anim; // import the active animation only
      auto startTime = anim->time_begin, endTime = anim->time_end;
      // evaluate the motion to 60fps
      double sampleDelta = 1.0 / 60;
      for (auto jointInd = 0; jointInd < globalBones.size(); ++jointInd) {
        auto jointNode = globalBones[jointInd].node;
        for (double currentTime = startTime; currentTime < endTime;
             currentTime += sampleDelta) {
          auto localTransform =
              ufbx_evaluate_transform(anim, jointNode, currentTime);
          FBX::KeyFrame kf;
          kf.localPosition = FBX::Convert(localTransform.translation);
          kf.localRotation = FBX::Convert(localTransform.rotation);
          kf.localScale = FBX::Convert(localTransform.scale);
          animationPerJoint[jointInd].push_back(kf);
        }
      }
    }

    // create skeleton, register it in allSkeletons
    skeleton skel;
    // map indices from scene->skin_clusters to globalBones
    for (int clusterInd = 0; clusterInd < scene->skin_clusters.count;
         ++clusterInd) {
      auto cluster = scene->skin_clusters[clusterInd];
      std::string name = cluster->bone_node->name.data;
      auto it = boneMapping.find(name);
      if (it == boneMapping.end()) {
        printf("cluster %s doesn't appear in globalBones", name.c_str());
      } else {
        // update offset matrix of this bone
        auto offsetMatrix = cluster->geometry_to_bone;
        vector4 col0, col1, col2, col3;
        col0 << FBX::Convert(offsetMatrix.cols[0]), 0.0f;
        col1 << FBX::Convert(offsetMatrix.cols[1]), 0.0f;
        col2 << FBX::Convert(offsetMatrix.cols[2]), 0.0f;
        col3 << FBX::Convert(offsetMatrix.cols[3]), 1.0f;
        globalBones[it->second].offsetMatrix << col0, col1, col2, col3;
        oldBoneInd2NewInd.insert(std::make_pair(static_cast<int>(clusterInd),
                                                static_cast<int>(it->second)));
      }
    }

    // setup variables in the skeleton
    skel.name = "armature";
    skel.path = path;
    for (int jointInd = 0; jointInd < globalBones.size(); ++jointInd) {
      skel.joint_names.push_back(
          replace(globalBones[jointInd].boneName, " ", "_"));
      skel.joint_parent.push_back(globalBones[jointInd].parentIndex);
      skel.joint_children.push_back(globalBones[jointInd].children);
      skel.joint_offset.push_back(globalBones[jointInd].localPosition);
      skel.joint_rotation.push_back(globalBones[jointInd].localRotation);
      skel.joint_scale.push_back(globalBones[jointInd].localScale);
      skel.offset_matrices.push_back(globalBones[jointInd].offsetMatrix);
    }

    package.skeleton = std::make_shared<skeleton>(skel);

    // create motion for the skeleton
    int numFrames = animationPerJoint[0].size();
    int numJoints = animationPerJoint.size();
    if (numFrames > 0) {
      package.motion = std::make_shared<motion>();
      package.motion->fps = 60; // use 60fps by default
      package.motion->skeleton = skel;
      package.motion->path = path;
      for (int frameInd = 0; frameInd < numFrames; ++frameInd) {
        pose pose;
        pose.skeleton = package.skeleton.get();
        pose.joint_local_rot.resize(numJoints, quat::Identity());
        for (int jointInd = 0; jointInd < numJoints; ++jointInd) {
          if (jointInd == 0) {
            // process localPosition only for root joint
            pose.root_local_pos =
                animationPerJoint[jointInd][frameInd].localPosition;
          }
          pose.joint_local_rot[jointInd] =
              animationPerJoint[jointInd][frameInd].localRotation;
        }
        package.motion->poses.push_back(pose);
      }
    }
  }

  // process the meshes after the bone has setup
  for (auto fbxMesh : scene->meshes) {
    for (auto fbxMeshPart : fbxMesh->material_parts) {
      package.meshes.emplace_back(
          FBX::processMesh(fbxMesh, fbxMeshPart, boneMapping, path));
    }
  }
  if (package.skeleton) {
    for (auto &mesh : package.meshes)
      mesh.skeleton = package.skeleton;
  }

  // free the scene object
  ufbx_free_scene(scene);

  return package;
}

// ----------------------------- OBJ file loading -----------------------------

vector3 FaceNormal(vector3 v0, vector3 v1, vector3 v2) {
  auto e01 = v1 - v0;
  auto e02 = v2 - v0;
  return (e01.cross(e02)).normalized();
}

obj_package load_obj(std::string path) {
  obj_package package;
  auto &meshes = package.meshes;

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string err, warn;
  if (tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                       path.c_str())) {
    package.model_path = path;
    for (auto &shape : shapes) {
      std::size_t indexOffset = 0;

      std::vector<mesh_vertex> vertices;
      std::vector<uint32_t> indices;

      for (auto f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
        auto fv = shape.mesh.num_face_vertices[f];
        if (fv != 3) {
          printf("Only triangle faces are handled, face id=%d has %d vertices",
                 f, fv);
          indexOffset += fv;
          continue;
        }
        mesh_vertex vert[3];

        bool withoutNormal = false;
        for (auto v = 0; v < 3; ++v) {
          auto idx = shape.mesh.indices[indexOffset + v];
          vert[v].position << attrib.vertices[3 * idx.vertex_index + 0],
              attrib.vertices[3 * idx.vertex_index + 1],
              attrib.vertices[3 * idx.vertex_index + 2], 1.0f;
          if (idx.normal_index >= 0) {
            vert[v].normal << attrib.normals[3 * idx.normal_index + 0],
                attrib.normals[3 * idx.normal_index + 1],
                attrib.normals[3 * idx.normal_index + 2], 0.0f;
          } else {
            withoutNormal = true;
            vert[v].normal = vector4::Zero();
          }
          if (idx.texcoord_index >= 0) {
            vert[v].tex_coords << attrib.texcoords[2 * idx.texcoord_index + 0],
                attrib.texcoords[2 * idx.texcoord_index + 1], 0.0, 0.0;
          } else
            vert[v].tex_coords = vector4::Zero();
        }
        if (withoutNormal) {
          // manually compute the normal
          auto faceNormal =
              FaceNormal(vert[0].position.head<3>(), vert[1].position.head<3>(),
                         vert[2].position.head<3>());
          vert[0].normal << faceNormal, 0.0f;
          vert[1].normal << faceNormal, 0.0f;
          vert[2].normal << faceNormal, 0.0f;
        }

        vertices.push_back(vert[0]);
        vertices.push_back(vert[1]);
        vertices.push_back(vert[2]);
        indices.push_back(indexOffset + 0);
        indices.push_back(indexOffset + 1);
        indices.push_back(indexOffset + 2);

        indexOffset += fv;
      }
      mesh mesh;
      mesh.mesh_name = shape.name;
      mesh.model_path = path;
      mesh.vertices = vertices;
      mesh.indices = indices;
      meshes.emplace_back(mesh);
    }
  } else {
    printf("Failed to load .obj model from %s", path.c_str());
  }

  return package;
}

}; // namespace toolkit::Assets