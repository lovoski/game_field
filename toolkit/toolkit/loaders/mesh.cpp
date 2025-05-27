#include "toolkit/loaders/mesh.hpp"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <iostream>
#include <spdlog/spdlog.h>

namespace toolkit::assets {

math::matrix4 to_matrix4(const aiMatrix4x4 &mat) {
  math::matrix4 result;
  result(0, 0) = mat.a1;
  result(0, 1) = mat.a2;
  result(0, 2) = mat.a3;
  result(0, 3) = mat.a4;
  result(1, 0) = mat.b1;
  result(1, 1) = mat.b2;
  result(1, 2) = mat.b3;
  result(1, 3) = mat.b4;
  result(2, 0) = mat.c1;
  result(2, 1) = mat.c2;
  result(2, 2) = mat.c3;
  result(2, 3) = mat.c4;
  result(3, 0) = mat.d1;
  result(3, 1) = mat.d2;
  result(3, 2) = mat.d3;
  result(3, 3) = mat.d4;
  return result;
}

std::vector<model> open_model(std::string filepath) {
  std::vector<model> loaded_data;
  Assimp::Importer importer;
  importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
  const aiScene *scene = importer.ReadFile(
      filepath, aiProcess_Triangulate | aiProcess_CalcTangentSpace |
                    aiProcess_OptimizeMeshes | aiProcess_LimitBoneWeights |
                    aiProcess_JoinIdenticalVertices |
                    aiProcess_PopulateArmatureData);
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
    spdlog::error("failed to open fbx model from {0}", filepath);
    return loaded_data;
  }

  // --- Step 1: Identify all bone nodes in the scene ---
  std::vector<const aiNode *> all_bone_nodes;
  std::map<const aiNode *, bool>
      is_bone_node_map; // Helper map for quick lookup

  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh *assimp_mesh = scene->mMeshes[i];
    for (unsigned int j = 0; j < assimp_mesh->mNumBones; ++j) {
      const aiBone *assimp_bone = assimp_mesh->mBones[j];
      aiNode *bone_node = scene->mRootNode->FindNode(assimp_bone->mName);
      if (bone_node) {
        if (is_bone_node_map.find(bone_node) == is_bone_node_map.end()) {
          all_bone_nodes.push_back(bone_node);
          is_bone_node_map[bone_node] = true;
        }
      }
    }
  }

  // --- Step 2: Identify skeleton root nodes ---
  // A skeleton root is a bone node whose parent is NOT a bone node.
  std::vector<const aiNode *> skeleton_root_nodes;
  for (const aiNode *bone_node : all_bone_nodes) {
    const aiNode *parent = bone_node->mParent;
    bool parent_is_bone = false;
    if (parent) {
      if (is_bone_node_map.count(parent)) {
        parent_is_bone = true;
      }
    }
    if (!parent_is_bone) {
      skeleton_root_nodes.push_back(bone_node);
    }
  }

  // --- Step 3: Build skeleton data for each identified skeleton ---
  std::map<const aiNode *, skeleton> skeleton_root_to_skeleton_map;

  for (const aiNode *skeleton_root_node : skeleton_root_nodes) {
    skeleton current_skeleton;
    current_skeleton.name = skeleton_root_node->mName.C_Str();

    std::vector<const aiNode *> skeleton_joint_nodes_temp;
    std::vector<int> joint_parent_indices;
    std::map<const aiNode *, int>
        node_to_joint_index_temp; // Map for current skeleton

    // Recursive lambda to collect joint data for this skeleton
    std::function<void(const aiNode *, int)> collect_joint_data =
        [&](const aiNode *node, int parent_index) {
          if (is_bone_node_map.count(
                  node)) { // Check if this node is a bone node
            int current_joint_index = skeleton_joint_nodes_temp.size();
            skeleton_joint_nodes_temp.push_back(node);
            current_skeleton.joint_names.push_back(node->mName.C_Str());
            joint_parent_indices.push_back(parent_index);
            node_to_joint_index_temp[node] = current_joint_index;

            // Extract local transformation
            aiVector3D scale, position;
            aiQuaternion rotation;
            node->mTransformation.Decompose(scale, rotation, position);
            current_skeleton.joint_offset.push_back(
                math::vector3(position.x, position.y, position.z));
            current_skeleton.joint_rotation.push_back(
                math::quat(rotation.w, rotation.x, rotation.y, rotation.z));
            current_skeleton.joint_scale.push_back(
                math::vector3(scale.x, scale.y, scale.z));

            // Find the corresponding bone in *any* mesh to get the offset
            // matrix
            toolkit::math::matrix4 offset_matrix;
            bool found_offset_matrix = false;
            for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
              const aiMesh *assimp_mesh = scene->mMeshes[i];
              for (unsigned int j = 0; j < assimp_mesh->mNumBones; ++j) {
                const aiBone *assimp_bone = assimp_mesh->mBones[j];
                if (std::string(assimp_bone->mName.C_Str()) ==
                    node->mName.C_Str()) {
                  offset_matrix = to_matrix4(assimp_bone->mOffsetMatrix);
                  found_offset_matrix = true;
                  break;
                }
              }
              if (found_offset_matrix)
                break;
            }
            if (!found_offset_matrix) {
              std::cerr
                  << "Warning: Could not find offset matrix for bone node: "
                  << node->mName.C_Str() << std::endl;
              offset_matrix =
                  toolkit::math::matrix4::Identity(); // Use identity as
                                                      // fallback
            }
            current_skeleton.offset_matrices.push_back(offset_matrix);

            // Recursively process bone children
            for (unsigned int i = 0; i < node->mNumChildren; ++i) {
              collect_joint_data(node->mChildren[i], current_joint_index);
            }
          } else {
            // If the node is not a bone node, stop traversing down this branch
            // for skeleton data We only want nodes that are part of the bone
            // hierarchy defined by bones.
          }
        };

    collect_joint_data(skeleton_root_node, -1);

    current_skeleton.joint_parent = joint_parent_indices;

    // Build joint_children structure
    current_skeleton.joint_children.resize(current_skeleton.get_num_joints());
    for (int i = 0; i < current_skeleton.get_num_joints(); ++i) {
      int parent_index = current_skeleton.joint_parent[i];
      if (parent_index != -1) {
        current_skeleton.joint_children[parent_index].push_back(i);
      }
    }

    if (current_skeleton.get_num_joints() > 0) {
      skeleton_root_to_skeleton_map[skeleton_root_node] = current_skeleton;
    }
  }

  // --- Step 4: Assign nodes to models and create initial models ---
  std::map<const aiNode *, int> node_to_model_index;
  int current_model_idx = 0;
  std::map<int, const aiNode *>
      model_index_to_skeleton_root; // Map model index to its skeleton root node

  // Assign model indices to skeleton hierarchies
  for (const auto &pair : skeleton_root_to_skeleton_map) {
    const aiNode *skeleton_root_node = pair.first;
    node_to_model_index[skeleton_root_node] = current_model_idx;
    model_index_to_skeleton_root[current_model_idx] = skeleton_root_node;

    // Propagate the model index down the skeleton hierarchy (bone nodes)
    std::function<void(const aiNode *)> assign_model_to_bone_children =
        [&](const aiNode *node) {
          if (is_bone_node_map.count(node)) {
            node_to_model_index[node] = current_model_idx;
            for (unsigned int i = 0; i < node->mNumChildren; ++i) {
              assign_model_to_bone_children(node->mChildren[i]);
            }
          }
        };
    assign_model_to_bone_children(skeleton_root_node);

    model new_model;
    new_model.name = skeleton_root_node->mName.C_Str();
    new_model.skeleton = pair.second; // Move the skeleton data
    new_model.has_skeleton = true;
    loaded_data.push_back(new_model);

    current_model_idx++;
  }

  // --- Step 5: Traverse the entire node hierarchy to assign remaining
  // nodes/meshes to models --- Nodes not part of a skeleton hierarchy but
  // having meshes or children with meshes will either be assigned to an
  // existing model if they are children of a skeleton node, or potentially
  // create new models for static assets. For simplicity here, we'll assign
  // non-bone nodes to the model of their closest ancestor that is part of a
  // skeleton hierarchy. Nodes with no bone ancestors and having meshes could be
  // treated as separate static models, but we'll skip them for now or require
  // further logic based on file structure.

  model static_mesh_model;
  static_mesh_model.name = "no skinning mesh";
  static_mesh_model.has_skeleton = false;

  std::function<void(const aiNode *, int)> assign_model_to_node_and_children =
      [&](const aiNode *node, int inherited_model_index) {
        int current_node_model_index = inherited_model_index;

        // If this node is a skeleton root, it already has a model index
        // assigned
        if (node_to_model_index.count(node)) {
          current_node_model_index = node_to_model_index.at(node);
        } else if (inherited_model_index != -1) {
          // Inherit model index from parent if not a skeleton root
          node_to_model_index[node] = inherited_model_index;
        } else {
          // If no inherited index and not a skeleton root, this node
          // and its children might be a separate static asset.
          // For this example, we'll only process nodes linked to skeletons.
          // You would add logic here to create new models for static nodes if
          // needed.
          node_to_model_index[node] = -1; // Mark as unassigned or static
        }

        // Process meshes attached to this node
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
          unsigned int mesh_index = node->mMeshes[i];
          if (mesh_index < scene->mNumMeshes) {
            const aiMesh *assimp_mesh = scene->mMeshes[mesh_index];
            if (assimp_mesh->mMaterialIndex >= scene->mNumMaterials) {
              continue;
            }

            if (assimp_mesh->HasBones() && (current_node_model_index == -1)) {
              std::vector<int> mathced_bone_counts(loaded_data.size(), 0);
              for (int cmi = 0; cmi < loaded_data.size(); cmi++) {
                auto &loaded_model = loaded_data[cmi];
                if (loaded_model.has_skeleton) {
                  for (int ambid = 0; ambid < assimp_mesh->mNumBones; ambid++) {
                    for (auto joint_name : loaded_model.skeleton.joint_names) {
                      if (joint_name ==
                          assimp_mesh->mBones[ambid]->mName.C_Str()) {
                        mathced_bone_counts[cmi]++;
                      }
                    }
                  }
                }
              }
              int max_matched_bone_counts = 0;
              for (int cmi = 0; cmi < loaded_data.size(); cmi++) {
                if (mathced_bone_counts[cmi] > max_matched_bone_counts) {
                  current_node_model_index = cmi;
                  max_matched_bone_counts = mathced_bone_counts[cmi];
                }
              }
              if (current_node_model_index != -1) {
                node_to_model_index[node] = current_node_model_index;
              }
            }

            if (assimp_mesh->HasBones() && current_node_model_index == -1) {
              spdlog::error("mesh {0} has bones but no matching armature found");
              continue;
            }

            mesh current_mesh;
            current_mesh.name =
                str_format("%s:%s", assimp_mesh->mName.C_Str(),
                           scene->mMaterials[assimp_mesh->mMaterialIndex]
                               ->GetName()
                               .C_Str());

            // Load Vertices
            current_mesh.vertices.resize(assimp_mesh->mNumVertices);
            for (unsigned int j = 0; j < assimp_mesh->mNumVertices; ++j) {
              current_mesh.vertices[j].position = toolkit::math::vector4(
                  assimp_mesh->mVertices[j].x, assimp_mesh->mVertices[j].y,
                  assimp_mesh->mVertices[j].z, 1.0f);
              if (assimp_mesh->HasNormals())
                current_mesh.vertices[j].normal = toolkit::math::vector4(
                    assimp_mesh->mNormals[j].x, assimp_mesh->mNormals[j].y,
                    assimp_mesh->mNormals[j].z, 0.0f);
              if (assimp_mesh->HasTextureCoords(0))
                current_mesh.vertices[j].tex_coords = toolkit::math::vector4(
                    assimp_mesh->mTextureCoords[0][j].x,
                    assimp_mesh->mTextureCoords[0][j].y,
                    assimp_mesh->mTextureCoords[0][j].z, 0.0f);
              if (assimp_mesh->HasVertexColors(0))
                current_mesh.vertices[j].color = toolkit::math::vector4(
                    assimp_mesh->mColors[0][j].r, assimp_mesh->mColors[0][j].g,
                    assimp_mesh->mColors[0][j].b, assimp_mesh->mColors[0][j].a);
              else
                current_mesh.vertices[j].color =
                    toolkit::math::vector4(1.0f, 1.0f, 1.0f, 1.0f);
            }

            // Load Indices
            for (unsigned int j = 0; j < assimp_mesh->mNumFaces; ++j) {
              const aiFace &face = assimp_mesh->mFaces[j];
              for (unsigned int k = 0; k < face.mNumIndices; ++k) {
                current_mesh.indices.push_back(face.mIndices[k]);
              }
            }

            // Load Bone Data (if skinned and model has a skeleton)
            if (assimp_mesh->HasBones() &&
                loaded_data[current_node_model_index].has_skeleton) {
              const skeleton &associated_skeleton =
                  loaded_data[current_node_model_index].skeleton;
              std::map<std::string, int> joint_name_to_index;
              for (size_t joint_idx = 0;
                   joint_idx < associated_skeleton.joint_names.size();
                   ++joint_idx) {
                joint_name_to_index[associated_skeleton
                                        .joint_names[joint_idx]] = joint_idx;
              }

              std::vector<int> bone_counts(assimp_mesh->mNumVertices, 0);

              for (unsigned int j = 0; j < assimp_mesh->mNumBones; ++j) {
                const aiBone *assimp_bone = assimp_mesh->mBones[j];
                std::string bone_name = assimp_bone->mName.C_Str();

                int bone_index_in_skeleton = -1;
                if (joint_name_to_index.count(bone_name)) {
                  bone_index_in_skeleton = joint_name_to_index.at(bone_name);
                } else {
                  spdlog::warn("Bone {0} not found in associated skeleton {1}.",
                               bone_name, associated_skeleton.name);
                  continue;
                }

                for (unsigned int k = 0; k < assimp_bone->mNumWeights; ++k) {
                  const aiVertexWeight &weight = assimp_bone->mWeights[k];
                  unsigned int vertex_id = weight.mVertexId;
                  float bone_weight = weight.mWeight;

                  if (vertex_id < current_mesh.vertices.size()) {
                    int &count = bone_counts[vertex_id];
                    if (count < 4) {
                      current_mesh.vertices[vertex_id].bone_indices[count] =
                          bone_index_in_skeleton;
                      current_mesh.vertices[vertex_id].bone_weights[count] =
                          bone_weight;
                      count++;
                    } else {
                      spdlog::warn("Vertex {0} has more than {1} bone weights "
                                   "in mesh {2}. Some weights will be ignored.",
                                   vertex_id, 4,
                                   current_mesh.name);
                    }
                  }
                }
              }

              // Normalize bone weights
              for (unsigned int j = 0; j < assimp_mesh->mNumVertices; ++j) {
                float total_weight = 0.0f;
                for (int k = 0; k < 4; ++k) {
                  total_weight += current_mesh.vertices[j].bone_weights[k];
                }
                if (total_weight > 0.0f) {
                  for (int k = 0; k < 4; ++k) {
                    current_mesh.vertices[j].bone_weights[k] /= total_weight;
                  }
                }
              }
            }

            // Load Blend Shapes
            if (assimp_mesh->mNumAnimMeshes > 0) {
              current_mesh.blendshapes.resize(assimp_mesh->mNumAnimMeshes);
              for (unsigned int j = 0; j < assimp_mesh->mNumAnimMeshes; ++j) {
                const aiAnimMesh *assimp_blend_shape =
                    assimp_mesh->mAnimMeshes[j];
                current_mesh.blendshapes[j].name =
                    assimp_blend_shape->mName.C_Str();
                current_mesh.blendshapes[j].weight = 0.0f; // Initialize weight

                current_mesh.blendshapes[j].data.resize(
                    assimp_blend_shape->mNumVertices);
                for (unsigned int k = 0; k < assimp_blend_shape->mNumVertices;
                     ++k) {
                  if (assimp_blend_shape->HasPositions())
                    current_mesh.blendshapes[j].data[k].offset_pos =
                        toolkit::math::vector4(
                            assimp_blend_shape->mVertices[k].x,
                            assimp_blend_shape->mVertices[k].y,
                            assimp_blend_shape->mVertices[k].z, 0.0f);
                  else
                    current_mesh.blendshapes[j].data[k].offset_pos =
                        toolkit::math::vector4(0.0f, 0.0f, 0.0f, 0.0f);

                  if (assimp_blend_shape->HasNormals())
                    current_mesh.blendshapes[j].data[k].offset_normal =
                        toolkit::math::vector4(
                            assimp_blend_shape->mNormals[k].x,
                            assimp_blend_shape->mNormals[k].y,
                            assimp_blend_shape->mNormals[k].z, 0.0f);
                  else
                    current_mesh.blendshapes[j].data[k].offset_normal =
                        toolkit::math::vector4(0.0f, 0.0f, 0.0f, 0.0f);
                }
              }
            }

            // Add the mesh to the current model
            if (current_node_model_index == -1)
              static_mesh_model.meshes.push_back(current_mesh);
            else
              loaded_data[current_node_model_index].meshes.push_back(
                  current_mesh);
          }
        }

        // Recursively process children nodes
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
          assign_model_to_node_and_children(node->mChildren[i],
                                            current_node_model_index);
        }
      };

  // Start the node traversal from the root
  // Root node itself doesn't have a specific model unless it's a skeleton root
  assign_model_to_node_and_children(scene->mRootNode, -1);
  // emplace back the static mesh model
  loaded_data.emplace_back(static_mesh_model);

  // --- Step 6: Clean up models with no meshes (optional) ---
  loaded_data.erase(
      std::remove_if(loaded_data.begin(), loaded_data.end(),
                     [](const model &m) { return m.meshes.empty(); }),
      loaded_data.end());

  return loaded_data;
}

}; // namespace toolkit::assets