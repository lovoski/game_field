#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/components/materials/all.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/scripts/test_draw.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace toolkit::assets {

math::matrix4 convert_assimp_mat4(const aiMatrix4x4 &assimp_mat4) {
  Eigen::Matrix4f eigen_mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      eigen_mat(i, j) = assimp_mat4[i][j];
    }
  }
  return eigen_mat;
}

void build_skeleton(
    const entt::registry &registry, entt::entity cur,
    const std::map<aiNode *, entt::entity> &node_mapping,
    const std::map<std::string, entt::entity> &name_to_bone_entity,
    anim::actor &actor_comp) {
  auto &cur_trans = registry.get<transform>(cur);
  if (name_to_bone_entity.find(cur_trans.name) != name_to_bone_entity.end()) {
    actor_comp.ordered_entities.push_back(cur);
    actor_comp.name_to_entity[cur_trans.name] = cur;
  }
  for (auto c : cur_trans.m_children)
    build_skeleton(registry, c, node_mapping, name_to_bone_entity, actor_comp);
}

void collect_bone_entities(entt::registry &registry,
                           std::map<aiNode *, entt::entity> &node_mapping,
                           std::vector<entt::entity> &bone_entities,
                           const aiScene *scene) {
  std::map<std::string, entt::entity> name_to_bone_entity;
  for (auto &p : node_mapping) {
    for (int i = 0; i < p.first->mNumMeshes; i++) {
      auto mesh = scene->mMeshes[p.first->mMeshes[i]];
      for (int j = 0; j < mesh->mNumBones; j++) {
        auto assimp_bone = mesh->mBones[j];
        if (node_mapping.find(assimp_bone->mNode) == node_mapping.end()) {
          spdlog::error("Can't find bone node {0} from mesh {1}",
                        assimp_bone->mNode->mName.C_Str(), mesh->mName.C_Str());
          continue;
        }
        auto bone_entity = node_mapping[assimp_bone->mNode];
        auto bone_node_comp = registry.try_get<anim::bone_node>(bone_entity);
        auto &bone_entity_trans = registry.get<transform>(bone_entity);
        if (bone_node_comp == nullptr) {
          name_to_bone_entity[bone_entity_trans.name] = bone_entity;
          bone_entities.push_back(bone_entity);
          registry.emplace<anim::bone_node>(bone_entity);
          bone_node_comp = registry.try_get<anim::bone_node>(bone_entity);
        }
        bone_node_comp->name = bone_entity_trans.name;
        bone_node_comp->offset_matrix =
            convert_assimp_mat4(assimp_bone->mOffsetMatrix);
      }
    }
  }

  // build actor components
  std::vector<entt::entity> skeleton_roots;
  for (int i = 0; i < bone_entities.size(); i++) {
    auto cur = bone_entities[i];
    auto cur_trans = registry.try_get<transform>(cur);
    // find the first parent that's a bone entity
    while (cur_trans->m_parent != entt::null) {
      cur_trans = registry.try_get<transform>(cur_trans->m_parent);
      if (name_to_bone_entity.find(cur_trans->name) != name_to_bone_entity.end())
        break;
      cur = cur_trans->m_parent;
    }
    if (cur_trans->m_parent == entt::null)
      skeleton_roots.push_back(bone_entities[i]);
  }
  for (auto ent : skeleton_roots) {
    auto &actor_comp = registry.emplace<anim::actor>(ent);
    auto &vis_skel_script = registry.emplace<anim::vis_skeleton>(ent);
    build_skeleton(registry, ent, node_mapping, name_to_bone_entity,
                   actor_comp);
    actor_comp.joint_active.resize(actor_comp.ordered_entities.size(), true);
  }
}

void process_mesh(entt::registry &registry, entt::entity container,
                  std::map<aiNode *, entt::entity> &node_mapping, aiMesh *mesh,
                  const aiScene *scene, const std::string model_name,
                  std::vector<entt::entity> &bone_entities,
                  math::matrix4 pretrans) {
  auto &mesh_data = registry.emplace<opengl::mesh_data>(container);
  auto &bp_mat = registry.emplace<opengl::blinn_phong_material>(container);
  mesh_data.mesh_name =
      str_format("%s:%s", mesh->mName.C_Str(),
                 scene->mMaterials[mesh->mMaterialIndex]->GetName().C_Str());
  mesh_data.vertices.resize(mesh->mNumVertices);
  mesh_data.model_name = model_name;

  // --- Extract vertices ---
  for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
    mesh_vertex v;
    // Position
    if (mesh->HasPositions()) {
      aiVector3D pos = mesh->mVertices[i];
      v.position = {pos.x, pos.y, pos.z, 1.0f};
      v.position = pretrans * v.position;
    }
    // Normal
    if (mesh->HasNormals()) {
      aiVector3D norm = mesh->mNormals[i];
      v.normal = {norm.x, norm.y, norm.z, 0.0f};
      v.normal = pretrans * v.normal;
    }
    // UV
    if (mesh->HasTextureCoords(0)) {
      aiVector3D uv = mesh->mTextureCoords[0][i];
      v.tex_coords = {uv.x, uv.y, 0.0f, 0.0f};
    }
    // Color
    if (mesh->HasVertexColors(0)) {
      aiColor4D col = mesh->mColors[0][i];
      v.color = {col.r, col.g, col.b, col.a};
    } else {
      v.color = {1.0f, 1.0f, 1.0f, 1.0f};
    }
    // Bones
    v.bone_indices = {0, 0, 0, 0};
    v.bone_weights = {0.0f, 0.0f, 0.0f, 0.0f};

    mesh_data.vertices[i] = v;
  }

  // --- Extract indices ---
  for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
    aiFace face = mesh->mFaces[f];
    for (unsigned int j = 0; j < face.mNumIndices; j++) {
      mesh_data.indices.push_back(face.mIndices[j]);
    }
  }

  // --- Process bones ---
  for (unsigned int b = 0; b < mesh->mNumBones; b++) {
    auto assimp_bone = mesh->mBones[b];
    if (node_mapping.find(assimp_bone->mNode) == node_mapping.end()) {
      spdlog::error("Can't find bone node {0} from mesh {1}",
                    assimp_bone->mNode->mName.C_Str(), mesh->mName.C_Str());
      continue;
    }
    auto bone_entity = node_mapping[assimp_bone->mNode];

    int bone_index = 0;
    for (int i = 0; i < bone_entities.size(); i++) {
      if (bone_entities[i] == bone_entity) {
        bone_index = i;
        break;
      }
    }

    // Assign vertex weights
    for (unsigned int w = 0; w < assimp_bone->mNumWeights; w++) {
      unsigned int vid = assimp_bone->mWeights[w].mVertexId;
      float weight = assimp_bone->mWeights[w].mWeight;

      auto &v = mesh_data.vertices[vid];

      // Find slot for this bone (max 4 influences)
      int min_weight_index = 0;
      float min_weight = v.bone_weights[0];
      for (int k = 0; k < 4; k++) {
        if (v.bone_weights[k] == 0.0f) {
          v.bone_indices[k] = bone_index;
          v.bone_weights[k] = weight;
          min_weight = -1.0f; // no replacement
          break;
        }
        if (v.bone_weights[k] < min_weight) {
          min_weight = v.bone_weights[k];
          min_weight_index = k;
        }
      }

      // Replace weakest weight if current one is stronger
      if (min_weight >= 0.0f && weight > min_weight) {
        v.bone_indices[min_weight_index] = bone_index;
        v.bone_weights[min_weight_index] = weight;
      }
    }
  }
  // Normalize weights per vertex
  for (auto &v : mesh_data.vertices) {
    float sum = v.bone_weights[0] + v.bone_weights[1] + v.bone_weights[2] +
                v.bone_weights[3];
    if (sum > 0.0f) {
      for (int i = 0; i < 4; i++) {
        v.bone_weights[i] /= sum;
      }
    }
  }
  if (mesh->mNumBones > 0) {
    auto &bundle_data = registry.get<opengl::skinned_mesh_bundle>(
        node_mapping[scene->mRootNode]);
    bundle_data.mesh_entities.push_back(container);
  }

  // --- Process blend shapes (morph targets) ---
  for (unsigned int m = 0; m < mesh->mNumAnimMeshes; m++) {
    aiAnimMesh *anim_mesh = mesh->mAnimMeshes[m];
    blend_shape shape;
    shape.weight = 0.0f;
    shape.name = anim_mesh->mName.C_Str();
    shape.data.resize(mesh->mNumVertices);
    for (unsigned int i = 0; i < anim_mesh->mNumVertices; i++) {
      blend_shape_vertex bsv;
      if (anim_mesh->HasPositions()) {
        aiVector3D offset = anim_mesh->mVertices[i] - mesh->mVertices[i];
        bsv.offset_pos = {offset.x, offset.y, offset.z, 0.0f};
        bsv.offset_pos = pretrans * bsv.offset_pos;
      }
      if (anim_mesh->HasNormals()) {
        aiVector3D offset_n = anim_mesh->mNormals[i] - mesh->mNormals[i];
        bsv.offset_normal = {offset_n.x, offset_n.y, offset_n.z, 0.0f};
        bsv.offset_normal = pretrans * bsv.offset_normal;
      }
      shape.data[i] = bsv;
    }
    mesh_data.blend_shapes.push_back(shape);
  }

  opengl::init_opengl_buffers(mesh_data);
}

entt::entity
build_node_mapping(entt::registry &registry, aiNode *node, const aiScene *scene,
                   std::map<aiNode *, entt::entity> &node_mapping) {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  trans.name = node->mName.C_Str();
  trans.set_local_transform(convert_assimp_mat4(node->mTransformation));
  node_mapping[node] = ent;
  for (int i = 0; i < node->mNumChildren; i++) {
    auto child =
        build_node_mapping(registry, node->mChildren[i], scene, node_mapping);
    trans.add_child(child, false);
  }
  return ent;
}

void open_model_assimp(entt::registry &registry, std::string filepath) {
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
      filepath, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                    aiProcess_CalcTangentSpace | aiProcess_GenNormals |
                    aiProcess_LimitBoneWeights |
                    aiProcess_PopulateArmatureData | aiProcess_GlobalScale);

  if (!scene || !scene->mRootNode) {
    std::cerr << "Assimp failed: " << importer.GetErrorString() << std::endl;
    return;
  }

#ifdef _WIN32
  std::string filename =
      wstring_to_string(std::filesystem::u8path(filepath).filename().wstring());
#else
  std::string filename = std::filesystem::path(filepath).filename().string();
#endif

  // build node mapping
  std::map<aiNode *, entt::entity> node_mapping;
  auto root_entity =
      build_node_mapping(registry, scene->mRootNode, scene, node_mapping);
  auto &root_trans = registry.get<transform>(root_entity);
  root_trans.name = filename;
  root_trans.force_update_hierarchy();

  // build skinned mesh bundle
  std::vector<entt::entity> bone_entities;
  collect_bone_entities(registry, node_mapping, bone_entities, scene);
  if (bone_entities.size() > 0) {
    // there's bone nodes
    auto &bundle_comp =
        registry.emplace<opengl::skinned_mesh_bundle>(root_entity);
    bundle_comp.bone_entities = bone_entities;
  }

  // handle node with meshes
  for (auto &p : node_mapping) {
    if (p.first->mNumMeshes > 0) {
      auto &ptrans = registry.get<transform>(p.second);
      for (int i = 0; i < p.first->mNumMeshes; i++) {
        auto ent = registry.create();
        auto &trans = registry.emplace<transform>(ent);
        trans.name = str_format(
            "%s:%s:%d", scene->mMeshes[p.first->mMeshes[i]]->mName.C_Str(),
            scene
                ->mMaterials[scene->mMeshes[p.first->mMeshes[i]]
                                 ->mMaterialIndex]
                ->GetName()
                .C_Str(),
            i);
        ptrans.add_child(ent);
        process_mesh(registry, ent, node_mapping,
                     scene->mMeshes[p.first->mMeshes[i]], scene, filename,
                     bone_entities, ptrans.matrix());
      }
    }
  }
  root_trans.force_update_hierarchy();

  // handle animation track loading if any
  // ...
}

}; // namespace toolkit::assets