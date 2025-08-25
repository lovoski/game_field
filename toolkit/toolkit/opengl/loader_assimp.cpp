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
      // Access Assimp elements row-wise, copy to Eigen
      eigen_mat(i, j) = assimp_mat4[i][j];
    }
  }
  return eigen_mat; // Convert row-major→column-major
}

void process_mesh(entt::registry &registry, entt::entity container,
                  std::map<aiNode *, entt::entity> &node_mapping, aiMesh *mesh,
                  const aiScene *scene, const std::string model_name) {
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
    }
    // Normal
    if (mesh->HasNormals()) {
      aiVector3D norm = mesh->mNormals[i];
      v.normal = {norm.x, norm.y, norm.z, 0.0f};
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

  // // --- Process bones ---
  // for (unsigned int b = 0; b < mesh->mNumBones; b++) {
  //   aiBone *bone = mesh->mBones[b];
  //   std::string bone_name = bone->mName.C_Str();

  //   int bone_index;
  //   if (bone_map.find(bone_name) == bone_map.end()) {
  //     bone_index = (int)bone_map.size();
  //     bone_map[bone_name] = bone_index;
  //   } else {
  //     bone_index = bone_map[bone_name];
  //   }

  //   for (unsigned int w = 0; w < bone->mNumWeights; w++) {
  //     unsigned int vid = bone->mWeights[w].mVertexId;
  //     float weight = bone->mWeights[w].mWeight;

  //     // Assign to first empty slot
  //     for (int k = 0; k < 4; k++) {
  //       if (out.vertices[vid].bone_weights[k] == 0.0f) {
  //         out.vertices[vid].bone_indices[k] = bone_index;
  //         out.vertices[vid].bone_weights[k] = weight;
  //         break;
  //       }
  //     }
  //   }
  // }

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
      }
      if (anim_mesh->HasNormals()) {
        aiVector3D offset_n = anim_mesh->mNormals[i] - mesh->mNormals[i];
        bsv.offset_normal = {offset_n.x, offset_n.y, offset_n.z, 0.0f};
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
                    aiProcess_PopulateArmatureData);

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

  // handle node with meshes
  for (auto &p : node_mapping) {
    if (p.first->mNumMeshes > 0) {
      if (p.first->mNumMeshes == 1) {
        process_mesh(registry, p.second, node_mapping,
                     scene->mMeshes[p.first->mMeshes[0]], scene, filename);
      } else {
        auto &ptrans = registry.get<transform>(p.second);
        auto ptrans_mat = ptrans.update_matrix();
        for (int i = 0; i < p.first->mNumMeshes; i++) {
          auto ent = registry.create();
          auto &trans = registry.emplace<transform>(ent);
          trans.name = str_format(
              "%s:%s", scene->mMeshes[p.first->mMeshes[i]]->mName.C_Str(),
              scene
                  ->mMaterials[scene->mMeshes[p.first->mMeshes[i]]
                                   ->mMaterialIndex]
                  ->GetName()
                  .C_Str());
          trans.set_world_transform(ptrans_mat);
          ptrans.add_child(ent);
          process_mesh(registry, ent, node_mapping,
                       scene->mMeshes[p.first->mMeshes[i]], scene, filename);
        }
      }
    }
  }
}

}; // namespace toolkit::assets