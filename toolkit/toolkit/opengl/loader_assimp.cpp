#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/components/materials/all.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/scripts/test_draw.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace toolkit::assets {

// === Data container per mesh ===
struct MeshData {
  std::string mesh_name;
  std::string model_name;
  std::vector<mesh_vertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<blend_shape> blend_shapes;
};

// A map from aiBone name to bone index
using BoneMap = std::unordered_map<std::string, int>;

// === Function to process one mesh ===
MeshData ProcessMesh(aiMesh *mesh, const aiScene *scene,
                     const std::string &model_name, BoneMap &bone_map) {
  MeshData out;
  out.mesh_name = mesh->mName.C_Str();
  out.model_name = model_name;

  out.vertices.resize(mesh->mNumVertices);

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

    out.vertices[i] = v;
  }

  // --- Extract indices ---
  for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
    aiFace face = mesh->mFaces[f];
    for (unsigned int j = 0; j < face.mNumIndices; j++) {
      out.indices.push_back(face.mIndices[j]);
    }
  }

  // --- Process bones ---
  for (unsigned int b = 0; b < mesh->mNumBones; b++) {
    aiBone *bone = mesh->mBones[b];
    std::string bone_name = bone->mName.C_Str();

    int bone_index;
    if (bone_map.find(bone_name) == bone_map.end()) {
      bone_index = (int)bone_map.size();
      bone_map[bone_name] = bone_index;
    } else {
      bone_index = bone_map[bone_name];
    }

    for (unsigned int w = 0; w < bone->mNumWeights; w++) {
      unsigned int vid = bone->mWeights[w].mVertexId;
      float weight = bone->mWeights[w].mWeight;

      // Assign to first empty slot
      for (int k = 0; k < 4; k++) {
        if (out.vertices[vid].bone_weights[k] == 0.0f) {
          out.vertices[vid].bone_indices[k] = bone_index;
          out.vertices[vid].bone_weights[k] = weight;
          break;
        }
      }
    }
  }

  // --- Process blend shapes (morph targets) ---
  for (unsigned int m = 0; m < mesh->mNumAnimMeshes; m++) {
    aiAnimMesh *anim_mesh = mesh->mAnimMeshes[m];
    blend_shape shape;
    shape.weight =
        0.0f; // Assimp doesn't store weight here — handled by animation
    shape.name = "Morph_" + std::to_string(m);
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

    out.blend_shapes.push_back(shape);
  }

  return out;
}

// === Recursively process nodes ===
void ProcessNode(aiNode *node, const aiScene *scene,
                 const std::string &model_name,
                 std::vector<MeshData> &all_meshes, BoneMap &bone_map) {
  // Process all meshes in this node
  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
    all_meshes.push_back(ProcessMesh(mesh, scene, model_name, bone_map));
  }

  // Process all child nodes
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    ProcessNode(node->mChildren[i], scene, model_name, all_meshes, bone_map);
  }
}

// === Main load function ===
std::vector<MeshData> LoadModel(const std::string &path) {
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
      path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                aiProcess_CalcTangentSpace | aiProcess_GenNormals |
                aiProcess_LimitBoneWeights);

  if (!scene || !scene->mRootNode) {
    std::cerr << "Assimp failed: " << importer.GetErrorString() << std::endl;
    return {};
  }

  std::vector<MeshData> all_meshes;
  BoneMap bone_map;

  std::string model_name = path.substr(path.find_last_of("/\\") + 1);

  ProcessNode(scene->mRootNode, scene, model_name, all_meshes, bone_map);

  return all_meshes;
}

void open_model_assimp(entt::registry &registry, std::string filepath) {}

}; // namespace toolkit::assets