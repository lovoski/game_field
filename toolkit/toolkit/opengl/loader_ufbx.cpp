#include "toolkit/opengl/editor.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/components/materials/all.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/scripts/test_draw.hpp"
#include <ufbx.h>

namespace toolkit::assets {

#ifdef _WIN32
#include <Windows.h>
std::string wstring_to_string(const std::wstring &wstr) {
  int buffer_size = WideCharToMultiByte(
      CP_UTF8,         // UTF-8 encoding for Chinese support
      0,               // No flags
      wstr.c_str(),    // Input wide string
      -1,              // Auto-detect length
      nullptr, 0,      // Null to calculate required buffer size
      nullptr, nullptr // Optional parameters (not needed here)
  );

  if (buffer_size == 0)
    return ""; // Handle error if needed

  std::string str(buffer_size, 0);
  WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &str[0], buffer_size,
                      nullptr, nullptr);
  str.pop_back(); // Remove null terminator added by -1
  return str;
}
#endif

math::vector2 ufbx_to_vec2(ufbx_vec2 v) {
  return math::vector2((float)v.x, (float)v.y);
}
math::vector3 ufbx_to_vec3(ufbx_vec3 v) {
  return math::vector3((float)v.x, (float)v.y, (float)v.z);
}
math::quat ufbx_to_quat(ufbx_quat v) {
  return math::quat((float)v.w, (float)v.x, (float)v.y, (float)v.z);
}
math::matrix4 ufbx_to_mat(ufbx_matrix m) {
  math::matrix4 result = math::matrix4::Zero();
  result(0, 0) = (float)m.m00;
  result(0, 1) = (float)m.m01;
  result(0, 2) = (float)m.m02;
  result(0, 3) = (float)m.m03;
  result(1, 0) = (float)m.m10;
  result(1, 1) = (float)m.m11;
  result(1, 2) = (float)m.m12;
  result(1, 3) = (float)m.m13;
  result(2, 0) = (float)m.m20;
  result(2, 1) = (float)m.m21;
  result(2, 2) = (float)m.m22;
  result(2, 3) = (float)m.m23;
  result(3, 3) = 1;
  return result;
}

ufbx_node *find_first_bone_parent(ufbx_node *node) {
  while (node) {
    if (node->bone)
      return node;
    node = node->parent;
  }
  return nullptr;
}

ufbx_node *find_last_bone_parent(ufbx_node *node) {
  ufbx_node *bone_parent = nullptr;
  while (node) {
    if (node->bone)
      bone_parent = node;
    node = node->parent;
  }
  return bone_parent;
}

std::map<ufbx_node *, entt::entity>
read_nodes(entt::registry &registry, ufbx_scene *scene, std::string filename) {
  std::map<ufbx_node *, entt::entity> ufbx_node_to_entity;
  // create node hierarchy given the fbx scene nodes, each node correspond to
  // one entity in the scene.
  for (int i = 0; i < scene->nodes.count; i++) {
    auto unode = scene->nodes[i];
    auto ent = registry.create();
    auto &trans = registry.emplace<transform>(ent);
    trans.name = unode->is_root ? filename : std::string(unode->name.data);
    ufbx_node_to_entity[unode] = ent;
  }
  // setup parent child hierarchy
  for (int i = 0; i < scene->nodes.count; i++) {
    auto unode = scene->nodes[i];
    if (unode->parent) {
      auto ent = ufbx_node_to_entity[unode];
      auto &trans = registry.get<transform>(ent);
      auto p_ent = ufbx_node_to_entity[unode->parent];
      auto &p_trans = registry.get<transform>(p_ent);
      p_trans.add_child(ent);
      trans.set_local_pos(ufbx_to_vec3(unode->local_transform.translation));
      trans.set_local_rot(ufbx_to_quat(unode->local_transform.rotation));
      trans.set_local_scale(ufbx_to_vec3(unode->local_transform.scale));
    }
  }
  if (scene->nodes.count > 0) {
    registry.ctx().get<iapp *>()->get_sys<transform_system>()->update_transform(
        registry);
  }
  // find root bones, add actor and bone_node component to them.
  std::vector<ufbx_node *> root_nodes;
  for (int i = 0; i < scene->nodes.count; i++) {
    if (scene->nodes[i]->is_root)
      root_nodes.push_back(scene->nodes[i]);
    if (scene->nodes[i]->bone) {
      auto &bone_comp = registry.emplace<anim::bone_node>(
          ufbx_node_to_entity[scene->nodes[i]]);
      bone_comp.name = std::string(scene->nodes[i]->name.data);
    }
  }
  for (auto root_node : root_nodes) {
    std::stack<std::pair<ufbx_node *, ufbx_node *>> q;
    std::vector<std::pair<ufbx_node *, ufbx_node *>> bone_nodes;
    q.push(std::make_pair(root_node, nullptr));
    while (!q.empty()) {
      auto node = q.top();
      if (node.first->bone)
        bone_nodes.push_back(node);
      q.pop();
      for (auto c : node.first->children) {
        q.push(std::make_pair(c, find_first_bone_parent(node.first)));
      }
    }
    if (bone_nodes.size() > 0) {
      // use bone nodes to create a skeleton
      std::vector<ufbx_node *> roots;
      for (int i = 0; i < bone_nodes.size(); i++) {
        if (bone_nodes[i].second == nullptr)
          roots.push_back(bone_nodes[i].first);
      }
      for (auto root : roots) {
        auto root_ent = ufbx_node_to_entity[root];
        std::vector<std::pair<ufbx_node *, ufbx_node *>> bone_nodes_sub;
        for (int i = 0; i < bone_nodes.size(); i++)
          if (find_last_bone_parent(bone_nodes[i].first) == root)
            bone_nodes_sub.push_back(bone_nodes[i]);
        int joint_num = bone_nodes_sub.size();
        auto &actor_comp = registry.emplace<anim::actor>(root_ent);
        auto &vis_script = registry.emplace<anim::vis_skeleton>(root_ent);
        actor_comp.joint_active.resize(joint_num, true);
        actor_comp.ordered_entities.resize(joint_num);
        std::map<entt::entity, int> ent_to_joint_id;
        for (int i = 0; i < joint_num; i++) {
          auto joint_ent = ufbx_node_to_entity[bone_nodes_sub[i].first];
          auto &joint_trans = registry.get<transform>(joint_ent);
          actor_comp.name_to_entity[joint_trans.name] = joint_ent;

          ent_to_joint_id[joint_ent] = i;
          actor_comp.ordered_entities[i] = joint_ent;
        }
      }
    }
  }
  return ufbx_node_to_entity;
}

struct skin_vertex {
  float bone_weights[4];
  unsigned int bone_ids[4];
};

void read_mesh(entt::registry &registry, ufbx_mesh *mesh,
               std::map<ufbx_node *, entt::entity> &ufbx_node_to_entity,
               ufbx_scene *scene, std::string filename) {
  bool skinned_mesh = mesh->skin_deformers.count > 0;
  std::vector<skin_vertex> skin_vertices;
  if (skinned_mesh) {
    auto bundle_entity = ufbx_node_to_entity[scene->root_node];
    auto &bundle_comp =
        registry.get<opengl::skinned_mesh_bundle>(bundle_entity);
    // use the first skin deformer only
    auto skin = mesh->skin_deformers.data[0];
    // collect weights for each skinned vertex
    skin_vertices.resize(mesh->num_vertices);
    for (int i = 0; i < mesh->num_vertices; i++) {
      int num_weights = 0;
      float total_weight = 0.0f;
      float weights[4] = {0.0f};
      entt::entity bone_entities[4];
      ufbx_skin_vertex vertex_weights = skin->vertices.data[i];
      for (int j = 0; j < vertex_weights.num_weights; j++) {
        if (num_weights > 4)
          break;
        ufbx_skin_weight weight =
            skin->weights.data[vertex_weights.weight_begin + j];
        if (weight.cluster_index < 4) {
          total_weight += (float)weight.weight;
          bone_entities[num_weights] =
              ufbx_node_to_entity[skin->clusters[weight.cluster_index]
                                      ->bone_node];
          weights[num_weights] = (float)weight.weight;
          num_weights++;
        }
      }
      if (total_weight > 0.0f) {
        auto &skin_vert = skin_vertices[i];
        for (int j = 0; j < 4; j++) {
          skin_vert.bone_weights[j] = weights[j] / total_weight;
          skin_vert.bone_ids[j] = 0;
          // map to entity indices inside bundle component
          for (int k = 0; k < bundle_comp.bone_entities.size(); k++) {
            if (bundle_comp.bone_entities[k] == bone_entities[j])
              skin_vert.bone_ids[j] = k;
          }
        }
      }
    }
  }

  // create one mesh component per part
  auto mesh_base_entity = ufbx_node_to_entity[mesh->instances.data[0]];
  auto &mesh_base_trans = registry.get<transform>(mesh_base_entity);
  for (int pi = 0; pi < mesh->material_parts.count; pi++) {
    ufbx_mesh_part *mesh_part = &mesh->material_parts.data[pi];
    if (mesh_part->num_triangles == 0)
      continue;

    auto mesh_entity = registry.create();
    auto &mesh_trans = registry.emplace<transform>(mesh_entity);
    auto &mesh_comp = registry.emplace<opengl::mesh_data>(mesh_entity);
    auto &material =
        registry.emplace<opengl::blinn_phong_material>(mesh_entity);
    if (mesh->materials.count >= pi + 1) {
      auto material = mesh->materials[pi];
      mesh_trans.name =
          str_format("%s:%s:%s", mesh->instances.data[0]->name.data,
                     mesh->name.data, material->name.data);
      mesh_comp.mesh_name =
          str_format("%s:%s", mesh->name.data, material->name.data);
    } else {
      mesh_trans.name =
          str_format("%s:%s:%d", mesh->instances.data[0]->name.data,
                     mesh->name.data, mesh_part->index);
      mesh_comp.mesh_name =
          str_format("%s:%d", mesh->name.data, mesh_part->index);
    }
    mesh_comp.model_name = filename;
    mesh_base_trans.add_child(mesh_entity);

    if (skinned_mesh) {
      auto &bundle_comp = registry.get<opengl::skinned_mesh_bundle>(
          ufbx_node_to_entity[scene->root_node]);
      bundle_comp.mesh_entities.push_back(mesh_entity);
    }

    mesh_comp.indices.resize(mesh_part->num_triangles * 3);
    int num_indices = 0;
    for (int face_index : mesh_part->face_indices) {
      ufbx_face face = mesh->faces[face_index];
      int num_tris = ufbx_triangulate_face(
          mesh_comp.indices.data(), mesh_comp.indices.size(), mesh, face);
      for (int i = 0; i < num_tris * 3; i++) {
        int index = mesh_comp.indices[i];
        assets::mesh_vertex vertex;
        vertex.position << ufbx_to_vec3(mesh->vertex_position[index]), 1.0f;
        vertex.normal << ufbx_to_vec3(mesh->vertex_normal[index]), 0.0f;
        if (mesh->uv_sets.count > 0) {
          vertex.tex_coords.x() = mesh->vertex_uv[index].x;
          vertex.tex_coords.y() = mesh->vertex_uv[index].y;
          if (mesh->uv_sets.count > 1) {
            vertex.tex_coords.z() = mesh->uv_sets[1].vertex_uv[index].x;
            vertex.tex_coords.w() = mesh->uv_sets[1].vertex_uv[index].y;
          } else {
            vertex.tex_coords.z() = 0;
            vertex.tex_coords.w() = 0;
          }
        }
        if (mesh->vertex_color.exists) {
          vertex.color.x() = mesh->vertex_color[index].x;
          vertex.color.y() = mesh->vertex_color[index].y;
          vertex.color.z() = mesh->vertex_color[index].z;
          vertex.color.w() = mesh->vertex_color[index].w;
        }

        // handle skinned mesh import
        for (int k = 0; k < 4; k++) {
          vertex.bone_indices[k] = 0;
          vertex.bone_weights[k] = 0.0f;
        }
        if (skinned_mesh) {
          auto bundle_entity = ufbx_node_to_entity[scene->root_node];
          auto &bundle_comp =
              registry.get<opengl::skinned_mesh_bundle>(bundle_entity);
          auto skin = mesh->skin_deformers.data[0];
          auto vertex_id = mesh->vertex_indices[index];
          auto skin_vertex = skin->vertices[vertex_id];
          auto num_weights =
              skin_vertex.num_weights > 4 ? 4 : skin_vertex.num_weights;
          float total_weights = 0.0f;
          for (int k = 0; k < num_weights; k++) {
            auto skin_weight = skin->weights[skin_vertex.weight_begin + k];
            auto bone_entity =
                ufbx_node_to_entity[skin->clusters[skin_weight.cluster_index]
                                        ->bone_node];
            for (int bone_id = 0; bone_id < bundle_comp.bone_entities.size();
                 bone_id++) {
              if (bundle_comp.bone_entities[bone_id] == bone_entity) {
                vertex.bone_indices[k] = bone_id;
                vertex.bone_weights[k] = (float)skin_weight.weight;
                total_weights += (float)skin_weight.weight;
              }
            }
          }
          if (total_weights != 0.0f) {
            for (int k = 0; k < num_weights; k++) {
              vertex.bone_weights[k] /= total_weights;
            }
          } else {
            // parent this mesh to root joint if it's not binded
            vertex.bone_indices[0] = 0;
            vertex.bone_weights[0] = 1.0f;
          }
        }

        // handle blend shape import
        if (mesh->blend_deformers.count > 0) {
          auto blend_deformer = mesh->blend_deformers.data[0];
          uint32_t vert_id = mesh->vertex_indices[index];
          size_t num_blends = blend_deformer->channels.count;
          mesh_comp.blend_shapes.resize(num_blends);
          for (int bsi = 0; bsi < num_blends; bsi++) {
            ufbx_blend_channel *channel = blend_deformer->channels[bsi];
            ufbx_blend_shape *shape = channel->target_shape;
            mesh_comp.blend_shapes[bsi].name = shape->name.data;
            blend_shape_vertex bs_vertex;
            bs_vertex.offset_pos << ufbx_to_vec3(
                ufbx_get_blend_shape_vertex_offset(shape, vert_id)),
                0.0f;
            bs_vertex.offset_normal = math::vector4::Zero();
            mesh_comp.blend_shapes[bsi].data.emplace_back(bs_vertex);
          }
        }
        mesh_comp.vertices.push_back(vertex);
        num_indices++;
      }
    }

    assert(mesh_comp.vertices.size() == mesh_part->num_triangles * 3);

    ufbx_vertex_stream streams[1000];
    int num_streams = mesh_comp.blend_shapes.size() == 0
                          ? 1
                          : 1 + mesh_comp.blend_shapes.size();
    streams[0].data = mesh_comp.vertices.data();
    streams[0].vertex_count = num_indices;
    streams[0].vertex_size = sizeof(assets::mesh_vertex);
    for (int i = 1; i < mesh_comp.blend_shapes.size() + 1; i++) {
      streams[i].data = mesh_comp.blend_shapes[i - 1].data.data();
      streams[i].vertex_count = num_indices;
      streams[i].vertex_size = sizeof(assets::blend_shape_vertex);
    }

    mesh_comp.indices.resize(mesh_part->num_triangles * 3);
    int num_vertices =
        ufbx_generate_indices(streams, num_streams, mesh_comp.indices.data(),
                              mesh_comp.indices.size(), nullptr, nullptr);
    mesh_comp.vertices.resize(num_vertices);

    opengl::init_opengl_buffers(mesh_comp);
  }
}

std::vector<entt::entity> create_skinned_mesh_bundle_data(
    entt::registry &registry, ufbx_scene *scene,
    std::map<ufbx_node *, entt::entity> &ufbx_node_to_entity) {
  std::vector<entt::entity> bone_entities;
  for (int i = 0; i < scene->nodes.count; i++) {
    auto unode = scene->nodes[i];
    auto entity = ufbx_node_to_entity[unode];
    if (unode->bone) {
      bone_entities.push_back(entity);
    }
  }
  if (bone_entities.size() > 0) {
    for (int i = 0; i < scene->meshes.count; i++) {
      if (scene->meshes[i]->skin_deformers.count > 0) {
        auto deformer = scene->meshes[i]->skin_deformers.data[0];
        for (int j = 0; j < deformer->clusters.count; j++) {
          auto cluster = deformer->clusters[j];
          auto bone_entity = ufbx_node_to_entity[cluster->bone_node];
          for (int k = 0; k < bone_entities.size(); k++) {
            if (bone_entities[k] == bone_entity) {
              auto &bone_node = registry.get<anim::bone_node>(bone_entity);
              bone_node.offset_matrix = ufbx_to_mat(cluster->geometry_to_bone);
              break;
            }
          }
        }
      }
    }
  }
  return bone_entities;
}

void open_model_ufbx(entt::registry &registry, std::string filepath) {
  ufbx_error error;
  ufbx_load_opts opts = {
      .load_external_files = true,
      .ignore_missing_external_files = true,
      .generate_missing_normals = true,
      .target_axes =
          {
              .right = UFBX_COORDINATE_AXIS_POSITIVE_X,
              .up = UFBX_COORDINATE_AXIS_POSITIVE_Y,
              .front = UFBX_COORDINATE_AXIS_POSITIVE_Z,
          },
      .target_unit_meters = 1.0f,
  };
  ufbx_scene *scene = ufbx_load_file(filepath.c_str(), &opts, &error);
  if (!scene) {
    spdlog::error("Failed to load: {0}", error.description.data);
    return;
  }

// create nodes
#ifdef _WIN32
  std::string filename =
      wstring_to_string(std::filesystem::u8path(filepath).filename().wstring());
#else
  std::string filename = std::filesystem::path(filepath).filename().string();
#endif
  auto ufbx_node_to_entity = read_nodes(registry, scene, filename);
  // create skinned mesh bundle
  auto bone_entities =
      create_skinned_mesh_bundle_data(registry, scene, ufbx_node_to_entity);
  if (bone_entities.size() > 0) {
    // create skinned mesh bundle if there's any bone
    // use the root node's entity as bundle entity holder
    auto bundle_entity = ufbx_node_to_entity[scene->root_node];
    auto &bundle_comp =
        registry.emplace<opengl::skinned_mesh_bundle>(bundle_entity);
    bundle_comp.bone_entities = bone_entities;
  }
  // create meshes
  for (int i = 0; i < scene->meshes.count; i++) {
    read_mesh(registry, scene->meshes[i], ufbx_node_to_entity, scene, filename);
  }

  ufbx_free_scene(scene);
}

}; // namespace toolkit::assets
