#include "toolkit/sim/components/colliders.hpp"
#include "QuickHull.hpp"
#include "toolkit/sim/algorithms/algo.hpp"

namespace toolkit::sim {

math::vector3
sphere_collider::get_support(const math::vector3 &direction) const {
  return world_pos + direction.normalized() * radius;
}

math::vector3
capsule_collider::get_support(const math::vector3 &direction) const {
  math::vector3 norm_dir = direction.normalized();
  math::vector3 sphere_support = norm_dir * cap_radius;
  math::vector3 seg_dir = world_dir.normalized();
  float half_dist = 0.5f * cap_distance;
  math::vector3 seg_support;
  if (direction.dot(seg_dir) > 0.0f) {
    seg_support = world_pos + (half_dist * seg_dir);
  } else {
    seg_support = world_pos - (half_dist * seg_dir);
  }
  return seg_support + sphere_support;
}

math::vector3
convex_hull_collider::get_support(const math::vector3 &direction) const {
  math::vector3 norm_dir = direction.normalized();
  float max_dot = -std::numeric_limits<float>::max();
  math::vector3 point = math::vector3::Zero();
  for (int i = 0; i < transformed_vertices.size(); i++) {
    float dot = transformed_vertices[i].dot(norm_dir);
    if (dot > max_dot) {
      max_dot = dot;
      point = transformed_vertices[i];
    }
  }
  return point;
}

void convex_hull_collider::create_from_data(
    std::vector<assets::mesh_vertex> &vertices_data,
    math::vector3 world_scale) {
  quickhull::QuickHull<float> qh;
  std::vector<quickhull::Vector3<float>> points(vertices_data.size());
  for (int i = 0; i < vertices_data.size(); i++) {
    points[i].x = vertices_data[i].position.x() * world_scale.x();
    points[i].y = vertices_data[i].position.y() * world_scale.y();
    points[i].z = vertices_data[i].position.z() * world_scale.z();
  }
  auto hull = qh.getConvexHull(points, true, false);
  auto &index_buffer = hull.getIndexBuffer();
  auto &vertex_buffer = hull.getVertexBuffer();

  vertices.resize(vertex_buffer.size());
  indices.resize(index_buffer.size());
  faces.resize(index_buffer.size() / 3);
  vertex_to_faces.resize(vertices.size(), std::set<std::uint32_t>());
  vertex_to_neighbors.resize(vertices.size(), std::set<std::uint32_t>());
  face_to_neighbors.resize(faces.size(), std::set<std::uint32_t>());

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i].x() = vertex_buffer[i].x;
    vertices[i].y() = vertex_buffer[i].y;
    vertices[i].z() = vertex_buffer[i].z;
  }
  for (int i = 0; i < faces.size(); i++) {
    indices[3 * i + 0] = index_buffer[3 * i + 0];
    indices[3 * i + 1] = index_buffer[3 * i + 1];
    indices[3 * i + 2] = index_buffer[3 * i + 2];
    vertex_to_faces[index_buffer[3 * i + 0]].insert(i);
    vertex_to_faces[index_buffer[3 * i + 1]].insert(i);
    vertex_to_faces[index_buffer[3 * i + 2]].insert(i);
    vertex_to_neighbors[index_buffer[3 * i + 0]].insert(
        index_buffer[3 * i + 1]);
    vertex_to_neighbors[index_buffer[3 * i + 1]].insert(
        index_buffer[3 * i + 0]);
    vertex_to_neighbors[index_buffer[3 * i + 2]].insert(
        index_buffer[3 * i + 1]);
    vertex_to_neighbors[index_buffer[3 * i + 0]].insert(
        index_buffer[3 * i + 2]);
    vertex_to_neighbors[index_buffer[3 * i + 1]].insert(
        index_buffer[3 * i + 2]);
    vertex_to_neighbors[index_buffer[3 * i + 2]].insert(
        index_buffer[3 * i + 0]);
    faces[i].elements.push_back(index_buffer[3 * i + 0]);
    faces[i].elements.push_back(index_buffer[3 * i + 1]);
    faces[i].elements.push_back(index_buffer[3 * i + 2]);
    auto v0 = vertices[index_buffer[3 * i + 0]],
         v1 = vertices[index_buffer[3 * i + 1]],
         v2 = vertices[index_buffer[3 * i + 2]];
    // normals should always points outwards the mesh
    faces[i].normal = -((v1 - v0).cross(v2 - v0)).normalized();
  }
  for (int i = 0; i < faces.size(); i++) {
    auto n0 = vertex_to_faces[faces[i].elements[0]];
    auto n1 = vertex_to_faces[faces[i].elements[1]];
    auto n2 = vertex_to_faces[faces[i].elements[2]];
    std::set<int> face_count;
    for (auto n : n0)
      face_count.insert(n);
    for (auto n : n1)
      if (face_count.count(n))
        face_to_neighbors[i].insert(n);
      else
        face_count.insert(n);
    for (auto n : n2)
      if (face_count.count(n))
        face_to_neighbors[i].insert(n);
      else
        face_count.insert(n);
    face_to_neighbors[i].erase(i);
  }

  transformed_vertices = vertices;
  transformed_faces = faces;

  auto bs = welzl_bounding_sphere(vertices, true);
  bounding_sphere_center = bs.first;
  bounding_sphere_radius = bs.second;
}

void rigid_sim_object::setup_mass_inertia(float imass, bool is_fixed) {
  if (is_fixed) {
    inverse_mass = 0;
    fixed = is_fixed;
    inertia_tensor = math::matrix3::Zero();
    inverse_inertia_tensor = math::matrix3::Zero();
  } else {
    inverse_mass = imass;
    if (collider != nullptr) {
      if (collider->type == collider_type::SPHERE) {
        mass_center_offset = math::vector3::Zero();
        auto c = dynamic_cast<sphere_collider *>(collider.get());
        inertia_tensor =
            (2.0f / (5.0f * inverse_mass) * c->radius * c->radius) *
            math::matrix3::Identity();
      } else if (collider->type == collider_type::CAPSULE) {
        auto c = dynamic_cast<capsule_collider *>(collider.get());
        float r = c->cap_radius;
        float h = c->cap_distance;
        float m = 1.0f / inverse_mass;
        if (m <= 0.0f) {
          inertia_tensor = math::matrix3::Zero();
        } else {
          if (r < 0.0f)
            r = 0.0f;
          if (h < 0.0f)
            h = 0.0f;
          float v_cyl = 3.1415926f * r * r * h;
          float v_sphere = (4.0f / 3.0f) * 3.1415926f * r * r * r;
          float total_volume = v_cyl + v_sphere;
          float m_cyl = 0.0f;
          float m_sphere = 0.0f;
          if (total_volume > 1e-9f) {
            m_cyl = m * (v_cyl / total_volume);
            m_sphere = m * (v_sphere / total_volume);
          } else if (m > 0.0f) {
            if (h == 0.0f && r > 0.0f) {
              m_sphere = m;
            }
          }
          float I_yy = (0.5f * m_cyl + 0.4f * m_sphere) * r * r;
          float I_cyl_xx = (1.0f / 12.0f) * m_cyl * (3.0f * r * r + h * h);
          float I_xx_zz = (1.0f / 4.0f) * (m_cyl + m_sphere) * r * r +
                          (1.0f / 12.0f) * m_cyl * h * h +
                          (1.0f / 4.0f) * m_sphere * h * r +
                          (2.0f / 5.0f) * m_sphere * r * r;
          float I_xx_zz_term_r = (0.25f * m_cyl + 0.4f * m_sphere) * r * r;
          float I_xx_zz_term_h =
              (1.0f / 12.0f * m_cyl + 0.25f * m_sphere) * h * h;
          I_xx_zz = I_xx_zz_term_r + I_xx_zz_term_h;
          inertia_tensor = math::matrix3::Zero();
          inertia_tensor(0, 0) = I_xx_zz; // I_xx
          inertia_tensor(1, 1) = I_yy;    // I_yy
          inertia_tensor(2, 2) = I_xx_zz; // I_zz
        }
        mass_center_offset = math::vector3::Zero();
      } else if (collider->type == collider_type::CONVEX_HULL) {
        auto c = dynamic_cast<convex_hull_collider *>(collider.get());
        int num_vertices = c->vertices.size();
        float mass_per_vertex = 1.0f / inverse_mass / num_vertices;
        inertia_tensor = math::matrix3::Zero();
        mass_center_offset = math::vector3::Zero();
        for (int i = 0; i < num_vertices; i++)
          mass_center_offset += c->vertices[i];
        mass_center_offset /= num_vertices;
        for (int i = 0; i < num_vertices; i++) {
          auto v = c->vertices[i] - mass_center_offset;
          inertia_tensor(0, 0) +=
              mass_per_vertex * (v.y() * v.y() + v.z() * v.z());
          inertia_tensor(0, 1) += mass_per_vertex * v.x() * v.y();
          inertia_tensor(0, 2) += mass_per_vertex * v.x() * v.z();
          inertia_tensor(1, 0) += mass_per_vertex * v.x() * v.y();
          inertia_tensor(1, 1) +=
              mass_per_vertex * (v.x() * v.x() + v.z() * v.z());
          inertia_tensor(1, 2) += mass_per_vertex * v.y() * v.z();
          inertia_tensor(2, 0) += mass_per_vertex * v.x() * v.z();
          inertia_tensor(2, 1) += mass_per_vertex * v.y() * v.z();
          inertia_tensor(2, 2) +=
              mass_per_vertex * (v.x() * v.x() + v.y() * v.y());
        }
      }
      inverse_inertia_tensor = inertia_tensor.inverse();
    }
  }
}

nlohmann::json rigid_sim_object::late_serialize(entt::registry &registry,
                                                entt::entity entity) {
  nlohmann::json extra_data;
  if (collider->type == collider_type::SPHERE) {
    extra_data["collider_type"] = "sphere";
    extra_data["collider_data"] =
        *(dynamic_cast<sphere_collider *>(collider.get()));
  } else if (collider->type == collider_type::CAPSULE) {
    extra_data["collider_type"] = "capsule";
    extra_data["collider_data"] =
        *(dynamic_cast<capsule_collider *>(collider.get()));
  } else if (collider->type == collider_type::CAPSULE) {
    extra_data["collider_type"] = "convex_hull";
    extra_data["collider_data"] =
        *(dynamic_cast<convex_hull_collider *>(collider.get()));
  }
  return extra_data;
}

void rigid_sim_object::late_deserialize(entt::registry &registry,
                                        entt::entity entity,
                                        nlohmann::json &data) {
  // if any convex hull colliders, create it from data
  if (data.contains("collider_type")) {
    if (data["collider_type"].get<std::string>() == "sphere") {
      sphere_collider c = data["collider_data"];
      collider = std::make_shared<sphere_collider>(c);
    } else if (data["collider_type"].get<std::string>() == "capsule") {
      capsule_collider c = data["collider_data"];
      collider = std::make_shared<capsule_collider>(c);
    } else if (data["collider_type"].get<std::string>() == "cconvex_hull") {
      convex_hull_collider c = data["collider_data"];
      c.create_from_data(registry.get<opengl::mesh_data>(entity).vertices,
                         registry.get<transform>(entity).world_scl());
      collider = std::make_shared<convex_hull_collider>(c);
    }
    setup_mass_inertia(fixed ? 0.0f : inverse_mass, fixed);
  }
}

void rigid_sim_object::update_collider_properties() {
  if (!collider)
    return;
  if (auto collider_ptr = dynamic_cast<sphere_collider *>(collider.get())) {
    collider_ptr->world_pos =
        world_rotation * collider_ptr->local_pos + world_position;
  } else if (auto collider_ptr =
                 dynamic_cast<capsule_collider *>(collider.get())) {
    collider_ptr->world_pos =
        world_rotation * collider_ptr->local_pos + world_position;
    collider_ptr->world_rot =
        world_rotation * math::euler_to_quat(collider_ptr->local_angle);
    collider_ptr->world_dir = world_rotation * math::world_up;
  } else if (auto collider_ptr =
                 dynamic_cast<convex_hull_collider *>(collider.get())) {
    collider_ptr->transformed_bounding_sphere_center =
        world_rotation * collider_ptr->bounding_sphere_center + world_position;
    collider_ptr->world_pos = collider_ptr->transformed_bounding_sphere_center;
    for (int i = 0; i < collider_ptr->transformed_vertices.size(); i++) {
      collider_ptr->transformed_vertices[i] =
          world_rotation * collider_ptr->vertices[i] + world_position;
    }
    for (int i = 0; i < collider_ptr->transformed_faces.size(); i++) {
      collider_ptr->transformed_faces[i].normal =
          (world_rotation * collider_ptr->faces[i].normal).normalized();
    }
  }
  update_bounding_volumn_given_colliders();
  mass_center_world_space =
      world_rotation * mass_center_offset + world_position;
}

void rigid_sim_object::update_bounding_volumn_given_colliders() {
  if (collider == nullptr) {
    bounding_sphere_radius = 0.0f;
    bounding_sphere_center = math::vector3::Zero();
  } else {
    if (collider->type == collider_type::SPHERE) {
      auto c = dynamic_cast<sphere_collider *>(collider.get());
      bounding_sphere_radius = c->radius;
      bounding_sphere_center = c->world_pos;
    } else if (collider->type == collider_type::CAPSULE) {
      auto c = dynamic_cast<capsule_collider *>(collider.get());
      auto capsule_dir = c->world_rot * math::world_up;
      bounding_sphere_radius = c->cap_distance * 0.5f + c->cap_radius;
      bounding_sphere_center = c->world_pos;
    } else if (collider->type == collider_type::CONVEX_HULL) {
      auto c = dynamic_cast<convex_hull_collider *>(collider.get());
      bounding_sphere_center = c->transformed_bounding_sphere_center;
      bounding_sphere_radius = c->bounding_sphere_radius;
    }
  }
}

void rigid_sim_object::draw_gui(entt::registry &registry, entt::entity entity) {
  if (ImGui::BeginMenu("Collider Type")) {
    if (ImGui::MenuItem("Sphere Collider")) {
      sphere_collider c;
      collider = std::make_shared<sphere_collider>(c);
      setup_mass_inertia(fixed ? 0.0f : inverse_mass, fixed);
    }
    if (ImGui::MenuItem("Capsule Collider")) {
      capsule_collider c;
      collider = std::make_shared<capsule_collider>(c);
      setup_mass_inertia(fixed ? 0.0f : inverse_mass, fixed);
    }
    if (ImGui::MenuItem("Convex Hull Collider")) {
      auto mesh_ptr = registry.try_get<opengl::mesh_data>(entity);
      if (mesh_ptr == nullptr) {
        spdlog::error("Entity doesn't have mesh component, can't set to "
                      "convex collider");
      } else {
        spdlog::info("Create convex hull collider for mesh");
        convex_hull_collider c;
        c.create_from_data(mesh_ptr->vertices,
                           registry.get<transform>(entity).world_scl());
        collider = std::make_shared<convex_hull_collider>(c);
        setup_mass_inertia(fixed ? 0.0f : inverse_mass, fixed);
      }
    }
    ImGui::EndMenu();
  }

  ImGui::Checkbox("Active", &active);
  ImGui::DragFloat("Inv Mass", &inverse_mass, 0.0001f, 0.0f, 1e8f);
  if (ImGui::Button("Setup Mass Inertia", {-1, 30}))
    setup_mass_inertia(inverse_mass, inverse_mass == 0);

  if (collider != nullptr) {
    if (collider->type == collider_type::SPHERE) {
      ImGui::SeparatorText("Sphere Collider");
      sphere_collider *c = dynamic_cast<sphere_collider *>(collider.get());
      ImGui::DragFloat(("Radius##" + std::to_string(0)).c_str(), &(c->radius),
                       0.001f, 0.0f, 1e10f);
      ImGui::DragFloat3(("Local Pos##" + std::to_string(0)).c_str(),
                        c->local_pos.data(), 0.001f, -1e38f, 1e38f);
    } else if (collider->type == collider_type::CAPSULE) {
      ImGui::SeparatorText("Capsule Collider");
      capsule_collider *c = dynamic_cast<capsule_collider *>(collider.get());
      ImGui::DragFloat("Cap Radius", &c->cap_radius, 0.001f, 0.0f, 10.0f);
      ImGui::DragFloat("Cap Length", &c->cap_distance, 0.001f, 0.0f, 20.0f);
      ImGui::DragFloat3(("Local Pos##" + std::to_string(0)).c_str(),
                        c->local_pos.data(), 0.001f, -1e38f, 1e38f);
      ImGui::DragFloat3(("Local Rot##" + std::to_string(0)).c_str(),
                        c->local_angle.data(), 0.001f, -180.0f, 180.0f);
    } else if (collider->type == collider_type::CONVEX_HULL) {
      ImGui::SeparatorText("Convex Hull Collider");
      if (ImGui::Button("Update Convex", {-1, 30})) {
        dynamic_cast<convex_hull_collider *>(collider.get())
            ->create_from_data(registry.get<opengl::mesh_data>(entity).vertices,
                               registry.get<transform>(entity).world_scl());
      }
      if (ImGui::BeginTable("##convex_hull_collider", 2,
                            ImGuiTableFlags_Resizable |
                                ImGuiTableFlags_Borders)) {
        ImGui::TableSetupColumn("Property");
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Num Vertex");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%d", dynamic_cast<convex_hull_collider *>(collider.get())
                              ->vertices.size());

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Num Faces");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text(
            "%d",
            dynamic_cast<convex_hull_collider *>(collider.get())->faces.size());

        ImGui::EndTable();
      }
    }
  }
}
}; // namespace toolkit::sim