#pragma once

#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::sim {

enum collider_type {
  SPHERE,
  CAPSULE,
  CONVEX_HULL,
};

struct base_collider {
  collider_type type;
  math::vector3 world_pos = math::vector3::Zero(),
                local_pos = math::vector3::Zero();
  virtual math::vector3 get_support(const math::vector3 &direction) const = 0;
};

struct sphere_collider : public base_collider {
  sphere_collider() { type = collider_type::SPHERE; }
  float radius = 1.0f;
  // local offset of the center to the attached sim_obj
  math::vector3 local_pos = math::vector3::Zero();
  math::vector3 get_support(const math::vector3 &direction) const override;
};
REFLECT(sphere_collider, radius, local_pos)

/**
 * By default the capsule points to math::world_up
 */
struct capsule_collider : public base_collider {
  capsule_collider() { type = collider_type::CAPSULE; }
  float cap_radius = 1.0f, cap_distance = 2.0f;
  math::vector3 local_angle = math::vector3::Zero(), world_dir = math::world_up;
  math::quat world_rot = math::quat::Identity();
  math::vector3 get_support(const math::vector3 &direction) const override;
};
REFLECT(capsule_collider, cap_radius, cap_distance, local_pos, local_angle)

struct convex_hull_collider_face {
  math::vector3 normal;
  std::vector<std::uint32_t> elements;
};

/**
 * This collider should only work when the entity with rigid_sim_object has a
 * static mesh (no skinning or morph targets) component.
 * The static mesh should also be simple enough to be used as a physics object.
 */
struct convex_hull_collider : public base_collider {
  convex_hull_collider() { type = collider_type::CONVEX_HULL; }

  void create_from_data(std::vector<assets::mesh_vertex> &vertices_data,
                        math::vector3 world_scale);

  std::vector<math::vector3> vertices, transformed_vertices;
  std::vector<std::uint32_t> indices;
  std::vector<convex_hull_collider_face> faces, transformed_faces;

  float bounding_sphere_radius = 0.0f;
  math::vector3 bounding_sphere_center = math::vector3::Zero(),
                transformed_bounding_sphere_center = math::vector3::Zero();

  std::vector<std::set<std::uint32_t>> vertex_to_faces, vertex_to_neighbors,
      face_to_neighbors;

  math::vector3 get_support(const math::vector3 &direction) const override;
};
REFLECT(convex_hull_collider)

struct collider_contact {
  float penetration = 0.0f;
  math::vector3 contact_point1, contact_point2, normal;
};

struct physics_force {
  math::vector3 position = math::vector3::Zero(), force = math::vector3::Zero();
};

std::vector<collider_contact> colliders_get_contacts(base_collider *c1,
                                                     base_collider *c2);

struct rigid_sim_object : public icomponent {
  rigid_sim_object() { setup_mass_inertia(1.0f, false); }
  float inverse_mass = 1.0f;
  math::vector3 mass_center_offset = math::vector3::Zero(),
                mass_center_world_space = math::vector3::Zero();
  float static_friction_coeff = 1.0f, dynamic_friction_coeff = 0.9f;

  bool active = true, fixed = false;
  float deactivation_time = 0.0f;

  float bounding_sphere_radius = 0.0f;
  math::vector3 bounding_sphere_center = math::vector3::Zero();

  void setup_mass_inertia(float imass, bool is_fixed = false);

  void update_collider_properties();
  void update_bounding_volumn_given_colliders();

  std::vector<physics_force> forces;
  // the union of colliders form as an intergrity for this sim_obj
  std::shared_ptr<base_collider> collider = nullptr;

  math::matrix3 inertia_tensor, inverse_inertia_tensor;

  math::vector3 prev_world_position = math::vector3::Zero(),
                world_position = math::vector3::Zero();
  math::quat prev_world_rotation = math::quat::Identity(),
             world_rotation = math::quat::Identity();
  math::vector3 prev_angular_velocity = math::vector3::Zero(),
                prev_linear_velocity = math::vector3::Zero(),
                angular_velocity = math::vector3::Zero(),
                linear_velocity = math::vector3::Zero();

  nlohmann::json late_serialize(entt::registry &registry,
                                entt::entity entity) override;
  void late_deserialize(entt::registry &registry, entt::entity entity,
                        nlohmann::json &data) override;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(rigid_sim_object, simulation)

}; // namespace toolkit::sim