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
  virtual void null_func() {}
};

struct sphere_collider : public base_collider {
  sphere_collider() { type = collider_type::SPHERE; }
  float radius = 1.0f;
  // local offset of the center to the attached sim_obj
  math::vector3 local_pos = math::vector3::Zero();
  // world position of the sphere center
  math::vector3 world_pos = math::vector3::Zero();
};
REFLECT(sphere_collider, radius, local_pos)

struct capsule_collider : public base_collider {
  capsule_collider() { type = collider_type::CAPSULE; }
  float cap_radius = 1.0f, cap_distance = 2.0f;
  math::vector3 local_pos = math::vector3::Zero(),
                world_pos = math::vector3::Zero();
  math::quat local_rot = math::quat::Identity(),
             world_rot = math::quat::Identity();
};
REFLECT(capsule_collider, cap_radius, cap_distance, local_pos, local_rot)

/**
 * This collider should only work when the entity with rigid_sim_object has a
 * static mesh (no skinning or morph targets) component.
 * The static mesh should also be simple enough to be used as a physics object.
 */
struct convex_hull_collider : public base_collider {
  convex_hull_collider() { type = collider_type::CONVEX_HULL; }
};

struct collider_contacts {
  math::vector3 contact_point1, contact_point2, normal;
};

struct physics_force {
  math::vector3 position = math::vector3::Zero(), force = math::vector3::Zero();
};

struct rigid_sim_object : public icomponent {
  rigid_sim_object() {}
  float inverse_mass = 1.0f;
  math::vector3 mass_center_offset = math::vector3::Zero(),
                mass_center_world_space = math::vector3::Zero();
  float static_friction_coeff = 1.0f, dynamic_friction_coeff = 0.9f;

  bool active = true, fixed = false;
  float deactivation_time = 0.0f;

  float bounding_sphere_radius = 0.0f;
  math::vector3 bounding_sphere_center = math::vector3::Zero();

  void update_bounding_volumn_given_colliders();

  std::vector<physics_force> forces;
  // the union of colliders form as an intergrity for this sim_obj
  std::vector<std::shared_ptr<base_collider>> colliders;

  math::vector3 angular_velocity = math::vector3::Zero(),
                linear_velocity = math::vector3::Zero();
  math::matrix3 inertia_tensor, inverse_inertia_tensor;
  math::vector3 world_position = math::vector3::Zero();
  math::quat world_rotation = math::quat::Identity();

  // auxilar
  math::vector3 prev_world_position = math::vector3::Zero();
  math::quat prev_world_rotation = math::quat::Identity();
  math::vector3 prev_angular_velocity = math::vector3::Zero(),
                prev_linear_velocity = math::vector3::Zero();

  nlohmann::json late_serialize() override;
  void late_deserialize(nlohmann::json &data) override;

  void draw_gui(iapp *app) override;
};
DECLARE_COMPONENT(rigid_sim_object, simulation)

}; // namespace toolkit::sim