#include "toolkit/opengl/editor.hpp"
#include "toolkit/sim/components/colliders.hpp"

#include "toolkit/opengl/draw.hpp"
#include "toolkit/sim/systems.hpp"

namespace toolkit::sim {

void phy_system::fixedupdate(entt::registry &registry, float dt) {
  auto sim_obj = registry.view<transform, rigid_sim_object>();
  std::vector<sim_obj_data> obj_data;
  sim_obj.each(
      [&](entt::entity entity, transform &trans, rigid_sim_object &sim_obj) {
        sim_obj_data entity_sim_data;
        entity_sim_data.entity = entity;
        entity_sim_data.trans = &trans;
        entity_sim_data.sim_obj = &sim_obj;
        obj_data.push_back(entity_sim_data);
      });
  // update center of mass with local offset
  for (auto &single_obj_data : obj_data) {
    single_obj_data.sim_obj->mass_center_world_space =
        single_obj_data.sim_obj->world_rotation *
            single_obj_data.sim_obj->mass_center_offset +
        single_obj_data.sim_obj->world_position;
  }
  // update the collider properties
  update_collider_properties(registry, obj_data);
  // apply gravity to all objects
  {
    physics_force gravity_force;
    gravity_force.force = gravity;
    sim_obj.each(
        [&](entt::entity entity, transform &trans, rigid_sim_object &sim_obj) {
          gravity_force.position = sim_obj.mass_center_world_space;
          sim_obj.forces.push_back(gravity_force);
        });
  }

  // ------------- start PBD based simulation -------------
  {
    // broad phase collision detection
    auto broad_collision_pairs =
        get_broadphase_collision_pairs(registry, obj_data);

    float step_dt = dt / num_sub_steps;
    for (int i = 0; i < num_sub_steps; i++) {
      // compute contact constraints in each sub step
      for (int j = 0; j < obj_data.size(); j++) {
        auto sim_obj = obj_data[j].sim_obj;
        // store previous position and rotation
        sim_obj->prev_world_position = obj_data[j].sim_obj->world_position;
        sim_obj->prev_world_rotation = obj_data[j].sim_obj->world_rotation;
        if (sim_obj->fixed || !(sim_obj->active))
          continue;
        // compute external force and torque
        math::vector3 ext_force = math::vector3::Zero();
        math::vector3 ext_torque = math::vector3::Zero();
        for (int k = 0; k < sim_obj->forces.size(); k++) {
          ext_force += sim_obj->forces[k].force;
          ext_torque +=
              (sim_obj->forces[k].position - sim_obj->mass_center_world_space)
                  .cross(sim_obj->forces[k].force);
        }
        // update object position and linear velocity given external force
        sim_obj->linear_velocity += step_dt * sim_obj->inverse_mass * ext_force;
        sim_obj->world_position += step_dt * sim_obj->linear_velocity;
        // update object rotation and angular velocity given external torque
      }
    }
  }
  // ------------- end PBD based simulation -------------

  // remove froces from all objects
  {
    sim_obj.each([&](entt::entity entity, transform &trans,
                     rigid_sim_object &sim_obj) { sim_obj.forces.clear(); });
  }
}

void phy_system::update_collider_properties(
    entt::registry &registry, std::vector<sim_obj_data> &obj_data) {
  for (auto &data : obj_data) {
    if (auto collider_ptr =
            dynamic_cast<sphere_collider *>(data.sim_obj->collider.get())) {
      collider_ptr->world_pos =
          data.sim_obj->world_rotation * collider_ptr->local_pos +
          data.sim_obj->world_position;
    } else if (auto collider_ptr = dynamic_cast<capsule_collider *>(
                   data.sim_obj->collider.get())) {
      collider_ptr->world_pos =
          data.sim_obj->world_rotation * collider_ptr->local_pos +
          data.sim_obj->world_position;
      collider_ptr->world_rot =
          data.sim_obj->world_rotation * collider_ptr->local_rot;
    } else if (auto collider_ptr = dynamic_cast<convex_hull_collider *>(
                   data.sim_obj->collider.get())) {
      for (int i = 0; i < collider_ptr->transformed_vertices.size(); i++) {
        collider_ptr->transformed_vertices[i] =
            data.sim_obj->world_rotation * collider_ptr->vertices[i] +
            data.sim_obj->world_position;
      }
      for (int i = 0; i < collider_ptr->transformed_faces.size(); i++) {
        collider_ptr->transformed_faces[i].normal =
            (data.sim_obj->world_rotation * collider_ptr->faces[i].normal)
                .normalized();
      }
    }
    data.sim_obj->update_bounding_volumn_given_colliders();
  }
}

std::vector<std::pair<entt::entity, entt::entity>>
phy_system::get_broadphase_collision_pairs(
    entt::registry &registry, std::vector<sim_obj_data> &obj_data) {
  std::vector<std::pair<entt::entity, entt::entity>> collision_pairs;
  for (int i = 0; i < obj_data.size(); i++) {
    for (int j = i + 1; j < obj_data.size(); j++) {
      // use bounding sphere for detection
      float center_dist = (obj_data[i].sim_obj->bounding_sphere_center -
                           obj_data[j].sim_obj->bounding_sphere_center)
                              .norm();
      // increase the collision distance in account for moving objects
      if (center_dist <= (obj_data[i].sim_obj->bounding_sphere_radius +
                          obj_data[j].sim_obj->bounding_sphere_radius + 0.1f))
        collision_pairs.emplace_back(
            std::make_pair(obj_data[i].entity, obj_data[j].entity));
    }
  }
  return collision_pairs;
}

void phy_system::draw_gui(entt::registry &registry, entt::entity entity) {
  if (auto ptr = registry.try_get<rigid_sim_object>(entity)) {
    if (ImGui::CollapsingHeader("PBD Physics Object"))
      ptr->draw_gui(registry, entity);
  }
}

void phy_system::draw_menu_gui() {
  ImGui::MenuItem("Settings", nullptr, nullptr, false);
  if (ImGui::InputInt("Num Steps", &num_sub_steps)) {
    num_sub_steps = num_sub_steps >= 0 ? num_sub_steps : 0;
  }
  if (ImGui::InputInt("Simulate FPS", &sim_fps)) {
    sim_fps = sim_fps >= 0 ? sim_fps : 0;
  }
}

void phy_system::draw_to_scene(entt::registry &registry, transform &cam_trans,
                               camera &cam_comp) {
  registry.view<transform, rigid_sim_object>().each(
      [&](entt::entity entity, transform &trans, rigid_sim_object &sim_obj) {
        if (sim_obj.collider != nullptr) {
          if (sim_obj.collider->type == collider_type::SPHERE) {
            auto collider_ptr =
                dynamic_cast<sphere_collider *>(sim_obj.collider.get());
            opengl::draw_sphere(collider_ptr->world_pos, cam_comp.vp,
                                collider_ptr->radius, opengl::White, true);
          } else if (sim_obj.collider->type == collider_type::CAPSULE) {
          } else if (sim_obj.collider->type == collider_type::CONVEX_HULL) {
            auto collider_ptr =
                dynamic_cast<convex_hull_collider *>(sim_obj.collider.get());
            std::vector<assets::mesh_vertex> vertices(
                collider_ptr->vertices.size());
            for (int i = 0; i < collider_ptr->vertices.size(); i++)
              vertices[i].position << collider_ptr->transformed_vertices[i],
                  1.0;
            opengl::draw_mesh(vertices, collider_ptr->indices, cam_comp.vp,
                              opengl::White);
          }
          // draw bounding sphere of collider
          opengl::draw_sphere(sim_obj.bounding_sphere_center, cam_comp.vp,
                              sim_obj.bounding_sphere_radius, opengl::Green,
                              true);
          // draw mass center
          opengl::draw_sphere(sim_obj.mass_center_world_space, cam_comp.vp,
                              0.01f, opengl::Purple);
        }
      });
}

void phy_system::update(entt::registry &registry, float dt) {
  float fixed_interval = 1.0f / sim_fps;
  float residual = cur_time - cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    residual -= fixed_interval;
    fixedupdate(registry, fixed_interval);
    cur_exec_fixed += 1;
  }
  cur_time += dt;
}

}; // namespace toolkit::sim