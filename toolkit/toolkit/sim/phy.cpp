#include "toolkit/opengl/editor.hpp"
#include "toolkit/sim/components/colliders.hpp"

#include "toolkit/opengl/draw.hpp"
#include "toolkit/sim/systems.hpp"

namespace toolkit::sim {

void phy_system::fixedupdate(entt::registry &registry, float dt) {
  auto sim_obj = registry.view<transform, rigid_sim_object>();
  std::vector<sim_obj_data> obj_data;
  std::map<entt::entity, int> entity_to_obj_data;
  sim_obj.each(
      [&](entt::entity entity, transform &trans, rigid_sim_object &sim_obj) {
        if (sim_obj.collider == nullptr)
          return;
        sim_obj_data entity_sim_data;
        entity_to_obj_data[entity] = obj_data.size();
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

  // broad phase collision detection
  broad_collision_pairs = get_broadphase_collision_pairs(registry, obj_data);

  narrow_collision_pairs.clear();
  for (int i = 0; i < broad_collision_pairs.size(); i++) {
    auto ep = broad_collision_pairs[i];
    auto sim_entity1 = obj_data[entity_to_obj_data[ep.first]];
    auto sim_entity2 = obj_data[entity_to_obj_data[ep.second]];
    if ((sim_entity1.sim_obj->fixed || !sim_entity1.sim_obj->active) &&
        (sim_entity2.sim_obj->fixed || !sim_entity2.sim_obj->active))
      continue;
    sim_entity1.sim_obj->update_collider_properties();
    sim_entity2.sim_obj->update_collider_properties();
    auto pair_contacts =
        colliders_get_contacts(sim_entity1.sim_obj->collider.get(),
                               sim_entity2.sim_obj->collider.get());
    for (int k = 0; k < pair_contacts.size(); k++)
      narrow_collision_pairs.push_back(pair_contacts[k]);
  }

  for (int i = 0; i < obj_data.size(); i++) {
    obj_data[i].sim_obj->world_position = obj_data[i].trans->world_pos();
    obj_data[i].sim_obj->world_rotation = obj_data[i].trans->world_rot();
  }

  // // ------------- start PBD based simulation -------------
  // {
  //   float step_dt = dt / num_sub_steps;
  //   for (int i = 0; i < num_sub_steps; i++) {
  //     // compute contact constraints in each sub step
  //     for (int j = 0; j < obj_data.size(); j++) {
  //       auto sim_obj = obj_data[j].sim_obj;
  //       // store previous position and rotation
  //       sim_obj->prev_world_position = obj_data[j].sim_obj->world_position;
  //       sim_obj->prev_world_rotation = obj_data[j].sim_obj->world_rotation;
  //       if (sim_obj->fixed || !(sim_obj->active))
  //         continue;
  //       // compute external force and torque
  //       math::vector3 ext_force = math::vector3::Zero();
  //       math::vector3 ext_torque = math::vector3::Zero();
  //       for (int k = 0; k < sim_obj->forces.size(); k++) {
  //         ext_force += sim_obj->forces[k].force;
  //         ext_torque +=
  //             (sim_obj->forces[k].position -
  //             sim_obj->mass_center_world_space)
  //                 .cross(sim_obj->forces[k].force);
  //       }
  //       // update object position and linear velocity given external force
  //       sim_obj->linear_velocity += step_dt * sim_obj->inverse_mass *
  //       ext_force; sim_obj->world_position += step_dt *
  //       sim_obj->linear_velocity;
  //       // update object rotation and angular velocity given external torque
  //       auto world_rot_matrix = sim_obj->world_rotation.toRotationMatrix();
  //       math::matrix3 inv_inertia_tensor =
  //           (world_rot_matrix * sim_obj->inverse_inertia_tensor) *
  //           (world_rot_matrix.transpose());
  //       math::matrix3 inertia_tensor =
  //           (world_rot_matrix * sim_obj->inertia_tensor) *
  //           (world_rot_matrix.transpose());
  //       sim_obj->angular_velocity =
  //           sim_obj->angular_velocity +
  //           step_dt * (inv_inertia_tensor *
  //                      (ext_torque -
  //                       sim_obj->angular_velocity.cross(
  //                           inertia_tensor * sim_obj->angular_velocity)));
  //       math::quat aux = math::quat(0.0f, sim_obj->angular_velocity.x(),
  //                                   sim_obj->angular_velocity.y(),
  //                                   sim_obj->angular_velocity.z());
  //       math::quat q = aux * sim_obj->world_rotation;
  //       sim_obj->world_rotation.x() =
  //           sim_obj->world_rotation.x() + step_dt * 0.5f * q.x();
  //       sim_obj->world_rotation.y() =
  //           sim_obj->world_rotation.y() + step_dt * 0.5f * q.y();
  //       sim_obj->world_rotation.z() =
  //           sim_obj->world_rotation.z() + step_dt * 0.5f * q.z();
  //       sim_obj->world_rotation.w() =
  //           sim_obj->world_rotation.w() + step_dt * 0.5f * q.w();
  //       sim_obj->world_rotation.normalize();
  //     }

  //     // check collision in each substep
  //     for (int k = 0; k < broad_collision_pairs.size(); k++) {
  //       auto ep = broad_collision_pairs[k];
  //       auto sim_entity1 = obj_data[entity_to_obj_data[ep.first]];
  //       auto sim_entity2 = obj_data[entity_to_obj_data[ep.second]];
  //       if ((sim_entity1.sim_obj->fixed || !sim_entity1.sim_obj->active) &&
  //           (sim_entity2.sim_obj->fixed || !sim_entity2.sim_obj->active))
  //         continue;
  //       sim_entity1.sim_obj->update_collider_properties();
  //       sim_entity2.sim_obj->update_collider_properties();

  //     }
  //   }
  // } // ------------- end PBD based simulation -------------

  // remove froces from all objects
  {
    sim_obj.each([&](entt::entity entity, transform &trans,
                     rigid_sim_object &sim_obj) { sim_obj.forces.clear(); });
  }
}

void phy_system::update_collider_properties(
    entt::registry &registry, std::vector<sim_obj_data> &obj_data) {
  for (auto &data : obj_data)
    data.sim_obj->update_collider_properties();
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
          if (debug_draw_collision_body) {
            if (sim_obj.collider->type == collider_type::SPHERE) {
              auto collider_ptr =
                  dynamic_cast<sphere_collider *>(sim_obj.collider.get());
              opengl::draw_sphere(collider_ptr->world_pos, cam_comp.vp,
                                  collider_ptr->radius, opengl::White, true);
            } else if (sim_obj.collider->type == collider_type::CAPSULE) {
              auto collider_ptr =
                  dynamic_cast<capsule_collider *>(sim_obj.collider.get());
              opengl::draw_capsule(
                  collider_ptr->world_pos -
                      collider_ptr->world_dir * collider_ptr->cap_distance * 0.5f,
                  collider_ptr->world_pos +
                      collider_ptr->world_dir * collider_ptr->cap_distance * 0.5f,
                  cam_comp.vp, opengl::White, true, collider_ptr->cap_radius,
                  collider_ptr->cap_radius);
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
          }
          if (debug_draw_bounding_spheres) {
            // draw bounding sphere of collider
            opengl::draw_sphere(sim_obj.bounding_sphere_center, cam_comp.vp,
                                sim_obj.bounding_sphere_radius, opengl::Green,
                                true);
            // draw mass center
            opengl::draw_sphere(sim_obj.mass_center_world_space, cam_comp.vp,
                                0.01f, opengl::Purple);
          }
        }
      });

  // render the collision pairs
  std::vector<math::vector3> contact_points;
  std::vector<std::pair<math::vector3, math::vector3>> contact_normals;
  for (int i = 0; i < narrow_collision_pairs.size(); i++) {
    contact_points.push_back(narrow_collision_pairs[i].contact_point1);
    contact_points.push_back(narrow_collision_pairs[i].contact_point2);
    contact_normals.push_back(
        std::make_pair(narrow_collision_pairs[i].contact_point1,
                       narrow_collision_pairs[i].contact_point1 +
                           narrow_collision_pairs[i].normal * 0.2f));
    contact_normals.push_back(
        std::make_pair(narrow_collision_pairs[i].contact_point2,
                       narrow_collision_pairs[i].contact_point2 -
                           narrow_collision_pairs[i].normal * 0.2f));
  }
  opengl::draw_quads(contact_points, cam_trans.local_right(),
                     cam_trans.local_up(), cam_comp.vp, 0.03f, opengl::Red);
  opengl::draw_lines(contact_normals, cam_comp.vp, opengl::Red);
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