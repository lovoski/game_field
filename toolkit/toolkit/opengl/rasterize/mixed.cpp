#include "toolkit/opengl/rasterize/mixed.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/rasterize/kernal.hpp"
#include "toolkit/opengl/rasterize/shaders.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::opengl {

struct _packed_vertex {
  math::vector4 position;
  math::vector4 normal;
};

struct _bone_matrix_block {
  math::matrix4 model_mat;
  math::matrix4 offset_mat;
};

void defered_forward_mixed::draw_menu_gui() {
  ImGui::MenuItem("Grid", nullptr, nullptr, false);
  ImGui::Checkbox("Show Grid", &should_draw_grid);
  ImGui::InputInt("Grid Spacing", &grid_spacing);
  ImGui::Separator();

  ImGui::MenuItem("Debug", nullptr, nullptr, false);
  ImGui::Checkbox("Draw Debug", &should_draw_debug);
}

void defered_forward_mixed::draw_gui(entt::registry &registry,
                                     entt::entity entity) {
  if (auto ptr = registry.try_get<camera>(entity)) {
    if (ImGui::CollapsingHeader("Camera"))
      ptr->draw_gui(nullptr);
  }
  if (auto ptr = registry.try_get<dir_light>(entity)) {
    if (ImGui::CollapsingHeader("Dir Light"))
      ptr->draw_gui(nullptr);
  }
  if (auto ptr = registry.try_get<point_light>(entity)) {
    if (ImGui::CollapsingHeader("Point Light"))
      ptr->draw_gui(nullptr);
  }
  if (auto ptr = registry.try_get<mesh_data>(entity)) {
    if (ImGui::CollapsingHeader("Mesh Data"))
      ptr->draw_gui(nullptr); // TODO: no need for mesh data component to access
                              // app settings
  }
}

void defered_forward_mixed::init0(entt::registry &registry) {
  pos_tex.create(GL_TEXTURE_2D);
  normal_tex.create(GL_TEXTURE_2D);
  gbuffer_depth_tex.create(GL_TEXTURE_2D);
  color_tex.create(GL_TEXTURE_2D);
  mask_tex.create(GL_TEXTURE_2D);
  color_buffer_depth_tex.create(GL_TEXTURE_2D);

  gbuffer_geometry_pass.compile_shader_from_source(gbuffer_geometry_pass_vs,
                                                   gbuffer_geometry_pass_fs);
  defered_phong_pass.compile_shader_from_source(defered_phong_pass_vs,
                                                defered_phong_pass_fs);

  collect_scene_buffer_program.create(
      str_format(collect_scene_buffer_program_source.c_str(),
                 scene_buffer_program_workgroup_size));
  collect_scene_index_buffer_program.create(
      str_format(collect_scene_index_buffer_program_source.c_str(),
                 scene_buffer_program_workgroup_size));
  scene_buffer_apply_blendshape_program.create(
      str_format(scene_buffer_apply_blendshape_program_source.c_str(),
                 scene_buffer_program_workgroup_size));
  scene_buffer_apply_mesh_skinning_program.create(
      str_format(scene_buffer_apply_mesh_skinning_program_source.c_str(),
                 scene_buffer_program_workgroup_size));
  scene_vertex_buffer.create();
  scene_index_buffer.create();
  scene_vao.create();
  scene_model_matrices_buffer.create();
  scene_indirect_draw_commands_buffer.create();
  draw_indirect_buffer.create();

  skeleton_matrices_buffer.create();

  scene_vao.bind();
  scene_vertex_buffer.bind_as(GL_ARRAY_BUFFER);
  scene_index_buffer.bind_as(GL_ELEMENT_ARRAY_BUFFER);
  scene_vao.link_attribute(scene_vertex_buffer, 0, 4, GL_FLOAT,
                           sizeof(_packed_vertex), (void *)0);
  scene_vao.link_attribute(scene_vertex_buffer, 1, 4, GL_FLOAT,
                           sizeof(_packed_vertex), (void *)(4 * sizeof(float)));
  scene_vao.unbind();
  scene_vertex_buffer.unbind_as(GL_ARRAY_BUFFER);
  scene_index_buffer.unbind_as(GL_ELEMENT_ARRAY_BUFFER);

  gbuffer.create();
  cbuffer.create();
  resize(g_instance.scene_width, g_instance.scene_height);

  light_data_buffer.create();
}

void defered_forward_mixed::init1(entt::registry &registry) {}

void defered_forward_mixed::resize(int width, int height) {
  gbuffer.bind();
  gbuffer.begin_draw_buffers();
  pos_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  pos_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                          {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                          {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                          {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  normal_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  normal_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                             {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                             {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                             {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  mask_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  mask_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                           {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                           {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                           {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  gbuffer_depth_tex.set_data(width, height, GL_DEPTH_COMPONENT24,
                             GL_DEPTH_COMPONENT, GL_FLOAT);
  gbuffer_depth_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                                    {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                                    {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                    {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  gbuffer.attach_color_buffer(pos_tex, GL_COLOR_ATTACHMENT0);
  gbuffer.attach_color_buffer(normal_tex, GL_COLOR_ATTACHMENT1);
  gbuffer.attach_color_buffer(mask_tex, GL_COLOR_ATTACHMENT2);
  gbuffer.end_draw_buffers();
  gbuffer.attach_depth_buffer(gbuffer_depth_tex);
  if (!gbuffer.check_status())
    spdlog::error("gbuffer not complete!");
  gbuffer.unbind();

  cbuffer.bind();
  cbuffer.begin_draw_buffers();
  color_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  color_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                            {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  color_buffer_depth_tex.set_data(width, height, GL_DEPTH_COMPONENT24,
                                  GL_DEPTH_COMPONENT, GL_FLOAT);
  color_buffer_depth_tex.set_parameters(
      {{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
       {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
       {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
       {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  cbuffer.attach_color_buffer(color_tex, GL_COLOR_ATTACHMENT0);
  cbuffer.end_draw_buffers();
  cbuffer.attach_depth_buffer(color_buffer_depth_tex);
  if (!cbuffer.check_status())
    spdlog::error("cbuffer not complete!");
  cbuffer.unbind();
}

void defered_forward_mixed::preupdate(entt::registry &registry, float dt) {
  // update vp matrices for cameras
  registry.view<camera>().each([&](entt::entity entity, camera &camera) {
    compute_vp_matrix(registry, entity, g_instance.scene_width,
                      g_instance.scene_height);
  });
}

void defered_forward_mixed::update_scene_buffers(entt::registry &registry) {
  auto mesh_data_entities = registry.view<entt::entity, transform, mesh_data>();

  int64_t current_scene_vertex_counter = 0, current_scene_index_counter = 0;
  std::vector<math::matrix4> scene_model_matrices;
  mesh_data_entities.each(
      [&](entt::entity entity, transform &trans, mesh_data &data) {
        scene_model_matrices.push_back(data.actor_entity == entt::null
                                           ? trans.matrix()
                                           : math::matrix4::Identity());
        current_scene_vertex_counter += data.vertices.size();
        current_scene_index_counter += data.indices.size();
      });
  scene_model_matrices_buffer.set_data_ssbo(scene_model_matrices,
                                            GL_DYNAMIC_DRAW);

  if (current_scene_vertex_counter != scene_vertex_counter) {
    indirect_commands.clear();
    scene_vertex_counter = current_scene_vertex_counter;
    scene_index_counter = current_scene_index_counter;
    // create new scene vertex and index buffer
    scene_vertex_buffer.set_data_ssbo(
        sizeof(_packed_vertex) * scene_vertex_counter, GL_DYNAMIC_DRAW);
    scene_index_buffer.set_data_ssbo(
        sizeof(unsigned int) * current_scene_index_counter, GL_DYNAMIC_DRAW);

    collect_scene_buffer_program.use();
    int64_t current_vertex_offset = 0, current_index_offset = 0;
    mesh_data_entities.each([&](entt::entity entity, transform &trans,
                                mesh_data &data) {
      if ((data.scene_vertex_offset != current_vertex_offset) ||
          (data.scene_index_offset != current_index_offset)) {
        data.scene_vertex_offset = current_vertex_offset;
        data.scene_index_offset = current_index_offset;

        // dispatch collect scene buffer program
        collect_scene_buffer_program
            .bind_buffer(data.vertex_buffer.get_handle(), 0)
            .bind_buffer(scene_vertex_buffer.get_handle(), 1);
        collect_scene_buffer_program.set_int("gActualSize",
                                             data.vertices.size());
        collect_scene_buffer_program.set_int("gVertexOffset",
                                             data.scene_vertex_offset);
        collect_scene_buffer_program.dispatch(
            (data.vertices.size() + scene_buffer_program_workgroup_size - 1) /
                scene_buffer_program_workgroup_size,
            1, 1);
        collect_scene_buffer_program.barrier(
            GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_ELEMENT_ARRAY_BARRIER_BIT |
            GL_COMMAND_BARRIER_BIT);
      }
      current_vertex_offset += data.vertices.size();
      current_index_offset += data.indices.size();

      draw_elements_indirect_command indirect_command;
      indirect_command.num_indices = data.indices.size();
      indirect_command.num_instances = 1;
      indirect_command.index_offset = data.scene_index_offset;
      indirect_command.vertex_offset = data.scene_vertex_offset;
      indirect_command.base_instance = 0;
      indirect_commands.emplace_back(indirect_command);
    });

    draw_indirect_buffer.set_data_as(GL_DRAW_INDIRECT_BUFFER, indirect_commands,
                                     GL_STATIC_DRAW);

    collect_scene_index_buffer_program.use();
    mesh_data_entities.each(
        [&](entt::entity entity, transform &trans, mesh_data &data) {
          collect_scene_index_buffer_program
              .bind_buffer(data.index_buffer.get_handle(), 0)
              .bind_buffer(scene_index_buffer.get_handle(), 1);
          collect_scene_index_buffer_program.set_int("gActualSize",
                                                     data.indices.size());
          collect_scene_index_buffer_program.set_int("gIndexOffset",
                                                     data.scene_index_offset);
          collect_scene_index_buffer_program.set_int("gVertexOffset", 0);
          collect_scene_index_buffer_program.dispatch(
              (data.indices.size() + scene_buffer_program_workgroup_size - 1) /
                  scene_buffer_program_workgroup_size,
              1, 1);
          collect_scene_index_buffer_program.barrier(
              GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT |
              GL_ELEMENT_ARRAY_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
        });
  }

  // check for blend shapes and skinned meshes
  std::set<entt::entity> mesh_with_active_bs;
  std::map<entt::entity, std::vector<entt::entity>> mesh_with_skeleton;
  collect_scene_buffer_program.use();
  mesh_data_entities.each([&](entt::entity entity, transform &trans,
                              mesh_data &data) {
    int bs_count = 0;
    for (auto &bs : data.blend_shapes) {
      if (bs.weight != 0.0f) {
        bs_count++;
      }
    }
    if (bs_count > 0) {
      mesh_with_active_bs.insert(entity);
      // reset default position and normal for mesh with blend shapes
      collect_scene_buffer_program
          .bind_buffer(data.vertex_buffer.get_handle(), 0)
          .bind_buffer(scene_vertex_buffer.get_handle(), 1);
      collect_scene_buffer_program.set_int("gActualSize", data.vertices.size());
      collect_scene_buffer_program.set_int("gVertexOffset",
                                           data.scene_vertex_offset);
      collect_scene_buffer_program.dispatch(
          (data.vertices.size() + scene_buffer_program_workgroup_size - 1) /
              scene_buffer_program_workgroup_size,
          1, 1);
      collect_scene_buffer_program.barrier(GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT |
                                           GL_ELEMENT_ARRAY_BARRIER_BIT |
                                           GL_COMMAND_BARRIER_BIT);
    }
    if (data.actor_entity != entt::null) {
      mesh_with_skeleton[data.actor_entity].push_back(entity);
    }
  });
  // perform blend shape deform first
  if (mesh_with_active_bs.size() > 0) {
    scene_buffer_apply_blendshape_program.use();
    for (auto ent : mesh_with_active_bs) {
      auto &data = mesh_data_entities.get<mesh_data>(ent);

      for (int i = 0; i < data.blend_shapes.size(); i++) {
        if (data.blend_shapes[i].weight == 0.0f) {
          continue; // skip inactive blend shapes
        }
        scene_buffer_apply_blendshape_program
            .bind_buffer(data.blendshape_targets[i].get_handle(), 0)
            .bind_buffer(scene_vertex_buffer.get_handle(), 1);
        scene_buffer_apply_blendshape_program.set_int("gActualSize",
                                                      data.vertices.size());
        scene_buffer_apply_blendshape_program.set_int("gVertexOffset",
                                                      data.scene_vertex_offset);
        scene_buffer_apply_blendshape_program.set_float(
            "gWeightValue", data.blend_shapes[i].weight);
        scene_buffer_apply_blendshape_program.dispatch(
            (data.vertices.size() + scene_buffer_program_workgroup_size - 1) /
                scene_buffer_program_workgroup_size,
            1, 1);
        scene_buffer_apply_blendshape_program.barrier();
      }
    }
  }
  // perform skinned mesh deform latter
  if (mesh_with_skeleton.size() > 0) {
    scene_buffer_apply_mesh_skinning_program.use();
    std::vector<_bone_matrix_block> bone_matrices;
    for (auto p : mesh_with_skeleton) {
      auto &actor_comp = registry.get<anim::actor>(p.first);

      bone_matrices.resize(actor_comp.ordered_entities.size());
      for (int i = 0; i < actor_comp.ordered_entities.size(); i++) {
        auto &joint_trans =
            registry.get<transform>(actor_comp.ordered_entities[i]);
        bone_matrices[i].model_mat = joint_trans.matrix();
        bone_matrices[i].offset_mat = actor_comp.skel.offset_matrices[i];
      }
      skeleton_matrices_buffer.set_data_ssbo(bone_matrices, GL_DYNAMIC_DRAW);

      for (auto ent : p.second) {
        auto &data = mesh_data_entities.get<mesh_data>(ent);
        scene_buffer_apply_mesh_skinning_program
            .bind_buffer(data.vertex_buffer.get_handle(), 0)
            .bind_buffer(skeleton_matrices_buffer.get_handle(), 1)
            .bind_buffer(scene_vertex_buffer.get_handle(), 2);
        scene_buffer_apply_mesh_skinning_program.set_int("gActualSize",
                                                         data.vertices.size());
        scene_buffer_apply_mesh_skinning_program.set_int(
            "gVertexOffset", data.scene_vertex_offset);
        scene_buffer_apply_mesh_skinning_program.set_bool(
            "gBlended", mesh_with_active_bs.count(ent) > 0);
        scene_buffer_apply_mesh_skinning_program.dispatch(
            (data.vertices.size() + scene_buffer_program_workgroup_size - 1) /
                scene_buffer_program_workgroup_size,
            1, 1);
        scene_buffer_apply_mesh_skinning_program.barrier();
      }
    }
  }
}

void defered_forward_mixed::update_obj_bbox(entt::registry &registry) {}

void defered_forward_mixed::update_scene_lights(entt::registry &registry) {
  std::vector<light_data_pacakge> lights;
  registry.view<dir_light, transform>().each(
      [&](entt::entity entity, dir_light &light, transform &trans) {
        light_data_pacakge package;
        package.idata[0] = 0;
        package.pos << trans.position(), 1.0f;
        package.color << light.color, 1.0f;
        package.fdata0 << trans.local_forward(), 0.0f;
        lights.emplace_back(package);
      });
  registry.view<point_light, transform>().each(
      [&](entt::entity entity, point_light &light, transform &trans) {
        light_data_pacakge package;
        package.idata[0] = 1;
        package.pos << trans.position(), 1.0f;
        package.color << light.color, 1.0f;
        lights.emplace_back(package);
      });
  light_data_buffer.set_data_ssbo(lights);
}

void defered_forward_mixed::render(entt::registry &registry) {
  if (auto cam_ptr = registry.try_get<camera>(g_instance.active_camera)) {
    auto &cam_trans = registry.get<transform>(g_instance.active_camera);
    auto &cam_comp = *cam_ptr;

    update_scene_lights(registry);

    gbuffer.bind();
    gbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    // render gbuffer
    if (scene_vertex_counter > 0) {
      scene_vao.bind();
      gbuffer_geometry_pass.use();
      gbuffer_geometry_pass.set_mat4("gVP", cam_comp.vp);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
                       scene_model_matrices_buffer.get_handle());
      draw_indirect_buffer.bind_as(GL_DRAW_INDIRECT_BUFFER);
      glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_INT, (GLvoid *)0,
                                  indirect_commands.size(), 0);
      // glDrawElements(GL_TRIANGLES, scene_index_counter, GL_UNSIGNED_INT, 0);
      scene_vao.unbind();
    }

    gbuffer.unbind();

    // render color buffer
    cbuffer.bind();
    cbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    if (should_draw_debug) {
      // debug rendering
      if (auto app_ptr = registry.ctx().get<iapp *>()) {
        auto script_sys = app_ptr->get_sys<script_system>();
        script_sys->draw_to_scene(app_ptr);
      }
    }

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    defered_phong_pass.use();
    defered_phong_pass.set_vec3("gViewDir", -cam_trans.local_forward());
    defered_phong_pass.set_texture2d("gPosTex", pos_tex.get_handle(), 0);
    defered_phong_pass.set_texture2d("gNormalTex", normal_tex.get_handle(), 1);
    defered_phong_pass.set_texture2d("gGbufferDepthTex",
                                     gbuffer_depth_tex.get_handle(), 2);
    defered_phong_pass.set_texture2d("gCbufferDepthTex",
                                     color_buffer_depth_tex.get_handle(), 3);
    defered_phong_pass.set_texture2d("gMaskTex", mask_tex.get_handle(), 4);
    defered_phong_pass.set_buffer_ssbo(light_data_buffer, 0);
    quad_draw_call();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    if (should_draw_grid)
      draw_infinite_grid(cam_comp.view, cam_comp.proj, grid_spacing);

    glDisable(GL_BLEND);

    cbuffer.unbind();
  }
}

}; // namespace toolkit::opengl