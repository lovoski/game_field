#include "toolkit/opengl/rasterize/mixed.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/components/materials/all.hpp"
#include "toolkit/opengl/effects/ambient_occlusion.hpp"
#include "toolkit/opengl/rasterize/kernal.hpp"
#include "toolkit/opengl/rasterize/shaders.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::opengl {

struct _packed_vertex {
  math::vector4 position;
  math::vector4 normal;
  math::vector4 texcoords;
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
  ImGui::Checkbox("Draw Bounding Boxes", &draw_bounding_boxes);
  ImGui::Separator();

  ImGui::MenuItem("Ambient Occlusion", nullptr, nullptr, false);
  ImGui::Checkbox("Enable AO", &enable_ao_pass);
  ImGui::InputInt("Filter Size", &ao_filter_size);
  ImGui::DragFloat("Filter Sigma", &ao_filter_sigma, 0.01f, 0.0f, 10.0f);
  ImGui::DragFloat("SSAO Radius", &ssao_radius, 0.01f, 0.0f, 10.0f);
  ImGui::DragFloat("SSAO Noise Scale", &ssao_noise_scale, 0.01f, 0.0f, 1000.0f);
  ImGui::Separator();

  ImGui::MenuItem("Sun Light Settings");
  ImGui::Checkbox("Enable", &enable_sun);
  bool sun_parameter_modified = false;
  sun_parameter_modified |=
      ImGui::DragFloat("Sun Turbidity", &sun_turbidity, 0.1f, 0.0f, 100.0f);
  gui::color_edit_3("Light Color", sun_color);
  sun_parameter_modified |= ImGui::DragFloat("Sun Horizontal Angle", &sun_h,
                                             0.1f, -180.0f, 180.0f, "%.3f");
  sun_parameter_modified |=
      ImGui::DragFloat("Sun Vertical Angle", &sun_v, 0.1f, 0.0f, 90, "%.3f");
  ImGui::DragFloat("Sun Gamma", &ss_model.sun_gamma, 0.01f, 1.0f, 10.0f);
  if (sun_parameter_modified) {
    float sun_v_rad = sun_v / 180 * 3.1415927f;
    float sun_h_rad = sun_h / 180 * 3.1415927f;
    math::vector3 sun_dir(cos(sun_v_rad) * cos(sun_h_rad),
                          cos(sun_v_rad) * sin(sun_h_rad), sin(sun_v_rad));
    ss_model.update(sun_dir, sun_turbidity);
  }
}

void defered_forward_mixed::draw_gui(entt::registry &registry,
                                     entt::entity entity) {
  if (auto ptr = registry.try_get<camera>(entity)) {
    if (ImGui::CollapsingHeader("Camera"))
      ptr->draw_gui(nullptr);
  }
  if (auto ptr = registry.try_get<mesh_data>(entity)) {
    if (ImGui::CollapsingHeader("Mesh Data"))
      ptr->draw_gui(nullptr); // TODO: no need for mesh data component to access
                              // app settings
  }
  if (has_any_materials(registry, entity)) {
    if (ImGui::CollapsingHeader("Materials")) {
      for (auto &func : material::__material_draw_gui__) {
        func.second(registry, entity);
      }
    }
  }
}

void defered_forward_mixed::init0(entt::registry &registry) {
  pos_tex.create(GL_TEXTURE_2D);
  normal_tex.create(GL_TEXTURE_2D);
  gbuffer_depth_tex.create(GL_TEXTURE_2D);
  color_tex.create(GL_TEXTURE_2D);
  mask_tex.create(GL_TEXTURE_2D);
  ao_color.create(GL_TEXTURE_2D);

  gbuffer_geometry_pass.compile_shader_from_source(gbuffer_geometry_pass_vs,
                                                   gbuffer_geometry_pass_fs);
  defered_phong_pass.compile_shader_from_source(defered_phong_pass_vs,
                                                defered_phong_pass_fs);

  collect_scene_vertex_buffer_program.create(str_format(
      collect_scene_vertex_buffer_program_source.c_str(), work_group_size));
  collect_scene_index_buffer_program.create(str_format(
      collect_scene_index_buffer_program_source.c_str(), work_group_size));
  scene_buffer_apply_blendshape_program.create(str_format(
      scene_buffer_apply_blendshape_program_source.c_str(), work_group_size));
  scene_buffer_apply_mesh_skinning_program.create(
      str_format(scene_buffer_apply_mesh_skinning_program_source.c_str(),
                 work_group_size));
  scene_vertex_buffer.create();
  scene_index_buffer.create();
  scene_vao.create();

  skeleton_matrices_buffer.create();

  gbuffer.create();
  cbuffer.create();
  msaa_buffer.create();
  ao_buffer.create();
  resize(g_instance.scene_width, g_instance.scene_height);

  light_data_buffer.create();

  float sun_v_rad = sun_v / 180 * 3.1415927f;
  float sun_h_rad = sun_h / 180 * 3.1415927f;
  math::vector3 sun_dir(cos(sun_v_rad) * cos(sun_h_rad),
                        cos(sun_v_rad) * sin(sun_h_rad), sin(sun_v_rad));
  ss_model.update(sun_dir, sun_turbidity);

  // initialize materials
  for (auto &initialier : material::__material_constructors__)
    initialier.second(registry);
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
  mask_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
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
  cbuffer.attach_color_buffer(color_tex, GL_COLOR_ATTACHMENT0);
  cbuffer.end_draw_buffers();
  if (!cbuffer.check_status())
    spdlog::error("cbuffer not complete!");
  cbuffer.unbind();

  ao_buffer.bind();
  ao_buffer.begin_draw_buffers();
  ao_color.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  ao_color.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                           {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                           {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                           {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  ao_buffer.attach_color_buffer(ao_color, GL_COLOR_ATTACHMENT0);
  ao_buffer.end_draw_buffers();
  if (!ao_buffer.check_status())
    spdlog::error("ao buffer not complete!");
  ao_buffer.unbind();

  msaa_buffer.bind();
  glGenRenderbuffers(1, &msaa_color_buffer);
  glBindRenderbuffer(GL_RENDERBUFFER, msaa_color_buffer);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa_samples, GL_RGB, width,
                                   height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, msaa_color_buffer);

  glGenRenderbuffers(1, &msaa_depth_buffer);
  glBindRenderbuffer(GL_RENDERBUFFER, msaa_depth_buffer);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa_samples,
                                   GL_DEPTH24_STENCIL8, width, height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER, msaa_depth_buffer);

  if (!msaa_buffer.check_status()) {
    spdlog::error("msaa buffer not complete!");
  }
  msaa_buffer.unbind();
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

  bool any_force_update_flag = false;
  int64_t current_scene_vertex_counter = 0, current_scene_index_counter = 0;
  mesh_data_entities.each(
      [&](entt::entity entity, transform &trans, mesh_data &data) {
        any_force_update_flag |= data.force_update_flag;
        data.force_update_flag = false;
        current_scene_vertex_counter += data.vertices.size();
        current_scene_index_counter += data.indices.size();
      });

  bool scene_vtx_count_mismatch =
      (current_scene_vertex_counter != scene_vertex_counter);
  bool scene_idx_count_mismatch =
      (current_scene_index_counter != scene_index_counter);
  bool scene_mesh_mismatch =
      scene_mesh_counter != mesh_data_entities.size_hint();
  if (scene_vtx_count_mismatch || scene_idx_count_mismatch ||
      scene_mesh_mismatch || any_force_update_flag) {
    spdlog::info("Detect change in scnene vertex count, scene index count, "
                 "scene mesh count or force update flag, "
                 "resize scene vertex buffer and scene index buffer.");
    scene_mesh_counter = mesh_data_entities.size_hint();
    scene_vertex_counter = current_scene_vertex_counter;
    scene_index_counter = current_scene_index_counter;
    // create new scene vertex and index buffer
    scene_vertex_buffer.set_data_ssbo(
        sizeof(_packed_vertex) * scene_vertex_counter, GL_DYNAMIC_DRAW);
    scene_index_buffer.set_data_ssbo(sizeof(unsigned int) * scene_index_counter,
                                     GL_DYNAMIC_DRAW);

    collect_scene_vertex_buffer_program.use();
    int64_t current_vertex_offset = 0, current_index_offset = 0;
    int base_instance_idx = 0;
    mesh_data_entities.each([&](entt::entity entity, transform &trans,
                                mesh_data &data) {
      // update the scene buffer content if necessary (the offset has changed or
      // it's the first mesh)
      if ((data.scene_vertex_offset != current_vertex_offset) ||
          (data.scene_index_offset != current_index_offset) ||
          (data.scene_vertex_offset == 0) || (data.scene_index_offset == 0)) {
        data.scene_vertex_offset = current_vertex_offset;
        data.scene_index_offset = current_index_offset;
        // dispatch collect scene buffer program
        collect_scene_vertex_buffer_program
            .bind_buffer(data.vertex_buffer.get_handle(), 0)
            .bind_buffer(scene_vertex_buffer.get_handle(), 1);
        collect_scene_vertex_buffer_program.set_int("gActualSize",
                                                    data.vertices.size());
        collect_scene_vertex_buffer_program.set_int("gVertexOffset",
                                                    data.scene_vertex_offset);
        collect_scene_vertex_buffer_program.dispatch(
            (data.vertices.size() + work_group_size - 1) / work_group_size, 1,
            1);
        collect_scene_vertex_buffer_program.barrier();
      }

      current_vertex_offset += data.vertices.size();
      current_index_offset += data.indices.size();
    });

    collect_scene_index_buffer_program.use();
    mesh_data_entities.each([&](entt::entity entity, transform &trans,
                                mesh_data &data) {
      collect_scene_index_buffer_program
          .bind_buffer(data.index_buffer.get_handle(), 0)
          .bind_buffer(scene_index_buffer.get_handle(), 1);
      collect_scene_index_buffer_program.set_int("gActualSize",
                                                 data.indices.size());
      collect_scene_index_buffer_program.set_int("gIndexOffset",
                                                 data.scene_index_offset);
      collect_scene_index_buffer_program.set_int("gVertexOffset",
                                                 data.scene_vertex_offset);
      collect_scene_index_buffer_program.dispatch(
          (data.indices.size() + work_group_size - 1) / work_group_size, 1, 1);
      collect_scene_index_buffer_program.barrier(GL_ELEMENT_ARRAY_BARRIER_BIT |
                                                 GL_SHADER_STORAGE_BARRIER_BIT);
    });
  }

  // check for blend shapes and skinned meshes
  std::set<entt::entity> mesh_with_active_bs;
  collect_scene_vertex_buffer_program.use();
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
      collect_scene_vertex_buffer_program
          .bind_buffer(data.vertex_buffer.get_handle(), 0)
          .bind_buffer(scene_vertex_buffer.get_handle(), 1);
      collect_scene_vertex_buffer_program.set_int("gActualSize",
                                                  data.vertices.size());
      collect_scene_vertex_buffer_program.set_int("gVertexOffset",
                                                  data.scene_vertex_offset);
      collect_scene_vertex_buffer_program.dispatch(
          (data.vertices.size() + work_group_size - 1) / work_group_size, 1, 1);
      collect_scene_vertex_buffer_program.barrier(
          GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_ELEMENT_ARRAY_BARRIER_BIT |
          GL_COMMAND_BARRIER_BIT);
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
            (data.vertices.size() + work_group_size - 1) / work_group_size, 1,
            1);
        scene_buffer_apply_blendshape_program.barrier();
      }
    }
  }
  // perform skinned mesh deform latter
  scene_buffer_apply_mesh_skinning_program.use();
  registry.view<entt::entity, skinned_mesh_bundle>().each(
      [&](entt::entity entity, skinned_mesh_bundle &bundle) {
        if (bundle.mesh_entities.size() == 0 ||
            bundle.bone_entities.size() == 0)
          return;
        std::vector<_bone_matrix_block> bone_matrices(
            bundle.bone_entities.size());
        for (int i = 0; i < bundle.bone_entities.size(); i++) {
          // mark invalid bones as null entity, we can't remove it since the
          // bone id in each vertex should remain static.
          if (!registry.valid(bundle.bone_entities[i])) {
            bundle.bone_entities[i] = entt::null;
            continue;
          }
          bone_matrices[i].model_mat =
              registry.get<transform>(bundle.bone_entities[i]).matrix();
          bone_matrices[i].offset_mat =
              registry.get<anim::bone_node>(bundle.bone_entities[i])
                  .offset_matrix;
        }
        skeleton_matrices_buffer.set_data_ssbo(bone_matrices, GL_DYNAMIC_DRAW);

        // check if there are any invalid mesh entities, if so, remove them from
        // the list.
        int valid_counter = 0;
        for (int k = 0; k < bundle.mesh_entities.size(); k++) {
          if (registry.valid(bundle.mesh_entities[k]))
            valid_counter++;
        }
        if (valid_counter != bundle.mesh_entities.size()) {
          std::vector<entt::entity> valid_mesh_entities;
          for (int k = 0; k < bundle.mesh_entities.size(); k++) {
            if (registry.valid(bundle.mesh_entities[k]))
              valid_mesh_entities.push_back(bundle.mesh_entities[k]);
          }
          bundle.mesh_entities = valid_mesh_entities;
        }
        for (int k = 0; k < bundle.mesh_entities.size(); k++) {
          auto &data =
              mesh_data_entities.get<mesh_data>(bundle.mesh_entities[k]);
          data.skinned = true;
          scene_buffer_apply_mesh_skinning_program
              .bind_buffer(data.vertex_buffer.get_handle(), 0)
              .bind_buffer(skeleton_matrices_buffer.get_handle(), 1)
              .bind_buffer(scene_vertex_buffer.get_handle(), 2);
          scene_buffer_apply_mesh_skinning_program.set_int(
              "gActualSize", data.vertices.size());
          scene_buffer_apply_mesh_skinning_program.set_int(
              "gVertexOffset", data.scene_vertex_offset);
          scene_buffer_apply_mesh_skinning_program.set_bool(
              "gBlended",
              mesh_with_active_bs.count(bundle.mesh_entities[k]) > 0);
          scene_buffer_apply_mesh_skinning_program.dispatch(
              (data.vertices.size() + work_group_size - 1) / work_group_size, 1,
              1);
          scene_buffer_apply_mesh_skinning_program.barrier();
        }
      });

  // bind new scene vertex buffer and index buffer to scene vao
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
}

void defered_forward_mixed::update_scene_lights(entt::registry &registry) {
  std::vector<light_data_pacakge> lights;
  if (enable_sun) {
    light_data_pacakge package;
    package.idata[0] = 0;
    package.color << sun_color, 1.0f;
    float sun_v_rad = sun_v / 180 * 3.1415927f;
    float sun_h_rad = sun_h / 180 * 3.1415927f;
    math::vector3 sun_dir(cos(sun_v_rad) * sin(sun_h_rad), sin(sun_v_rad),
                          cos(sun_v_rad) * cos(sun_h_rad));
    package.fdata0 << -sun_dir, 0.0f;
    lights.emplace_back(package);
  }
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

    // update visible cache for main camera only
    auto trans_mesh_view = registry.view<entt::entity, transform, mesh_data>();
    trans_mesh_view.each(
        [&](entt::entity entity, transform &trans, mesh_data &data) {

        });

    // ------------------ render to geometry framebuffer ------------------
    gbuffer.bind();
    gbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    if (scene_mesh_counter > 0) {
      scene_vao.bind();
      gbuffer_geometry_pass.use();
      gbuffer_geometry_pass.set_mat4("gVP", cam_comp.vp);
      gbuffer_geometry_pass.set_mat4("gproj", cam_comp.proj);
      trans_mesh_view.each([&](entt::entity entity, transform &trans,
                               mesh_data &data) {
        gbuffer_geometry_pass.set_mat4("gModel", data.skinned
                                                     ? math::matrix4::Identity()
                                                     : trans.matrix());
        glDrawElements(GL_TRIANGLES, data.indices.size(), GL_UNSIGNED_INT,
                       (void *)(data.scene_index_offset * sizeof(GLuint)));
      });
      scene_vao.unbind();
    }
    gbuffer.unbind();

    // render ao buffer if needed
    if (enable_ao_pass) {
      ao_buffer.bind();
      ao_buffer.set_viewport(0, 0, g_instance.scene_width,
                             g_instance.scene_height);
      glClear(GL_COLOR_BUFFER_BIT);
      glClearColor(0, 0, 0, 1);
      ssao(pos_tex, normal_tex, mask_tex, cam_comp.view, cam_comp.proj,
           ssao_noise_scale, ssao_radius);
      ao_buffer.unbind();
    }

    // ------------------- render to multisample framebuffer -------------------
    msaa_buffer.bind();
    msaa_buffer.set_viewport(0, 0, g_instance.scene_width,
                             g_instance.scene_height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    glDisable(GL_DEPTH_TEST);
    ss_model.render(cam_comp.vp, cam_trans.position());
    glEnable(GL_DEPTH_TEST);

    // iterate through all material types
    scene_vao.bind();
    for (auto &mat_shader_pair : material::__material_shaders__) {
      auto mat_name = mat_shader_pair.first;
      auto &mat_shader = mat_shader_pair.second;
      material::__material_instance__[mat_name]->prepare0();
      mat_shader.use();
      // bind common bindings
      mat_shader.set_mat4("gViewMat", cam_comp.view);
      mat_shader.set_mat4("gProjMat", cam_comp.proj);
      mat_shader.set_vec3("gViewDir", -cam_trans.local_forward());
      mat_shader.set_vec2("gViewport", g_instance.get_scene_size());
      mat_shader.set_buffer_ssbo(light_data_buffer, 0);
      material::__material_view__[mat_name](registry, [&](entt::entity entity,
                                                          material *mat) {
        if (auto mesh_ptr = registry.try_get<mesh_data>(entity)) {
          if (mesh_ptr->should_render_mesh) {
            auto &trans = registry.get<transform>(entity);
            mat_shader.set_int("gVertexOffset", mesh_ptr->scene_vertex_offset);
            mat_shader.set_mat4("gModelToWorldPoint",
                                mesh_ptr->skinned ? math::matrix4::Identity()
                                                  : trans.matrix());
            mat_shader.set_mat3(
                "gModelToWorldDir",
                mesh_ptr->skinned
                    ? (math::matrix3)(math::matrix3::Identity())
                    : (math::matrix3)(trans.matrix().block<3, 3>(0, 0)));
            mat->bind_uniforms(mat_shader);
            glDrawElements(
                GL_TRIANGLES, mesh_ptr->indices.size(), GL_UNSIGNED_INT,
                (void *)(mesh_ptr->scene_index_offset * sizeof(GLuint)));
          }
        }
      });
      material::__material_instance__[mat_name]->prepare1();
    }
    scene_vao.unbind();

    // render grids
    if (should_draw_grid) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBlendEquation(GL_FUNC_ADD);
      draw_infinite_grid(cam_comp.view, cam_comp.proj, cam_comp.z_near,
                         cam_comp.z_far, grid_spacing);
      glDisable(GL_BLEND);
    }

    // render debug ui from scripts
    if (should_draw_debug) {
      glDisable(GL_DEPTH_TEST);
      // debug rendering
      if (auto app_ptr = registry.ctx().get<iapp *>()) {
        auto script_sys = app_ptr->get_sys<script_system>();
        script_sys->draw_to_scene(app_ptr);
      }
      glEnable(GL_DEPTH_TEST);
    }

    // copy color texture from multisample framebuffer to color framebuffer for
    // presentation
    glBindFramebuffer(GL_READ_FRAMEBUFFER, msaa_buffer.get_handle());
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cbuffer.get_handle());
    glBlitFramebuffer(0, 0, g_instance.scene_width, g_instance.scene_height, 0,
                      0, g_instance.scene_width, g_instance.scene_height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    msaa_buffer.unbind();

    // apply post processing with alpha blending
    cbuffer.bind();
    cbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    // ambient occlusion
    if (enable_ao_pass) {
      ao_gaussian_filter(ao_color, ao_filter_size, ao_filter_sigma);
    }

    glDisable(GL_BLEND);
    cbuffer.unbind();
  }
}

}; // namespace toolkit::opengl