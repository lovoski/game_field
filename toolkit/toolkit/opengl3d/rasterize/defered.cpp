#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/effects/ambient_occlusion.hpp"
#include "toolkit/opengl3d/gui.hpp"
#include "toolkit/opengl3d/rasterize/kernal.hpp"
#include "toolkit/opengl3d/rasterize/shaders.hpp"
#include "toolkit/opengl3d/rasterize/system.hpp"

#include "toolkit/opengl3d/native_subsys.hpp"

namespace toolkit::opengl3d {

struct _packed_vertex {
  math::vector4 position;
  math::vector4 normal;
  math::vector4 texcoords;
};

struct _bone_matrix_block {
  math::matrix4 model_mat;
  math::matrix4 offset_mat;
};

void defered_render_system::draw_menu_gui() {
  ImGui::MenuItem("Grid", nullptr, nullptr, false);
  ImGui::Checkbox("Show Grid", &should_draw_grid);
  ImGui::InputInt("Grid Spacing", &grid_spacing);
  ImGui::Separator();

  ImGui::MenuItem("Background", nullptr, nullptr, false);
  ImGui::Checkbox("Use Pure Color", &use_pure_color_bg);
  color_edit_3("Pure Color", bg_color);
  ImGui::Separator();

  ImGui::MenuItem("Debug", nullptr, nullptr, false);
  ImGui::Checkbox("Draw Debug", &should_draw_debug);
  ImGui::Checkbox("Draw Wireframe", &enable_wireframe);
  ImGui::Checkbox("Show Texture Wnd", &show_textures_wnd);
  static int capture_frame_counter = 0;
  if (ImGui::Button("Export Frame Image", {-1, 30})) {
    std::string save_frame_capture_filepath;
    if (save_file_dialog("Save Frame Capture", {"*.png"},
                         save_frame_capture_filepath))
      color_tex.save_as_image().save_png(save_frame_capture_filepath);
  }
  ImGui::Separator();

  ImGui::MenuItem("Ambient Occlusion", nullptr, nullptr, false);
  ImGui::Checkbox("Enable AO", &enable_ao_pass);
  ImGui::InputInt("Filter Size", &ao_filter_size);
  ImGui::DragFloat("Filter Sigma", &ao_filter_sigma, 0.01f, 0.0f, 10.0f);
  ImGui::DragFloat("SSAO Radius", &ssao_radius, 0.01f, 0.0f, 10.0f);
  ImGui::DragFloat("SSAO Noise Scale", &ssao_noise_scale, 0.01f, 0.0f, 1000.0f);
  ImGui::Separator();

  ImGui::MenuItem("Post Processing", nullptr, nullptr, false);
  ImGui::Checkbox("Enable FXAA", &enable_fxaa);

  ImGui::MenuItem("Sun Light Settings", nullptr, nullptr, false);
  ImGui::Checkbox("Enable", &enable_sun);
  bool sun_parameter_modified = false;
  sun_parameter_modified |=
      ImGui::DragFloat("Sun Turbidity", &sun_turbidity, 0.1f, 0.0f, 100.0f);
  color_edit_3("Light Color", sun_color);
  sun_parameter_modified |= ImGui::DragFloat("Sun Horizontal Angle", &sun_h,
                                             0.1f, -180.0f, 180.0f, "%.3f");
  sun_parameter_modified |=
      ImGui::DragFloat("Sun Vertical Angle", &sun_v, 0.1f, 0.0f, 90.0f, "%.3f");
  ImGui::DragFloat("Sun Gamma", &ss_model.sun_gamma, 0.01f, 1.0f, 10.0f);
  if (sun_parameter_modified) {
    float sun_v_rad = sun_v / 180 * 3.1415927f;
    float sun_h_rad = sun_h / 180 * 3.1415927f;
    math::vector3 sun_dir(cos(sun_v_rad) * cos(sun_h_rad),
                          cos(sun_v_rad) * sin(sun_h_rad), sin(sun_v_rad));
    ss_model.update(sun_dir, sun_turbidity);
  }

  ImGui::MenuItem("Skinned Character Shadow Maps", nullptr, nullptr, false);
  ImGui::DragFloat("Max Bias Term", &shadowmap_max_bias, 0.000001f, 0.0f, 1.0f,
                   "%.6f");
  ImGui::DragFloat("Min Bias Term", &shadowmap_min_bias, 0.000001f, 0.0f, 1.0f,
                   "%.6f");

  ImGui::MenuItem("Scene Light Mask", nullptr, nullptr, false);
  ImGui::DragFloat("Light Mask Shadow Weight", &light_mask_shadow_weight, 0.1,
                   0.0, 1.0);

  ImGui::MenuItem("Cascaded Shadow Maps", nullptr, nullptr, false);
  ImGui::Checkbox("Enable CSM", &enable_csm);
  bool csm_modified = false;
  csm_modified |= ImGui::InputInt("Num Cascades", &num_cascades);
  num_cascades = std::max(1, std::min(num_cascades, 8));
  csm_modified |= ImGui::InputInt("CSM Dimension", &csm_depth_dim);
  ImGui::SliderFloat("Split Lambda", &csm_split_lambda, 0.0f, 1.0f);
  ImGui::DragFloat("Normal Offset Scale", &csm_normal_offset_scale, 0.01f,
                   0.0f, 10.0f, "%.3f");
  ImGui::DragFloat("Bias Scale", &csm_bias_scale, 0.01f, 0.0f, 10.0f,
                   "%.3f");
  ImGui::Checkbox("Debug Cascade Splits", &csm_debug_cascades);
  if (csm_modified)
    resize_csm_buffer();
}

void defered_render_system::save_color_buffer_as_png(std::string filepath) {
  color_tex.save_as_image().save_png(filepath);
}

void defered_render_system::show_textures_wnd_func() {
  ImGui::Begin("mixed_rsys_tex_wnd", &show_textures_wnd);
  auto size = ImGui::GetWindowSize();
  ImGui::Text("Shadow Mask");
  ImGui::Image(
      (void *)static_cast<std::uintptr_t>(scene_light_mask_tex.get_handle()),
      {size.x, size.x / canvas_width * canvas_height}, ImVec2(0, 1),
      ImVec2(1, 0));
  ImGui::End();
}

void defered_render_system::init0(entt::registry &registry) {
  pos_tex.create(GL_TEXTURE_2D);
  normal_tex.create(GL_TEXTURE_2D);
  gbuffer_depth_tex.create(GL_TEXTURE_2D);
  color_tex.create(GL_TEXTURE_2D);
  cbuffer_depth.create(GL_TEXTURE_2D);
  mask_tex.create(GL_TEXTURE_2D);
  ao_color.create(GL_TEXTURE_2D);
  csm_depth_atlas.create(GL_TEXTURE_2D);
  scene_light_mask_tex.create(GL_TEXTURE_2D);
  color_backup_tex.create(GL_TEXTURE_2D);
  albedo_tex.create(GL_TEXTURE_2D);

  noise_tex_random.create(GL_TEXTURE_2D);
  assets::image img;
  img.resize(64, 64, 1);
  for (int i = 0; i < 64; i++)
    for (int j = 0; j < 64; j++)
      img.pixel(i, j, 0) = (unsigned char)(255 * math::rand(0, 1));
  noise_tex_random.set_data_from_image(img);
  noise_tex_random.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                                   {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                                   {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                   {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  gbuffer_geometry_pass.compile_shader_from_source(gbuffer_geometry_pass_vs,
                                                   gbuffer_geometry_pass_fs,
                                                   gbuffer_geometry_pass_gs);
  defered_default_pass.compile_shader_from_source(quad_vs,
                                                  defered_default_pass_fs);

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
  ao_buffer.create();
  csm_buffer.create();
  scene_light_mask_buffer.create();
  csm_vp_matrix_buffer.create();
  shadow_depth_program.compile_shader_from_source(shadow_vs, shadow_fs);
  csm_selection_mask_program.compile_shader_from_source(quad_vs,
                                                        csm_selection_mask_fs);
  shadow_mask_program.compile_shader_from_source(quad_vs, shadow_mask_fs);
  static_mesh_light_mask_program.compile_shader_from_source(
      quad_vs, static_mesh_light_mask_fs);
  fxaa_program.compile_shader_from_source(quad_vs, fxaa_fs);

  float sun_v_rad = sun_v / 180 * 3.1415927f;
  float sun_h_rad = sun_h / 180 * 3.1415927f;
  math::vector3 sun_dir(cos(sun_v_rad) * cos(sun_h_rad),
                        cos(sun_v_rad) * sin(sun_h_rad), sin(sun_v_rad));
  ss_model.update(sun_dir, sun_turbidity);

  // ---------- call resize after all initialzation finishes ----------
  resize(canvas_width, canvas_height);
}

void defered_render_system::init1(entt::registry &registry) {
  resize(canvas_width, canvas_height);
}

void defered_render_system::resize(int width, int height) {
  canvas_width = width;
  canvas_height = height;

  gbuffer.bind();
  gbuffer.begin_draw_buffers();

  pos_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  pos_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                          {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                          {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                          {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  normal_tex.set_data(width, height, GL_RGB8, GL_RGB, GL_FLOAT);
  normal_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                             {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                             {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                             {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  mask_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  mask_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                           {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                           {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                           {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  albedo_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_FLOAT);
  albedo_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
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
  gbuffer.attach_color_buffer(albedo_tex, GL_COLOR_ATTACHMENT3);
  gbuffer.end_draw_buffers();
  gbuffer.attach_depth_buffer(gbuffer_depth_tex);
  if (!gbuffer.check_status())
    std::cout << "gbuffer not complete!" << std::endl;
  gbuffer.unbind();

  color_backup_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  color_backup_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                                   {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                                   {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                   {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  cbuffer.bind();
  cbuffer.begin_draw_buffers();
  color_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  color_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                            {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  cbuffer_depth.set_data(width, height, GL_DEPTH_COMPONENT24,
                         GL_DEPTH_COMPONENT, GL_FLOAT);
  cbuffer_depth.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                                {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                                {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  cbuffer.attach_color_buffer(color_tex, GL_COLOR_ATTACHMENT0);
  cbuffer.end_draw_buffers();
  cbuffer.attach_depth_buffer(cbuffer_depth);
  if (!cbuffer.check_status())
    std::cout << "cbuffer not complete!" << std::endl;
  cbuffer.unbind();

  scene_light_mask_buffer.bind();
  scene_light_mask_buffer.begin_draw_buffers();
  scene_light_mask_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  scene_light_mask_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                                       {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                                       {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                       {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  scene_light_mask_buffer.attach_color_buffer(scene_light_mask_tex,
                                              GL_COLOR_ATTACHMENT0);
  scene_light_mask_buffer.end_draw_buffers();
  if (!scene_light_mask_buffer.check_status())
    std::cout << "scene_light_mask_buffer not complete!" << std::endl;
  scene_light_mask_buffer.unbind();

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
    std::cout << "ao buffer not complete!" << std::endl;
  ao_buffer.unbind();

  resize_csm_buffer();
}

void defered_render_system::preupdate(entt::registry &registry) {
  // update vp matrices for cameras
  registry.view<camera>().each([&](entt::entity entity, camera &camera) {
    compute_vp_matrix(registry, entity, canvas_width, canvas_height);
  });
}

void defered_render_system::update_scene_buffers(entt::registry &registry) {
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
    std::cout << "Detect change in scene vertex count, scene index count, "
                 "scene mesh count or force update flag, "
                 "resize scene vertex buffer and scene index buffer."
              << std::endl;
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
    for (auto &bs : data.blendshapes) {
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
      if (!data.should_render_mesh)
        continue;
      for (int i = 0; i < data.blendshapes.size(); i++) {
        if (data.blendshapes[i].weight == 0.0f) {
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
            "gWeightValue", data.blendshapes[i].weight);
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
              registry.get<bone_node>(bundle.bone_entities[i]).offset_matrix;
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
          if (!data.should_render_mesh)
            continue;
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

  // for skinned mesh bundled character, we would compute its uniformed aabb, so
  // we can create a independent shadow map for it.
  registry.view<entt::entity, skinned_mesh_bundle>().each(
      [&](entt::entity entity, skinned_mesh_bundle &bundle_data) {
        // create aabb for each of its mesh component, then merge these aabb
        // into one uniform bounding box
        if (bundle_data.mesh_entities.size() == 0)
          return;
        bool should_render_any_mesh = false;
        int first_mesh_should_render = -1;
        for (int i = 0; i < bundle_data.mesh_entities.size(); i++) {
          if (registry.get<mesh_data>(bundle_data.mesh_entities[i])
                  .should_render_mesh) {
            should_render_any_mesh = true;
            first_mesh_should_render = i;
            break;
          }
        }
        if (!should_render_any_mesh)
          return;
        for (auto mesh_ent : bundle_data.mesh_entities) {
          auto &mesh_comp = registry.get<mesh_data>(mesh_ent);
          if (!mesh_comp.should_render_mesh)
            continue;
          auto [bb_min, bb_max] = per_mesh_global_aabb_program(
              scene_vertex_buffer, scene_index_buffer,
              mesh_comp.scene_vertex_offset, mesh_comp.scene_index_offset,
              mesh_comp.indices.size() / 3);
          mesh_comp.bb_min = bb_min;
          mesh_comp.bb_max = bb_max;
        }
        auto &mesh_comp0 = registry.get<mesh_data>(
            bundle_data.mesh_entities[first_mesh_should_render]);
        bundle_data.bb_min = mesh_comp0.bb_min;
        bundle_data.bb_max = mesh_comp0.bb_max;
        for (int i = 0; i < bundle_data.mesh_entities.size(); i++) {
          auto &mesh_comp =
              registry.get<mesh_data>(bundle_data.mesh_entities[i]);
          if (!mesh_comp.should_render_mesh)
            continue;
          bundle_data.bb_min = math::min3(bundle_data.bb_min, mesh_comp.bb_min);
          bundle_data.bb_max = math::max3(bundle_data.bb_max, mesh_comp.bb_max);
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
  scene_vao.link_attribute(scene_vertex_buffer, 2, 4, GL_FLOAT,
                           sizeof(_packed_vertex), (void *)(8 * sizeof(float)));
  scene_vao.unbind();
  scene_vertex_buffer.unbind_as(GL_ARRAY_BUFFER);
  scene_index_buffer.unbind_as(GL_ELEMENT_ARRAY_BUFFER);
}

void defered_render_system::update_scene_lights(entt::registry &registry) {
  float sun_v_rad = sun_v / 180 * 3.1415927f;
  float sun_h_rad = sun_h / 180 * 3.1415927f;
  sun_direction =
      -math::vector3(cos(sun_v_rad) * sin(sun_h_rad), sin(sun_v_rad),
                     cos(sun_v_rad) * cos(sun_h_rad));
}

void defered_render_system::resize_csm_buffer() {
  csm_buffer.bind();
  csm_depth_atlas.set_data(num_cascades * csm_depth_dim, csm_depth_dim,
                           GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_FLOAT);
  csm_depth_atlas.set_parameters(
      {{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
       {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
       {GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE},
       {GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL},
       {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
       {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  csm_buffer.attach_depth_buffer(csm_depth_atlas);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  if (!csm_buffer.check_status())
    std::cout << "csm buffer not complete!" << std::endl;
  csm_buffer.unbind();
}

void defered_render_system::compute_csm_matrices(camera &cam_comp,
                                                  transform &cam_trans) {
  float z_near = cam_comp.z_near;
  float z_far = cam_comp.z_far;

  // Practical split scheme (log/linear blend)
  csm_cascades[0] = z_near;
  for (int i = 1; i <= num_cascades; i++) {
    float p = (float)i / (float)num_cascades;
    float log_split = z_near * powf(z_far / z_near, p);
    float uni_split = z_near + (z_far - z_near) * p;
    csm_cascades[i] =
        csm_split_lambda * log_split + (1.0f - csm_split_lambda) * uni_split;
  }

  csm_vp_matrix.resize(num_cascades);

  float aspect = (float)canvas_width / (float)canvas_height;
  float fovy_rad = cam_comp.fovy_degree / 180.0f * 3.1415927f;
  float tan_half_fovy = tanf(fovy_rad * 0.5f);

  math::matrix4 view_inv = cam_comp.view.inverse();

  // Stable light basis — avoid degeneracy when sun is near vertical
  math::vector3 world_up(0.0f, 1.0f, 0.0f);
  if (std::abs(sun_direction.normalized().dot(world_up)) > 0.99f)
    world_up = math::vector3(0.0f, 0.0f, 1.0f);
  math::vector3 light_right = sun_direction.cross(world_up).normalized();
  math::vector3 light_up = light_right.cross(sun_direction).normalized();

  for (int i = 0; i < num_cascades; i++) {
    float near_split = csm_cascades[i];
    float far_split = csm_cascades[i + 1];

    // Compute frustum corners in view space (camera views -Z)
    float xn = near_split * tan_half_fovy * aspect;
    float yn = near_split * tan_half_fovy;
    float xf = far_split * tan_half_fovy * aspect;
    float yf = far_split * tan_half_fovy;

    math::vector4 view_corners[8] = {
        math::vector4(-xn, -yn, -near_split, 1.0f),
        math::vector4(xn, -yn, -near_split, 1.0f),
        math::vector4(xn, yn, -near_split, 1.0f),
        math::vector4(-xn, yn, -near_split, 1.0f),
        math::vector4(-xf, -yf, -far_split, 1.0f),
        math::vector4(xf, -yf, -far_split, 1.0f),
        math::vector4(xf, yf, -far_split, 1.0f),
        math::vector4(-xf, yf, -far_split, 1.0f),
    };

    // Transform to world space and compute center
    math::vector3 center = math::vector3::Zero();
    math::vector3 world_corners[8];
    for (int j = 0; j < 8; j++) {
      math::vector4 wc = view_inv * view_corners[j];
      world_corners[j] = math::vector3(wc.x(), wc.y(), wc.z());
      center += world_corners[j];
    }
    center /= 8.0f;

    // Build light view matrix looking along the sun direction
    math::matrix4 light_view =
        math::lookat(center, center + sun_direction, light_up);

    // Find tight bounding box in light space
    math::vector4 ls0 = light_view * math::vector4(world_corners[0].x(),
                                                    world_corners[0].y(),
                                                    world_corners[0].z(), 1.0f);
    float ls_min_x = ls0.x(), ls_min_y = ls0.y(), ls_min_z = ls0.z();
    float ls_max_x = ls0.x(), ls_max_y = ls0.y(), ls_max_z = ls0.z();
    for (int j = 1; j < 8; j++) {
      math::vector4 ls =
          light_view * math::vector4(world_corners[j].x(),
                                     world_corners[j].y(),
                                     world_corners[j].z(), 1.0f);
      ls_min_x = std::min(ls_min_x, ls.x());
      ls_min_y = std::min(ls_min_y, ls.y());
      ls_min_z = std::min(ls_min_z, ls.z());
      ls_max_x = std::max(ls_max_x, ls.x());
      ls_max_y = std::max(ls_max_y, ls.y());
      ls_max_z = std::max(ls_max_z, ls.z());
    }

    // Extend Z range to catch shadow casters behind camera frustum
    float z_range = ls_max_z - ls_min_z;
    ls_min_z -= z_range * 2.0f;

    math::matrix4 light_proj =
        math::ortho(ls_min_x, ls_max_x, ls_max_y, ls_min_y, ls_min_z, ls_max_z);
    csm_vp_matrix[i] = light_proj * light_view;

    // Store the world-space texel size for this cascade (used for adaptive bias)
    float cascade_world_width = ls_max_x - ls_min_x;
    float cascade_world_height = ls_max_y - ls_min_y;
    csm_texel_sizes[i] =
        std::max(cascade_world_width, cascade_world_height) / (float)csm_depth_dim;
  }
}

void defered_render_system::update_scene_data_structures(
    entt::registry &registry) {
  // build scene bvh on cpu with obb (static mesh) and aabb (deformable mesh)
  registry.view<entt::entity, transform, mesh_data>().each(
      [&](entt::entity entity, transform &trans, mesh_data &mesh_comp) {});
}

void defered_render_system::render(entt::registry &registry,
                                   transform &cam_trans, camera &cam_comp) {
  update_scene_lights(registry);

  // compute CSM cascade VP matrices
  if (enable_sun && enable_csm && cam_comp.perspective) {
    compute_csm_matrices(cam_comp, cam_trans);
  }
  auto trans_mesh_view = registry.view<entt::entity, transform, mesh_data>();
  auto skinned_mesh_bundle_view =
      registry.view<entt::entity, skinned_mesh_bundle>();

  // ------------------ render to geometry framebuffer ------------------
  {
    gbuffer.bind();
    gbuffer.set_viewport(0, 0, canvas_width, canvas_height);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    if (scene_mesh_counter > 0) {
      scene_vao.bind();
      gbuffer_geometry_pass.use();
      gbuffer_geometry_pass.set_mat4("gVP", cam_comp.vp);
      gbuffer_geometry_pass.set_mat4("gproj", cam_comp.proj);
      gbuffer_geometry_pass.set_vec2(
          "gViewport", math::vector2(canvas_width, canvas_height));
      trans_mesh_view.each([&](entt::entity entity, transform &trans,
                               mesh_data &data) {
        if (!visibility_check(cam_comp.planes, data.bb_min, data.bb_max,
                              data.skinned ? math::matrix4::Identity()
                                           : trans.matrix())) {
          return; // break if not visible
        }
        if (!data.should_render_mesh)
          return; // break if should not be rendered
        gbuffer_geometry_pass.set_vec3("albedo", data.material.albedo)
            .set_bool("wireframe", data.material.wireframe && enable_wireframe)
            .set_float("wireframe_width", data.material.wireframe_width)
            .set_float("wireframe_smooth", data.material.wireframe_smooth)
            .set_mat4("gModel",
                      data.skinned ? math::matrix4::Identity() : trans.matrix())
            .set_bool("has_albedo_tex",
                      data.material.albedo_tex_filepath != "");
        if (data.material.albedo_tex_filepath != "")
          gbuffer_geometry_pass.set_texture2d(
              "model_albedo_tex", data.material.albedo_tex.get_handle(), 0);
        glDrawElements(GL_TRIANGLES, data.indices.size(), GL_UNSIGNED_INT,
                       (void *)(data.scene_index_offset * sizeof(GLuint)));
      });
      scene_vao.unbind();
    }
    gbuffer.unbind();
  }

  // render ao buffer if needed
  if (enable_ao_pass) {
    ao_buffer.bind();
    ao_buffer.set_viewport(0, 0, canvas_width, canvas_height);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    ssao(pos_tex, normal_tex, mask_tex, cam_comp.view, cam_comp.proj,
         ssao_noise_scale, ssao_radius);
    ao_buffer.unbind();
  }

  // ---------------- render CSM depth atlas if enabled ----------------
  if (enable_sun && enable_csm && cam_comp.perspective && scene_mesh_counter > 0) {
    csm_buffer.bind();
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    // Clear entire atlas once
    csm_buffer.set_viewport(0, 0, num_cascades * csm_depth_dim, csm_depth_dim);
    glClear(GL_DEPTH_BUFFER_BIT);
    scene_vao.bind();
    shadow_depth_program.use();
    for (int cascade_idx = 0; cascade_idx < num_cascades; cascade_idx++) {
      csm_buffer.set_viewport(cascade_idx * csm_depth_dim, 0, csm_depth_dim,
                              csm_depth_dim);
      shadow_depth_program.set_mat4("gVP", csm_vp_matrix[cascade_idx]);
      trans_mesh_view.each(
          [&](entt::entity entity, transform &trans, mesh_data &data) {
            if (!data.should_render_mesh)
              return;
            shadow_depth_program.set_mat4(
                "gModel",
                data.skinned ? math::matrix4::Identity() : trans.matrix());
            glDrawElements(GL_TRIANGLES, data.indices.size(), GL_UNSIGNED_INT,
                           (void *)(data.scene_index_offset * sizeof(GLuint)));
          });
    }
    scene_vao.unbind();
    csm_buffer.unbind();
  }

  // ------------- render per-character shadow maps if sun is enabled -----------
  {
    // render per bundle shadow map if sun is enabled
    static math::vector3 tmp_sun_up_dir =
        math::vector3(0.0, 0.97, 0.3).normalized();
    if (enable_sun) {
      skinned_mesh_bundle_view.each([&](entt::entity entity,
                                        skinned_mesh_bundle &bundle_data) {
        bundle_data.try_setup();
        bundle_data.shadowmap_fb.bind();
        bundle_data.shadowmap_fb.set_viewport(0, 0, 4096, 4096);
        glClear(GL_DEPTH_BUFFER_BIT);
        glClearColor(0, 0, 0, 1);
        scene_vao.bind();
        shadow_depth_program.use();
        math::vector3 center = (bundle_data.bb_min + bundle_data.bb_max) / 2;
        float radius = (bundle_data.bb_max - bundle_data.bb_min).norm() / 2;
        math::vector3 norm_dir =
            sun_direction.cross(tmp_sun_up_dir).normalized();
        bundle_data.shadow_vp =
            math::ortho(-radius, radius, radius, -radius, -50, 50) *
            math::lookat(center, center + sun_direction, norm_dir);
        update_bounding_planes(bundle_data.vis_planes, bundle_data.shadow_vp);
        shadow_depth_program.set_mat4("gVP", bundle_data.shadow_vp);
        trans_mesh_view.each([&](entt::entity entity, transform &trans,
                                 mesh_data &data) {
          if (!data.should_render_mesh ||
              !visibility_check(
                  bundle_data.vis_planes, data.bb_min, data.bb_max,
                  data.skinned ? math::matrix4::Identity() : trans.matrix())) {
            return; // break if not visible
          }
          shadow_depth_program.set_mat4("gModel",
                                        data.skinned ? math::matrix4::Identity()
                                                     : trans.matrix());
          glDrawElements(GL_TRIANGLES, data.indices.size(), GL_UNSIGNED_INT,
                         (void *)(data.scene_index_offset * sizeof(GLuint)));
        });
        scene_vao.unbind();
        bundle_data.shadowmap_fb.unbind();
      });
    }
  }

  // render scene light mask texture
  {
    scene_light_mask_buffer.bind();
    scene_light_mask_buffer.set_viewport(0, 0, canvas_width, canvas_height);
    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    if (enable_sun) {
      // Base layer: scene-wide shadow (CSM or N·L fallback)
      if (enable_csm && cam_comp.perspective) {
        csm_vp_matrix_buffer.set_data_ssbo(csm_vp_matrix, GL_DYNAMIC_DRAW);
        csm_selection_mask_program.use();
        csm_selection_mask_program.set_buffer_ssbo(csm_vp_matrix_buffer, 0)
            .set_int("num_cascades", num_cascades)
            .set_int("csm_depth_dim", csm_depth_dim)
            .set_float("normal_offset_scale", csm_normal_offset_scale)
            .set_float("bias_scale", csm_bias_scale)
            .set_bool("debug_cascades", csm_debug_cascades)
            .set_float("shadow_weight", light_mask_shadow_weight)
            .set_vec3("light_dir", sun_direction)
            .set_mat4("cam_view", cam_comp.view);
        for (int i = 0; i <= num_cascades; i++) {
          csm_selection_mask_program.set_float(
              str_format("csm_cascades[%d]", i), csm_cascades[i]);
        }
        for (int i = 0; i < num_cascades; i++) {
          csm_selection_mask_program.set_float(
              str_format("csm_texel_sizes[%d]", i), csm_texel_sizes[i]);
        }
        csm_selection_mask_program.set_texture2d("scene_pos",
                                                 pos_tex.get_handle(), 0);
        csm_selection_mask_program.set_texture2d("scene_normal",
                                                 normal_tex.get_handle(), 1);
        csm_selection_mask_program.set_texture2d("scene_mask",
                                                 mask_tex.get_handle(), 2);
        csm_selection_mask_program.set_texture2d(
            "cascade_depth", csm_depth_atlas.get_handle(), 3);
        quad_draw_call();
      } else {
        // Fallback: N·L only (no shadow map)
        static_mesh_light_mask_program.use();
        static_mesh_light_mask_program.set_vec3("light_dir", sun_direction)
            .set_float("shadow_weight", light_mask_shadow_weight);
        static_mesh_light_mask_program.set_texture2d("scene_pos",
                                                     pos_tex.get_handle(), 0);
        static_mesh_light_mask_program.set_texture2d(
            "scene_normal", normal_tex.get_handle(), 1);
        static_mesh_light_mask_program.set_texture2d("scene_mask",
                                                     mask_tex.get_handle(), 2);
        quad_draw_call();
      }

      // Overlay per-character high-res shadows with MIN blending
      // (only darkens — preserves whichever shadow is stronger)
      glEnable(GL_BLEND);
      glBlendEquation(GL_MIN);
      skinned_mesh_bundle_view.each([&](entt::entity entity,
                                        skinned_mesh_bundle &bundle_data) {
        shadow_mask_program.use();
        shadow_mask_program.set_mat4("shadow_vp", bundle_data.shadow_vp)
            .set_int("shadowmap_dim", 4096)
            .set_float("max_bias", shadowmap_max_bias)
            .set_float("min_bias", shadowmap_min_bias)
            .set_vec3("light_dir", sun_direction)
            .set_float("shadow_weight", light_mask_shadow_weight)
            .set_vec2("viewport_size",
                      math::vector2(canvas_width, canvas_height));

        shadow_mask_program.set_texture2d("scene_pos", pos_tex.get_handle(), 0);
        shadow_mask_program.set_texture2d("scene_normal",
                                          normal_tex.get_handle(), 1);
        shadow_mask_program.set_texture2d("scene_mask", mask_tex.get_handle(),
                                          2);
        shadow_mask_program.set_texture2d(
            "shadowmap", bundle_data.shadowmap_depth.get_handle(), 3);
        quad_draw_call();
      });
      glBlendEquation(GL_FUNC_ADD);
      glDisable(GL_BLEND);
    }

    glClearColor(0, 0, 0, 1);
    scene_light_mask_buffer.unbind();
  }

  // ------------- render to default color framebuffer -------------
  {
    cbuffer.bind();
    cbuffer.set_viewport(0, 0, canvas_width, canvas_height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    glDisable(GL_DEPTH_TEST);
    if (!use_pure_color_bg) {
      ss_model.render(cam_comp.vp, cam_trans.world_pos());
    } else {
    }
    glEnable(GL_DEPTH_TEST);

    defered_default_pass.use();
    defered_default_pass.set_texture2d("pos_tex", pos_tex.get_handle(), 0);
    defered_default_pass.set_texture2d("normal_tex", normal_tex.get_handle(),
                                       1);
    defered_default_pass.set_texture2d("mask_tex", mask_tex.get_handle(), 2);
    defered_default_pass.set_texture2d("albedo_tex", albedo_tex.get_handle(),
                                       3);
    defered_default_pass.set_texture2d("light_mask",
                                       scene_light_mask_tex.get_handle(), 4);
    defered_default_pass.set_texture2d("gbuffer_depth",
                                       gbuffer_depth_tex.get_handle(), 5);
    defered_default_pass.set_texture2d("cbuffer_depth",
                                       cbuffer_depth.get_handle(), 6);
    quad_draw_call();

    // ------------------- debug rendering -------------------

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
      if (auto app_ptr = registry.ctx().get<iapp *>()) {
        auto ss_handler = app_ptr->get_sys<sub_system_handler>();
        ss_handler->proxy_draw_to_scene(registry, cam_trans, cam_comp);
      }
    }

    for (auto &f : custom_draw_func)
      f();

    // render skeleton for skinned mesh bundle
    skinned_mesh_bundle_view.each([&](entt::entity entity,
                                      skinned_mesh_bundle &bundle_data) {
      if (!bundle_data.actor_draw_skeleton)
        return;
      for (int actor_idx = 0; actor_idx < bundle_data.actor_entities.size();
           actor_idx++) {
        auto actor_entity = bundle_data.actor_entities[actor_idx];
        if ((actor_idx >= bundle_data.actor_draw.size()) ||
            !bundle_data.actor_draw[actor_idx])
          continue;
        auto &actor_comp = registry.get<actor>(actor_entity);
        bundle_data._actor_active_joint_entities.clear();
        for (int i = 0; i < actor_comp.joint_active.size(); i++)
          if (actor_comp.joint_active[i])
            bundle_data._actor_active_joint_entities.insert(
                actor_comp.ordered_entities[i]);
        collect_skeleton_draw_queue(registry, actor_comp,
                                    bundle_data._actor_draw_queue);
        if (bundle_data._actor_draw_queue.size() == 0)
          continue;
        draw_bones(bundle_data._actor_draw_queue, cam_comp.vp,
                   bundle_data.actor_bone_color, bundle_data.actor_bone_alpha,
                   !bundle_data.actor_bones_on_top);
        float avg_bone_length = 0.0f;
        for (int i = 0; i < bundle_data._actor_draw_queue.size(); i++)
          avg_bone_length += (bundle_data._actor_draw_queue[i].first -
                              bundle_data._actor_draw_queue[i].second)
                                 .norm();
        avg_bone_length /= bundle_data._actor_draw_queue.size();
        if (bundle_data.actor_draw_axes) {
          bundle_data._actor_x_dir.clear();
          bundle_data._actor_y_dir.clear();
          bundle_data._actor_z_dir.clear();
          bundle_data._actor_x_dir.reserve(
              bundle_data._actor_active_joint_entities.size());
          bundle_data._actor_y_dir.reserve(
              bundle_data._actor_active_joint_entities.size());
          bundle_data._actor_z_dir.reserve(
              bundle_data._actor_active_joint_entities.size());
          for (auto joint_entity : bundle_data._actor_active_joint_entities) {
            auto &joint_trans = registry.get<transform>(joint_entity);
            bundle_data._actor_x_dir.push_back(std::make_pair(
                joint_trans.world_pos(),
                joint_trans.world_pos() + bundle_data.actor_axes_length *
                                              avg_bone_length *
                                              joint_trans.local_right()));
            bundle_data._actor_y_dir.push_back(std::make_pair(
                joint_trans.world_pos(),
                joint_trans.world_pos() + bundle_data.actor_axes_length *
                                              avg_bone_length *
                                              joint_trans.local_up()));
            bundle_data._actor_z_dir.push_back(std::make_pair(
                joint_trans.world_pos(),
                joint_trans.world_pos() + bundle_data.actor_axes_length *
                                              avg_bone_length *
                                              joint_trans.local_forward()));
          }
          draw_arrows(bundle_data._actor_x_dir, cam_comp.vp, Red,
                      0.1f * bundle_data.actor_axes_length * avg_bone_length,
                      bundle_data.actor_bone_alpha,
                      !bundle_data.actor_bones_on_top);
          draw_arrows(bundle_data._actor_y_dir, cam_comp.vp, Green,
                      0.1f * bundle_data.actor_axes_length * avg_bone_length,
                      bundle_data.actor_bone_alpha,
                      !bundle_data.actor_bones_on_top);
          draw_arrows(bundle_data._actor_z_dir, cam_comp.vp, Blue,
                      0.1f * bundle_data.actor_axes_length * avg_bone_length,
                      bundle_data.actor_bone_alpha,
                      !bundle_data.actor_bones_on_top);
        }
        if (bundle_data.actor_draw_spheres) {
          bundle_data._actor_joint_positions.clear();
          bundle_data._actor_joint_positions.reserve(
              bundle_data._actor_active_joint_entities.size());
          for (auto joint_entity : bundle_data._actor_active_joint_entities) {
            auto &joint_trans = registry.get<transform>(joint_entity);
            bundle_data._actor_joint_positions.push_back(
                joint_trans.world_pos());
          }
          draw_spheres(bundle_data._actor_joint_positions, cam_comp.vp,
                       0.08f * avg_bone_length, bundle_data.actor_bone_color,
                       false, bundle_data.actor_bone_alpha,
                       !bundle_data.actor_bones_on_top);
        }
        if (bundle_data.actor_draw_names) {
          for (auto joint_entity : bundle_data._actor_active_joint_entities) {
            auto &joint_trans = registry.get<transform>(joint_entity);
            draw_text3d(joint_trans.name, joint_trans.world_pos(),
                        math::quat::Identity(), cam_comp.vp, White, 0.0f, 0.02f,
                        1.0f, 1.0f, 0.0f, 1.0f, 1.0f, false);
          }
        }
        bundle_data._actor_draw_queue.clear();
      }
    });

    // ------------------- apply post processing -------------------
    glEnable(GL_BLEND);
    // 1. ambient occlusion
    if (enable_ao_pass) {
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBlendEquation(GL_FUNC_ADD);
      ao_gaussian_filter(ao_color, ao_filter_size, ao_filter_sigma);
    }
    glDisable(GL_BLEND);

    // 2. fxaa
    if (enable_fxaa) {
      glCopyImageSubData(color_tex.get_handle(), GL_TEXTURE_2D, 0, 0, 0, 0,
                         color_backup_tex.get_handle(), GL_TEXTURE_2D, 0, 0, 0,
                         0, canvas_width, canvas_height, 1);
      fxaa_program.use();
      fxaa_program.set_vec2("viewport_size",
                            math::vector2(canvas_width, canvas_height));
      fxaa_program.set_texture2d("color_tex", color_backup_tex.get_handle(), 0);
      quad_draw_call();
    }

    cbuffer.unbind();
  }

  custom_draw_func.clear();
}

}; // namespace toolkit::opengl3d