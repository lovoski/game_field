#pragma once

#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

#include "toolkit/opengl/draw.hpp"

void draw_view_frustom(toolkit::transform &trans, float fovy_deg, float znear,
                       float zfar, float w_div_h, toolkit::math::matrix4 &vp,
                       toolkit::math::vector3 color = toolkit::opengl::White);

struct vis_frsutom_bbs : public toolkit::scriptable {
  void draw_gui(toolkit::iapp *app) override {
    ImGui::InputInt("Num Cascades", &num_cascades, 1);
    ImGui::DragFloat("Split Lambda", &csm_split_lambda, 0.01f, 0.0f, 1.0f);
  }

  void draw_to_scene(toolkit::iapp *app) override {
    toolkit::opengl::script_draw_to_scene_proxy(
        app, [&](toolkit::opengl::editor *editor, toolkit::transform &cam_trans,
                 toolkit::opengl::camera &cam_comp) {
          if (auto ent_cam =
                  registry->try_get<toolkit::opengl::camera>(entity)) {
            auto ent_trans = registry->get<toolkit::transform>(entity);
            float scn_w = toolkit::opengl::g_instance.scene_width;
            float scn_h = toolkit::opengl::g_instance.scene_height;
            std::vector<float> csm_split_depth(num_cascades + 1);
            csm_split_depth[0] = ent_cam->z_near;
            for (int i = 0; i < num_cascades; i++) {
              csm_split_depth[i + 1] =
                  csm_split_lambda *
                      (ent_cam->z_near * pow(ent_cam->z_far / ent_cam->z_near,
                                             (float)(i + 1) / num_cascades)) +
                  (1.0f - csm_split_lambda) *
                      (ent_cam->z_near +
                       (i + 1) * (ent_cam->z_far - ent_cam->z_near) /
                           num_cascades);
              auto [bbs_center, bbs_radius] =
                  toolkit::opengl::frustom_bounding_sphere(
                      csm_split_depth[i], csm_split_depth[i + 1],
                      ent_cam->fovy_degree, scn_w, scn_h);
              toolkit::math::vector4 tmp_point;
              tmp_point << bbs_center, 1.0f;
              tmp_point = ent_trans.matrix() * tmp_point;
              toolkit::opengl::draw_wire_sphere(tmp_point.head<3>(),
                                                cam_comp.vp, bbs_radius);
              draw_view_frustom(ent_trans, ent_cam->fovy_degree,
                                csm_split_depth[i], csm_split_depth[i + 1],
                                scn_w / scn_h, cam_comp.vp,
                                toolkit::opengl::Red);
            }
          }
        });
  }

  int num_cascades = 3;
  float csm_split_lambda = 0.7f;
};
DECLARE_SCRIPT(vis_frsutom_bbs, debug)
