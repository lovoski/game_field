/**
 * Implementation for SMPL and SMPLX
 */
#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"
#include <cnpy.h>

namespace toolkit::opengl {

class smplx : public sub_system {
public:
  void start() override;
  void init1() override;
  void destroy() override;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp) override;

  // Modify the weights of shape0-9 blendshapes, change the offset matrices and
  // transforms of joint entity transforms
  void apply_smpl_betas(entt::registry &registry, std::vector<float> betas);
  // Change the body shape and skeleton of SMPLX
  void apply_smplx_betas(entt::registry &registry, std::vector<float> betas);
  // Tune pose based blend shape weights
  void preupdate(entt::registry &registry, float dt) override;

private:
  int model_index = 0, gender_index = 0, num_betas = 10;
  std::vector<float> beta_cache;
  std::string model_path = "", model_type = "smpl", gender_type = "NEUTRAL";
  std::vector<entt::entity> bone_entities;
  std::vector<math::vector3> joint_rest_world_pos;
  cnpy::npz_t smpl_data;

  void setup_smplx_model(entt::registry &registry, cnpy::npz_t &data);
  void setup_smpl_model(entt::registry &registry, cnpy::npz_t &data);

  REFLECT_PRIVATE(smplx)
};
DECLARE_SUB_SYSTEM(smplx, utils, model_index, gender_index, model_type, gender_type,
               num_betas, beta_cache, bone_entities)

}; // namespace toolkit::opengl