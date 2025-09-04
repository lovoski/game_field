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

class smplx : public scriptable {
public:
  void start() override;
  void destroy() override;

  void draw_gui(iapp *app) override;
  void draw_to_scene(iapp *app) override;

private:
  int model_index = 0, gender_index = 0;
  std::string model_path = "", model_type = "smpl", gender_type = "NEUTRAL";
  std::vector<entt::entity> bone_entities;
  std::vector<math::vector3> joint_rest_world_pos;
  cnpy::npz_t smpl_data;

  void setup_smplx_model(cnpy::npz_t &data);
  void setup_smpl_model(cnpy::npz_t &data);

  REFLECT_PRIVATE(smplx)
};
DECLARE_SCRIPT(smplx, utils, model_index, gender_index, model_type, gender_type)

}; // namespace toolkit::opengl