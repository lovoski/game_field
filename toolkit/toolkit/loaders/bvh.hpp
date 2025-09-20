#pragma once

#include "toolkit/math.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/utils.hpp"

namespace toolkit::assets {

struct bvh_data {
  float frametime;
  std::vector<std::vector<math::quat>> local_rot;
  std::vector<std::vector<math::vector3>> local_pos;

  std::vector<int> parents;
  std::vector<std::string> names;
  std::vector<math::vector3> offsets;
};

bvh_data load_bvh(std::string filepath);

void save_bvh(std::string filepath, bvh_data &data, bool save_position = true);

}; // namespace toolkit::assets