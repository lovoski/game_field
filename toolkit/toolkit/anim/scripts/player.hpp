#pragma once

#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit::anim {

class motion_player : public scriptable {
public:
  inline static float system_frame_time = 0.0f;
};
DECLARE_SCRIPT(motion_player, animation)

}; // namespace toolkit::anim