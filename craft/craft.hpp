#pragma once

#include "toolkit/opengl3d/engine.hpp"

namespace toolkit::opengl3d {

class craft : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_custom_cleanup() override;
  void handle_engine_gui() override;

  void engine_fixed_update(float dt);

  void run();
  void reset() override;

private:
  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 90.0f;

  // engine render properties
};

}; // namespace toolkit::opengl3d