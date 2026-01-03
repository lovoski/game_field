#pragma once

#include "diffusion.hpp"

#include "toolkit/opengl3d/engine.hpp"

namespace toolkit::opengl3d {

class camdmpp : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_game_logic_tick(float dt) override;
  void handle_engine_gui() override;

private:
};

}; // namespace toolkit::opengl3d