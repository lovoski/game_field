#pragma once

#include "toolkit/opengl/sdl_context.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

#include "toolkit/transform.hpp"

#include <SDL_image.h>
#include <SDL_mixer.h>

#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_sdlrenderer2.h>
#include <imgui.h>

namespace toolkit::sdl2d {

class engine2d : public iapp {
public:
  void init(int width = 1280, int height = 720);
  void run();
  void shutdown();

  void late_deserialize(nlohmann::json &j) override;
  void late_serialize(nlohmann::json &j) override;

private:
  SDL_Window *window = nullptr;
  SDL_Renderer *renderer = nullptr;
  SDL_Event event;

  bool engine_running = true;
};

}; // namespace toolkit::sdl2d