#include "camdmpp.hpp"

namespace toolkit::opengl3d {

void camdmpp::handle_engine_gui() {}

void camdmpp::handle_custom_initialization() {
  set_game_mode(true, hide_mouse);
  model.setup("camdmpp/model.onnx", "camdmpp/config.json");
}

void camdmpp::handle_game_logic_tick(float dt) {
  model.process_completions();

  // most of the logic are executed with fixed interval (60fps)
  double residual = __cur_time - __cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    fixed_interval_logic();
    residual -= fixed_interval;
    __cur_exec_fixed += 1;
  }
  __cur_time += dt;

  // update application state based on user input
  if (is_key_triggered(SDLK_ESCAPE))
    quit_app_running();
  if (is_key_triggered(SDLK_1)) {
    hide_mouse = !hide_mouse;
    set_game_mode(true, hide_mouse);
  }

  // update the camera movement every logic tick after character update
}

void camdmpp::fixed_interval_logic() {
  // make new predictions to the trajectory based on user input
  {}

  // apply pose to the character
  {

    // increase the apply frame counter
    applied_frames++;
  }

  // submit a new prediction when counter reaches a threashold
  if ((applied_frames >= submit_prediction_interval) &&
      !(waiting_for_model_output.load())) {
    waiting_for_model_output.store(true);
    model.submit_inference([this](std::vector<float> model_output) {
      printf("Inference finished when applied_frames=%d, start update pose "
             "cache\n",
             applied_frames);
      applied_frames = 0;
      waiting_for_model_output.store(false);
    });
  }
}

}; // namespace toolkit::opengl3d