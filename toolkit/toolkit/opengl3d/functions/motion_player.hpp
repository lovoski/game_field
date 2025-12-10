// #pragma once

// #include "toolkit/anim/components/actor.hpp"
// #include "toolkit/loaders/bvh.hpp"
// #include "toolkit/opengl/base.hpp"
// #include "toolkit/opengl/draw.hpp"
// #include "toolkit/scriptable.hpp"
// #include <cnpy.h>

// namespace toolkit::anim {

// class bvh_motion_player : public sub_system {
// public:
//   void start(entt::registry &registry) override {}
//   void destroy(entt::registry &registry) override {}

//   void update(entt::registry &registry, float dt) override;
//   void lateupdate(entt::registry &registry, float dt) override;
//   void draw_gui(entt::registry &registry, entt::entity entity) override;

//   entt::entity load_motion(entt::registry &registry, std::string filepath);

//   static inline float current_time = 0.0f;
//   static inline float play_speed = 1.0f;
//   static inline bool auto_play = false;

//   bool apply_motion = true;

// private:
//   bool motion_loaded = false;
//   assets::bvh_data motion;
// };
// DECLARE_SUB_SYSTEM(bvh_motion_player, animation)

// void import_all_bvh_motion(entt::registry &registry, std::string dirpath,
//                            float scale = 1.0f);

// }; // namespace toolkit::anim
