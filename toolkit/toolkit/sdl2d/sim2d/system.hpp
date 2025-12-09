#pragma once

#include "toolkit/sdl2d/header.hpp"

namespace toolkit::sdl2d {

struct body {
  body() { reset(); }

  math::vector2 position;
  float rotation;

  math::vector2 linear_velocity;
  float angular_velocity;

  math::vector2 force;
  float torque;

  math::vector2 size;

  float friction;
  float mass, inv_mass;
  float I, inv_I;

  void reset();
  void setup(const math::vector2 &_size,
             float _m = std::numeric_limits<float>::max());
  void add_force(const math::vector2 &_force) { force += _force; }
};

union feature_pair {
  struct edges {
    char inEdge1;
    char outEdge1;
    char inEdge2;
    char outEdge2;
  } e;
  int value;
};

struct contact {
  math::vector2 position, normal;
  math::vector2 r1, r2;

  float separation;
  float Pn;  // accumulated normal impulse
  float Pt;  // accumulated tangent impulse
  float Pnb; // accumulated normal impulse for position bias
  float massNormal, massTangent;
  float bias;
  feature_pair feature;
};

struct arbiter_key {
  arbiter_key(body *_b1, body *_b2) {
    if (_b1 < _b2) {
      b1 = _b1;
      b2 = _b2;
    } else {
      b1 = _b2;
      b2 = _b1;
    }
  }
  body *b1, *b2;
};

struct arbiter {
  arbiter() {}
  arbiter(body *_b1, body *_b2);
  void update(std::vector<contact> &_contacts);
  void prestep(float inv_dt);
  void apply_impulse();

  body *b1, *b2;
  float friction;
  std::vector<contact> contacts;
};

inline bool operator<(const arbiter_key &a1, const arbiter_key &a2) {
  if (a1.b1 < a2.b1)
    return true;
  if ((a1.b1 == a2.b1) && (a1.b2 < a2.b2))
    return true;
  return false;
}

bool collide(std::vector<contact> &contacts, body *b1, body *b2);

class sim_sys_2d : public isystem {
public:
  void update(entt::registry &registry, float dt) override;
  void step(entt::registry &registry, float dt);

  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_menu_gui() override;

  void broadphase(entt::registry &registry);

  std::string get_name() override { return "Physics System"; }

  math::vector2 gravity = math::vector2(0.0f, 9.8f);

  inline static bool accumulate_impulses = true, warm_starting = true,
                     position_correction = true;

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f;

  bool fixed_timestep = true;
  int num_sub_steps = 20, sim_fps = 60;

  std::vector<body *> bodies_cache;
  std::map<arbiter_key, arbiter> arbiters;

  REFLECT_PRIVATE(sim_sys_2d)
};
DECLARE_SYSTEM(sim_sys_2d, num_sub_steps, gravity, fixed_timestep)

}; // namespace toolkit::sdl2d