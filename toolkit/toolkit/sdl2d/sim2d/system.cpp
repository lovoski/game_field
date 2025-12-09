#include "toolkit/sdl2d/sim2d/system.hpp"

namespace toolkit::sdl2d {

inline float cross(const math::vector2 &a, const math::vector2 &b) {
  return a.x() * b.y() - a.y() * b.x();
}

inline math::vector2 cross(const math::vector2 &a, float s) {
  return math::vector2(s * a.y(), -s * a.x());
}

inline math::vector2 cross(float s, const math::vector2 &a) {
  return math::vector2(-s * a.y(), s * a.x());
}

void body::reset() {
  position = math::vector2::Zero();
  rotation = 0.0f;
  linear_velocity = math::vector2::Zero();
  angular_velocity = 0.0f;
  force = math::vector2::Zero();
  torque = 0.0f;
  size = math::vector2(1.0f, 1.0f);
  friction = 0.2f;
  mass = std::numeric_limits<float>::max();
  inv_mass = 0.0f;
  I = std::numeric_limits<float>::max();
  inv_I = 0.0f;
}

void body::setup(const math::vector2 &_size, float _m) {
  reset();
  size = _size;
  mass = _m;
  if (mass < std::numeric_limits<float>::max()) {
    inv_mass = 1.0f / mass;
    // box inertia
    I = mass * (size.x() * size.x() + size.y() * size.y()) / 12.0f;
    inv_I = 1.0f / I;
  } else {
    mass = std::numeric_limits<float>::max();
    inv_mass = 0.0f;
    I = std::numeric_limits<float>::max();
    inv_I = 0.0f;
  }
}

arbiter::arbiter(body *_b1, body *_b2) {
  if (_b1 < _b2) {
    b1 = _b1;
    b2 = _b2;
  } else {
    b1 = _b2;
    b2 = _b1;
  }
  collide(contacts, b1, b2);
  friction = std::sqrtf(b1->friction * b2->friction); // TODO: what's this?
}

void arbiter::update(std::vector<contact> &_contacts) {
  std::vector<contact> merged_contacts;
  for (int i = 0; i < _contacts.size(); i++) {
    auto &c_new = _contacts[i];
    int k = -1;
    for (int j = 0; j < contacts.size(); j++) {
      auto &c_old = contacts[j];
      if (c_new.feature.value == c_old.feature.value) {
        k = j;
        break;
      }
    }
    merged_contacts.push_back(c_new);
    if (k > -1) {
      auto &c_old = contacts[k];
      auto &c = merged_contacts[merged_contacts.size() - 1];
      if (sim_sys_2d::warm_starting) {
        c.Pn = c_old.Pn;
        c.Pt = c_old.Pt;
        c.Pnb = c_old.Pnb;
      } else {
        c.Pn = 0.0f;
        c.Pt = 0.0f;
        c.Pnb = 0.0f;
      }
    }
  }
  contacts = merged_contacts;
}

void arbiter::prestep(float inv_dt) {
  const float k_allowed_penetration = 0.01f;
  float k_bias_factor =
      sim_sys_2d::position_correction ? 0.2f : 0.0f; // TODO: what's this?
  for (int i = 0; i < contacts.size(); i++) {
    auto &c = contacts[i];
    math::vector2 r1 = c.position - b1->position;
    math::vector2 r2 = c.position - b2->position;
    // precompute normal mass, tangent mass and bias
    float rn1 = r1.dot(c.normal);
    float rn2 = r2.dot(c.normal);
    float k_normal = b1->inv_mass + b2->inv_mass;
    k_normal += b1->inv_I * (r1.dot(r1) - rn1 * rn1) +
                b2->inv_I * (r2.dot(r2) - rn2 * rn2);
    c.massNormal = 1.0f / k_normal;

    math::vector2 tangent = cross(c.normal, 1.0f);
    float rt1 = r1.dot(tangent);
    float rt2 = r2.dot(tangent);
    float k_tangent = b1->inv_mass + b2->inv_mass;
    k_tangent += b1->inv_I * (r1.dot(r1) - rt1 * rt1) +
                 b2->inv_I * (r2.dot(r2) - rt2 * rt2);
    c.massTangent = 1.0f / k_tangent;
    c.bias = -k_bias_factor * inv_dt *
             std::min(0.0f, c.separation + k_allowed_penetration);
    if (sim_sys_2d::accumulate_impulses) {
      // apply normal + friction impulse
      math::vector2 P = c.Pn * c.normal + c.Pt * tangent;
      b1->linear_velocity -= b1->inv_mass * P;
      b1->angular_velocity -= b1->inv_I * cross(r1, P);
      b2->linear_velocity += b2->inv_mass * P;
      b2->angular_velocity += b2->inv_I * cross(r2, P);
    }
  }
}

void arbiter::apply_impulse() {
  for (int i = 0; i < contacts.size(); i++) {
    auto &c = contacts[i];
    c.r1 = c.position - b1->position;
    c.r2 = c.position - b2->position;
    // relative velocity at contact
    math::vector2 dv = b2->linear_velocity + cross(b2->angular_velocity, c.r2) -
                       b1->linear_velocity - cross(b1->angular_velocity, c.r1);
    // compute normal impulse
    float vn = dv.dot(c.normal);
    float dPn = c.massNormal * (-vn + c.bias);
    if (sim_sys_2d::accumulate_impulses) {
      float Pn0 = c.Pn;
      c.Pn = std::max(Pn0 + dPn, 0.0f);
      dPn = c.Pn - Pn0;
    } else {
      dPn = std::max(dPn, 0.0f);
    }
    // apply contact impulse
    math::vector2 Pn = dPn * c.normal;
    b1->linear_velocity -= b1->inv_mass * Pn;
    b1->angular_velocity -= b1->inv_I * cross(c.r1, Pn);
    b2->linear_velocity += b2->inv_mass * Pn;
    b2->angular_velocity += b2->inv_I * cross(c.r2, Pn);
    // relative velocity at contact
    dv = b2->linear_velocity + cross(b2->angular_velocity, c.r2) -
         b1->linear_velocity - cross(b1->angular_velocity, c.r1);
    math::vector2 tangent = cross(c.normal, 1.0f);
    float vt = dv.dot(tangent);
    float dPt = c.massTangent * (-vt);
    if (sim_sys_2d::accumulate_impulses) {
      // compute friction impulse
      float maxPt = friction * c.Pn;
      // clamp friction
      float old_tangent_impulse = c.Pt;
      c.Pt = std::clamp(old_tangent_impulse + dPt, -maxPt, maxPt);
      dPt = c.Pt - old_tangent_impulse;
    } else {
      float maxPt = friction * dPn;
      dPt = std::clamp(dPt, -maxPt, maxPt);
    }
    // apply contact impulse
    math::vector2 Pt = dPt * tangent;
    b1->linear_velocity -= b1->inv_mass * Pt;
    b1->angular_velocity -= b1->inv_I * cross(c.r1, Pt);
    b2->linear_velocity += b2->inv_mass * Pt;
    b2->angular_velocity += b2->inv_I * cross(c.r2, Pt);
  }
}

void sim_sys_2d::broadphase(entt::registry &registry) {
  // O(n^2)
  for (int i = 0; i < bodies_cache.size(); i++) {
    auto bi = bodies_cache[i];
    for (int j = i + 1; j < bodies_cache.size(); j++) {
      auto bj = bodies_cache[j];
      if (bi->inv_mass == 0.0f && bj->inv_mass == 0.0f)
        continue;
      arbiter new_arb(bi, bj);
      arbiter_key key(bi, bj);
      if (new_arb.contacts.size() > 0) {
        auto iter = arbiters.find(key);
        if (iter == arbiters.end()) {
          arbiters[key] = new_arb;
        } else {
          iter->second.update(new_arb.contacts);
        }
      } else {
        arbiters.erase(key);
      }
    }
  }
}

void sim_sys_2d::step(entt::registry &registry, float dt) {
  float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;
  bodies_cache.clear();
  registry.view<body>().each([&](entt::entity entity, body &body_comp) {
    bodies_cache.emplace_back(&body_comp);
  });
  // determine overlapping bodies and update contact points
  broadphase(registry);

  // integrate forces
  for (int i = 0; i < bodies_cache.size(); i++) {
    auto b = bodies_cache[i];
    if (b->inv_mass == 0.0f)
      continue;

    b->linear_velocity += dt * (gravity + b->inv_mass * b->force);
    b->angular_velocity += dt * b->inv_I * b->torque;
  }

  // perform pre-steps
  for (auto arb = arbiters.begin(); arb != arbiters.end(); arb++) {
    arb->second.prestep(inv_dt);
  }

  // perform iterations
  for (int i = 0; i < num_sub_steps; i++) {
    for (auto arb = arbiters.begin(); arb != arbiters.end(); arb++) {
      arb->second.apply_impulse();
    }
  }

  // integrate velocities
  for (int i = 0; i < bodies_cache.size(); i++) {
    auto b = bodies_cache[i];
    b->position += dt * b->linear_velocity;
    b->rotation += dt * b->angular_velocity;
    b->force = math::vector2::Zero();
    b->torque = 0.0f;
  }
}

void sim_sys_2d::draw_gui(entt::registry &registry, entt::entity entity) {}

void sim_sys_2d::draw_menu_gui() {
  ImGui::Checkbox("Fixed Time Step", &fixed_timestep);
  if (ImGui::InputInt("Num Sub Steps", &num_sub_steps)) {
    num_sub_steps = std::clamp(num_sub_steps, 1, 100);
  }
}

void sim_sys_2d::update(entt::registry &registry, float dt) {
  float fixed_interval = 1.0f / sim_fps;
  if (fixed_timestep) {
    float residual = cur_time - cur_exec_fixed * fixed_interval;
    while (residual > fixed_interval) {
      residual -= fixed_interval;
      step(registry, fixed_interval);
      cur_exec_fixed += 1;
    }
  } else {
    step(registry, dt);
    cur_exec_fixed = cur_time / fixed_interval;
  }
  cur_time += dt;
}

}; // namespace toolkit::sdl2d