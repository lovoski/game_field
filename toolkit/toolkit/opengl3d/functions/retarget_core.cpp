#include "retarget_core.hpp"

#include <algorithm>
#include <cmath>

namespace toolkit::retarget {

// ─────────────────────────────────────────────────────────────────────────
// Small math helpers (kept local to avoid leaking conventions out).
// ─────────────────────────────────────────────────────────────────────────

static math::matrix3 skew(const math::vector3 &v) {
  math::matrix3 m;
  m <<     0.0f, -v.z(),  v.y(),
        v.z(),     0.0f, -v.x(),
       -v.y(),  v.x(),     0.0f;
  return m;
}

// log_SO(3): rotation -> 3-vec whose direction = axis, magnitude = angle.
// Wraps Eigen's AngleAxis to honor w >= 0 (no double-cover discontinuity).
static math::vector3 log_quat(math::quat q) {
  if (q.w() < 0.0f) q.coeffs() = -q.coeffs();
  math::angle_axis aa(q);
  return aa.axis() * aa.angle();
}

// exp_SO(3): 3-vec -> quaternion.
static math::quat exp_quat(const math::vector3 &v) {
  float n = v.norm();
  if (n < 1e-9f) return math::quat::Identity();
  return math::quat(math::angle_axis(n, v / n));
}

// ─────────────────────────────────────────────────────────────────────────
// skeleton / pose
// ─────────────────────────────────────────────────────────────────────────

int skeleton::find(const std::string &name) const {
  for (int i = 0; i < (int)names.size(); ++i)
    if (names[i] == name) return i;
  return -1;
}

pose pose::rest(const skeleton &s) {
  pose p;
  p.local_rot.assign(s.num_joints(), math::quat::Identity());
  p.root_pos = math::vector3::Zero();
  return p;
}

void fk(const skeleton &s, const pose &p,
        std::vector<math::vector3> &world_pos,
        std::vector<math::quat> &world_rot) {
  const int N = s.num_joints();
  world_pos.assign(N, math::vector3::Zero());
  world_rot.assign(N, math::quat::Identity());
  // parents[0] == -1 and parent index < child index (BVH guarantee).
  world_pos[0] = p.root_pos;
  world_rot[0] = p.local_rot[0];
  for (int i = 1; i < N; ++i) {
    int par = s.parents[i];
    world_rot[i] = world_rot[par] * p.local_rot[i];
    world_pos[i] = world_pos[par] + world_rot[par] * s.offsets[i];
  }
}

// Walk parent chain inclusive: returns [root, ..., k].
static std::vector<int> chain_to(const skeleton &s, int k) {
  std::vector<int> chain;
  for (int i = k; i != -1; i = s.parents[i]) chain.push_back(i);
  std::reverse(chain.begin(), chain.end());
  return chain;
}

// ─────────────────────────────────────────────────────────────────────────
// config preparation
// ─────────────────────────────────────────────────────────────────────────

void prepare_config(config &cfg, const skeleton &target) {
  auto resolve = [&](std::vector<ik_task> &tasks) {
    for (auto &t : tasks) {
      t.target_idx = target.find(t.target_link);
    }
    // Drop unresolved tasks (silently — robot may not have every link).
    tasks.erase(std::remove_if(tasks.begin(), tasks.end(),
                               [](const ik_task &t) { return t.target_idx < 0; }),
                tasks.end());
  };
  resolve(cfg.tasks_lower);
  resolve(cfg.tasks_upper);

  // Bake height ratio into per-bone scale (GUIDE 3.2). Skip if assumption
  // looks degenerate to avoid blowing up scales on bad config.
  float ratio = 1.0f;
  if (cfg.source_height_assumption > 1e-3f)
    ratio = cfg.source_actual_height / cfg.source_height_assumption;
  for (auto &kv : cfg.source_scale_table) kv.second *= ratio;
}

// ─────────────────────────────────────────────────────────────────────────
// STAGE 0 — morphology warp
// ─────────────────────────────────────────────────────────────────────────

source_frame stage0_warp(const skeleton &src, const pose &src_pose,
                         const config &cfg) {
  std::vector<math::vector3> wp;
  std::vector<math::quat>    wr;
  fk(src, src_pose, wp, wr);

  const int N = src.num_joints();
  const math::vector3 root_pos = wp[0]; // anchor for root-local scaling

  source_frame out;
  for (int i = 0; i < N; ++i) {
    auto it = cfg.source_scale_table.find(src.names[i]);
    float s = (it != cfg.source_scale_table.end()) ? it->second : 1.0f;

    source_bone_xform x;
    // Per-bone scale in ROOT-local frame: angles preserved, reach changed.
    x.pos = root_pos + s * (wp[i] - root_pos);
    x.rot = wr[i];
    out.emplace(src.names[i], x);
  }
  return out;
}

// ─────────────────────────────────────────────────────────────────────────
// STAGE 1 — foot contact
// ─────────────────────────────────────────────────────────────────────────

std::vector<bool> stage1_detect_contact(const std::vector<math::vector3> &fp,
                                        float dt, float vel_thresh,
                                        float z_thresh, float ground_height) {
  const int T = (int)fp.size();
  std::vector<bool> mask(T, false);
  for (int t = 0; t < T; ++t) {
    math::vector3 v = math::vector3::Zero();
    if (t > 0)         v = (fp[t] - fp[t - 1]) / dt;
    else if (T > 1)    v = (fp[1] - fp[0]) / dt;
    bool low_v = v.norm() < vel_thresh;
    bool low_z = (fp[t].y() - ground_height) < z_thresh; // +Y up
    mask[t] = low_v && low_z;
  }
  return mask;
}

std::vector<bool> median_filter5(const std::vector<bool> &m) {
  const int T = (int)m.size();
  std::vector<bool> o = m;
  for (int t = 2; t < T - 2; ++t) {
    int s = m[t - 2] + m[t - 1] + m[t] + m[t + 1] + m[t + 2];
    o[t] = s >= 3;
  }
  return o;
}

// ─────────────────────────────────────────────────────────────────────────
// STAGE 2 — IK helpers
// ─────────────────────────────────────────────────────────────────────────

// Build the per-task target SE(3) by composing source bone with task offset.
// (GUIDE 3.3: rot_off applied first, then pos_off rotated by NEW rotation.)
static void task_target(const ik_task &t, const source_bone_xform &src,
                        math::vector3 &p_star, math::quat &r_star) {
  r_star = (src.rot * t.rot_off).normalized();
  p_star = src.pos + r_star * t.pos_off;
}

float task_residual_sq(const skeleton &tgt, const pose &q,
                       const std::vector<ik_task> &tasks,
                       const source_frame &source) {
  std::vector<math::vector3> wp;
  std::vector<math::quat>    wr;
  fk(tgt, q, wp, wr);

  float s = 0.0f;
  for (const auto &t : tasks) {
    auto it = source.find(t.source_bone);
    if (it == source.end()) continue;
    math::vector3 p_star; math::quat r_star;
    task_target(t, it->second, p_star, r_star);
    math::vector3 e_pos = p_star - wp[t.target_idx];
    math::vector3 e_rot = log_quat(r_star * wr[t.target_idx].inverse());
    s += t.w_pos * t.w_pos * e_pos.squaredNorm()
       + t.w_rot * t.w_rot * e_rot.squaredNorm();
  }
  return s;
}

Eigen::VectorXf lm_step(const skeleton &tgt, const pose &q,
                        const std::vector<ik_task> &tasks,
                        const source_frame &source, const config &cfg,
                        const Eigen::VectorXf *prev_qdot) {
  const int N  = tgt.num_joints();
  const int nv = 6 + 3 * (N - 1);

  std::vector<math::vector3> wp;
  std::vector<math::quat>    wr;
  fk(tgt, q, wp, wr);
  std::vector<math::matrix3> R_world(N);
  for (int i = 0; i < N; ++i) R_world[i] = wr[i].toRotationMatrix();

  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(nv, nv);
  Eigen::VectorXf g = Eigen::VectorXf::Zero(nv);
  float mu = 0.0f;
  const float dt = cfg.ik_dt;

  // Column index helper. Layout:
  //   [0:3]  root v_body
  //   [3:6]  root omega_local
  //   [6+3k:9+3k] omega_local for joint k+1   (k = 0 .. N-2)
  auto root_v_col   = [](){ return 0; };
  auto root_w_col   = [](){ return 3; };
  auto joint_w_col  = [](int j){ return 6 + 3 * (j - 1); };

  // Per-task contributions.
  Eigen::MatrixXf J(6, nv);
  Eigen::VectorXf e(6);
  for (const auto &t : tasks) {
    auto it = source.find(t.source_bone);
    if (it == source.end()) continue;
    const int k = t.target_idx;
    math::vector3 p_star; math::quat r_star;
    task_target(t, it->second, p_star, r_star);

    // World-frame error (linear part on top, angular below).
    math::vector3 e_pos = p_star - wp[k];
    math::vector3 e_rot = log_quat(r_star * wr[k].inverse());
    e << e_pos, e_rot;

    // Build Jacobian column-by-column.
    J.setZero();
    // Root translation (v in root-LOCAL = root-BODY frame): linear = R0.
    J.block<3,3>(0, root_v_col()) = R_world[0];
    // Joints on chain root -> k contribute angular columns.
    for (int j : chain_to(tgt, k)) {
      int col = (j == 0) ? root_w_col() : joint_w_col(j);
      math::matrix3 Rj = R_world[j];
      // Angular: world omega = Rj * omega_local.
      J.block<3,3>(3, col) = Rj;
      // Linear: cross with arm from joint-anchor to point. Anchor = wp[j].
      J.block<3,3>(0, col) = -skew(wp[k] - wp[j]) * Rj;
    }

    // W^2 along the diagonal of a 6x6 weight matrix.
    float wp2 = t.w_pos * t.w_pos;
    float wr2 = t.w_rot * t.w_rot;
    Eigen::Matrix<float, 6, 6> W2 = Eigen::Matrix<float, 6, 6>::Zero();
    W2.block<3,3>(0,0) = wp2 * Eigen::Matrix3f::Identity();
    W2.block<3,3>(3,3) = wr2 * Eigen::Matrix3f::Identity();

    Eigen::MatrixXf JtW2 = J.transpose() * W2;
    H.noalias() += dt * dt * (JtW2 * J);
    g.noalias() += dt       * (JtW2 * e);

    // Adaptive damping (GUIDE 3.5): mu_i = mu_scale * ||e_i||^2.
    mu += cfg.lm_mu_scale * (wp2 * e_pos.squaredNorm() + wr2 * e_rot.squaredNorm());
  }

  // Velocity regularization (GUIDE Part 7).
  if (prev_qdot && cfg.vel_reg_beta > 0.0f && prev_qdot->size() == nv) {
    H.diagonal().array() += cfg.vel_reg_beta;
    g.noalias()         += cfg.vel_reg_beta * (*prev_qdot);
  }

  // Final damped LM solve. cwiseSqrt avoids huge mu blowing up the matrix.
  H.diagonal().array() += cfg.lm_damping + mu;
  return H.ldlt().solve(g);
}

void retract(pose &q, const Eigen::VectorXf &qdot, float dt) {
  const int N = (int)q.local_rot.size();
  // Root.
  math::vector3 v_body  (qdot[0], qdot[1], qdot[2]);
  math::vector3 w_root  (qdot[3], qdot[4], qdot[5]);
  math::matrix3 R0      = q.local_rot[0].toRotationMatrix();
  q.root_pos += R0 * v_body * dt;
  q.local_rot[0] = (q.local_rot[0] * exp_quat(w_root * dt)).normalized();
  if (q.local_rot[0].w() < 0.0f) q.local_rot[0].coeffs() *= -1.0f;
  // Joints.
  for (int j = 1; j < N; ++j) {
    int c = 6 + 3 * (j - 1);
    math::vector3 wj(qdot[c], qdot[c + 1], qdot[c + 2]);
    q.local_rot[j] = (q.local_rot[j] * exp_quat(wj * dt)).normalized();
    if (q.local_rot[j].w() < 0.0f) q.local_rot[j].coeffs() *= -1.0f;
  }
}

Eigen::VectorXf stage2_run_ik_table(const skeleton &tgt, pose &q,
                                    const std::vector<ik_task> &tasks,
                                    const source_frame &source,
                                    const config &cfg,
                                    const Eigen::VectorXf *prev_qdot) {
  Eigen::VectorXf qdot;
  float prev_e = task_residual_sq(tgt, q, tasks, source);
  for (int it = 0; it < cfg.max_iters; ++it) {
    qdot = lm_step(tgt, q, tasks, source, cfg, prev_qdot);
    retract(q, qdot, cfg.ik_dt);
    float e = task_residual_sq(tgt, q, tasks, source);
    if (prev_e - e < cfg.convergence_eps) break;
    prev_e = e;
  }
  return qdot;
}

// ─────────────────────────────────────────────────────────────────────────
// Per-frame top-level
// ─────────────────────────────────────────────────────────────────────────

source_frame retarget_frame(const skeleton &src, const pose &src_pose,
                            const skeleton &tgt, pose &q_seed,
                            const config &cfg, Eigen::VectorXf &prev_qdot) {
  source_frame sf = stage0_warp(src, src_pose, cfg);
  // Stage A: lower body first (contacts win over arms).
  Eigen::VectorXf qd = stage2_run_ik_table(
      tgt, q_seed, cfg.tasks_lower, sf, cfg,
      prev_qdot.size() ? &prev_qdot : nullptr);
  // Stage B: layer the upper body on top.
  qd = stage2_run_ik_table(
      tgt, q_seed, cfg.tasks_upper, sf, cfg,
      prev_qdot.size() ? &prev_qdot : nullptr);
  prev_qdot = qd;
  return sf;
}

// ─────────────────────────────────────────────────────────────────────────
// Clip-level: foot lock + median filter on top of per-frame retargeting.
// ─────────────────────────────────────────────────────────────────────────

std::vector<pose> retarget_clip(const skeleton &src,
                                const std::vector<pose> &src_poses,
                                float src_dt, const skeleton &tgt,
                                const pose &tgt_rest, const config &cfg) {
  const int T = (int)src_poses.size();
  std::vector<pose> out(T, tgt_rest);
  if (T == 0) return out;

  // Pre-compute per-foot contact masks on the SOURCE side. We trace each
  // foot's world position through src FK across the whole clip first; then
  // run the IK with the masks consulted per frame.
  struct foot_track {
    int src_idx;
    std::vector<math::vector3> pos_traj;
    std::vector<bool> contact;
  };
  std::vector<foot_track> tracks;
  for (const auto &name : cfg.foot_bones) {
    int idx = src.find(name);
    if (idx < 0) continue;
    tracks.push_back({idx, std::vector<math::vector3>(T), std::vector<bool>(T, false)});
  }
  if (!tracks.empty()) {
    std::vector<math::vector3> wp; std::vector<math::quat> wr;
    for (int t = 0; t < T; ++t) {
      fk(src, src_poses[t], wp, wr);
      for (auto &tr : tracks) tr.pos_traj[t] = wp[tr.src_idx];
    }
    for (auto &tr : tracks) {
      tr.contact = stage1_detect_contact(tr.pos_traj, src_dt,
                                         cfg.contact_vel_thresh,
                                         cfg.contact_z_thresh,
                                         cfg.ground_height);
      tr.contact = median_filter5(tr.contact);
    }
  }

  // Walk the clip. For each frame, swap in foot-locked source positions and
  // boost the corresponding task's w_pos when locked.
  Eigen::VectorXf prev_qdot;
  std::vector<int> contact_t0(tracks.size(), -1); // start of current contact
  for (int t = 0; t < T; ++t) {
    pose src_p = src_poses[t];
    // Foot lock: replace per-foot world target by t0's value within the span.
    // We can't edit src_poses (used for FK), but stage0 reads from src
    // directly. Instead, we override the source_frame after stage0_warp.
    source_frame sf = stage0_warp(src, src_p, cfg);

    // Make a working copy of the lower-body tasks so we can boost weights.
    std::vector<ik_task> lower = cfg.tasks_lower;
    for (size_t k = 0; k < tracks.size(); ++k) {
      auto &tr = tracks[k];
      if (tr.contact[t]) {
        if (contact_t0[k] < 0) contact_t0[k] = t;
        const auto &name = src.names[tr.src_idx];
        // Freeze world target position for the foot bone.
        auto it = sf.find(name);
        if (it != sf.end()) it->second.pos = tr.pos_traj[contact_t0[k]];
        // Boost w_pos for any lower-body task hitting that bone.
        for (auto &task : lower)
          if (task.source_bone == name) task.w_pos *= 10.0f;
      } else {
        contact_t0[k] = -1;
      }
    }

    // Stage A then Stage B — exactly retarget_frame() but with the locked sf.
    const Eigen::VectorXf *prev = prev_qdot.size() ? &prev_qdot : nullptr;
    Eigen::VectorXf qd = stage2_run_ik_table(tgt, out[t], lower, sf, cfg, prev);
    qd                = stage2_run_ik_table(tgt, out[t], cfg.tasks_upper, sf, cfg, prev);
    prev_qdot         = qd;
    if (t + 1 < T) out[t + 1] = out[t]; // warm-start next frame
  }
  return out;
}

} // namespace toolkit::retarget
