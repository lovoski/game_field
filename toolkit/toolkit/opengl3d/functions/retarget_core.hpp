// Motion retargeter — kinematic core.
//
// SCOPE
//   Algorithm only. No engine / ECS / SDL / OpenGL deps. Eigen + STL only.
//   Reflects the staged pipeline in `retarget/GUIDE.md` 1:1 so a reader can
//   pair every function below with one section of the document.
//
// CONVENTIONS (locked on day one — do NOT mix)
//   * Quaternions ........... wxyz (Eigen::Quaternion default), w >= 0.
//   * Up axis ............... +Y up (matches BVH / this repository).
//   * Length unit ........... meters.
//   * Angle unit ............ radians.
//   * DOF parameterization .. body-frame velocity for root translation,
//                              joint-LOCAL angular velocity for every joint
//                              (root included).
//   * Retraction ............ p_root <- p_root + R_world[0] * v_body * dt
//                              R_local[j] <- R_local[j] * exp(omega_j * dt)
//   * Task error & Jacobian . expressed in WORLD frame (simpler to reason
//                              about; documented exception to GUIDE 3.5).
//   * Skeleton layout ....... parent index < child index (BVH guarantee).
//
// PIPELINE  (see GUIDE Part 2)
//   stage0_warp(...)        morphology warp on source skeleton
//   stage1_detect_contact(...)  foot-contact mask + 5-frame median
//   stage2_run_ik_table(...)    damped LM, two-stage scheduling
//   stage3_temporal_smooth(...) optional velocity regularization

#pragma once

#include "toolkit/math.hpp"

#include <map>
#include <string>
#include <vector>

namespace toolkit::retarget {

// ─────────────────────────────────────────────────────────────────────────
// Skeleton + pose. Used identically for source (driver) and target (driven).
// ─────────────────────────────────────────────────────────────────────────

struct skeleton {
  std::vector<std::string> names;     // joint name, indexed [0..N)
  std::vector<int> parents;           // parents[0] == -1, root.
  std::vector<math::vector3> offsets; // local rest offset to parent (meters)

  int num_joints() const { return (int)names.size(); }
  int find(const std::string &name) const; // returns -1 if not found
};

struct pose {
  math::vector3 root_pos = math::vector3::Zero();
  std::vector<math::quat> local_rot; // size == skeleton.num_joints()

  static pose rest(const skeleton &s);
};

// World-space FK. Output vectors are resized to num_joints().
void fk(const skeleton &s, const pose &p,
        std::vector<math::vector3> &world_pos,
        std::vector<math::quat> &world_rot);

// ─────────────────────────────────────────────────────────────────────────
// Stage-0 outputs: per-bone world SE(3), keyed by SOURCE bone name.
// This is what an IK task reads from.
// ─────────────────────────────────────────────────────────────────────────

struct source_bone_xform {
  math::vector3 pos;
  math::quat rot;
};
using source_frame = std::map<std::string, source_bone_xform>;

// ─────────────────────────────────────────────────────────────────────────
// Configuration (GMR-style JSON schema, see GUIDE Part 4).
// ─────────────────────────────────────────────────────────────────────────

// One row of an IK match table.
struct ik_task {
  std::string target_link;   // joint name in TARGET skeleton
  std::string source_bone;   // joint name in SOURCE skeleton
  float w_pos = 100.0f;      // position weight
  float w_rot = 10.0f;       // rotation weight
  math::vector3 pos_off = math::vector3::Zero(); // SE(3) offset, in source bone local
  math::quat rot_off = math::quat::Identity();   // wxyz, multiplied on the right

  // Resolved at config-load time. -1 means "skip this task".
  int target_idx = -1;
};

struct config {
  std::string source_root_name = "pelvis";
  std::string target_root_name = "pelvis";

  float ground_height = 0.0f;
  float source_height_assumption = 1.8f;
  float source_actual_height = 1.8f;

  // Per source-bone scale, applied in root-local frame (GUIDE 3.2).
  // Scaled at runtime by (source_actual_height / source_height_assumption).
  std::map<std::string, float> source_scale_table;

  // Two-stage IK scheduling (GUIDE Part 2, stage 2):
  //   stage A = lower body: root + feet (lock during contact) + pelvis/knee.
  //   stage B = upper body: spine, shoulders, elbows, wrists, head.
  // Order matters: refining arms first pulls the torso off contacts.
  std::vector<ik_task> tasks_lower;
  std::vector<ik_task> tasks_upper;

  // Optional foot bones used for contact detection (must already appear
  // in tasks_lower as well). Empty -> contact detection disabled.
  std::vector<std::string> foot_bones; // source-bone names

  // LM parameters (GUIDE 3.5).
  int   max_iters       = 10;
  float convergence_eps = 1e-3f;
  float lm_damping      = 0.5f;   // lambda
  float lm_mu_scale     = 1.0f;   // mu = mu_scale * ||e||^2

  // Velocity regularization (GUIDE 3.7 / Part 7). 0 disables.
  float vel_reg_beta    = 0.01f;

  // Pseudo-time-step inside the IK loop. Independent from frame dt.
  float ik_dt           = 1.0f;

  // Foot contact detection (GUIDE 3.7).
  float contact_vel_thresh = 0.3f; // m/s
  float contact_z_thresh   = 0.05f; // meters above ground
};

// Resolves task target_link names against the target skeleton (sets target_idx)
// and bakes the height-ratio multiplier into source_scale_table.
void prepare_config(config &cfg, const skeleton &target);

// ─────────────────────────────────────────────────────────────────────────
// STAGE 0 — morphology warp.
// Input  : raw source skeleton + pose.
// Output : per-bone world SE(3), with per-bone scale applied in ROOT-local
//          frame (preserves angles, fixes reach). See GUIDE 3.2 / 3.3.
// ─────────────────────────────────────────────────────────────────────────

source_frame stage0_warp(const skeleton &src, const pose &src_pose,
                         const config &cfg);

// ─────────────────────────────────────────────────────────────────────────
// STAGE 1 — foot-contact detection (per foot, per frame).
//   contact_t = (||v_foot|| < vel_thresh) AND (z_foot < ground + z_thresh)
// followed by a 5-frame median filter (GUIDE 3.7).
// ─────────────────────────────────────────────────────────────────────────

std::vector<bool> stage1_detect_contact(const std::vector<math::vector3> &foot_pos,
                                        float dt, float vel_thresh,
                                        float z_thresh, float ground_height);
std::vector<bool> median_filter5(const std::vector<bool> &mask);

// ─────────────────────────────────────────────────────────────────────────
// STAGE 2 — whole-body IK (one table at a time, called twice per frame).
//
// All tasks must already have target_idx resolved (see prepare_config).
// Source frame already provides the per-task target SE(3) source: we
// compose the bone (pos, rot) with each task's (pos_off, rot_off) inside
// the loop (GUIDE 3.3).
//
// A null `prev_qdot` disables velocity regularization (first frame).
// ─────────────────────────────────────────────────────────────────────────

// Sum of weighted squared task residuals. Used as convergence monitor.
float task_residual_sq(const skeleton &tgt, const pose &q,
                       const std::vector<ik_task> &tasks,
                       const source_frame &source);

// Single damped Levenberg-Marquardt step.
// Returns q̇ ∈ R^nv (nv = 6 + 3*(N-1)). Order:
//   q̇[0:3]   = root v in root-LOCAL frame
//   q̇[3:6]   = root omega in root-LOCAL frame
//   q̇[6+3k : 9+3k] = omega for joint (k+1) in joint-LOCAL frame
Eigen::VectorXf lm_step(const skeleton &tgt, const pose &q,
                        const std::vector<ik_task> &tasks,
                        const source_frame &source, const config &cfg,
                        const Eigen::VectorXf *prev_qdot);

// Apply a q̇ step to the pose.
void retract(pose &q, const Eigen::VectorXf &qdot, float dt);

// Run the LM loop until ||e|| stops decreasing or max_iters is hit.
// Returns the last computed q̇ (for use as `prev_qdot` next frame).
Eigen::VectorXf stage2_run_ik_table(const skeleton &tgt, pose &q,
                                    const std::vector<ik_task> &tasks,
                                    const source_frame &source,
                                    const config &cfg,
                                    const Eigen::VectorXf *prev_qdot);

// ─────────────────────────────────────────────────────────────────────────
// Convenience: run both stages on a single frame.
// `q_seed` is updated in place. `prev_qdot` is updated to the last q̇.
// Returns the source frame after Stage 0 (useful for visualization).
// ─────────────────────────────────────────────────────────────────────────

source_frame retarget_frame(const skeleton &src, const pose &src_pose,
                            const skeleton &tgt, pose &q_seed,
                            const config &cfg, Eigen::VectorXf &prev_qdot);

// ─────────────────────────────────────────────────────────────────────────
// Whole-clip retarget. Applies foot lock if cfg.foot_bones is non-empty:
// during a contact span [t0, t1], the source bone target position is
// frozen to its value at t0 and the corresponding task gets w_pos *= 10.
// ─────────────────────────────────────────────────────────────────────────

std::vector<pose> retarget_clip(const skeleton &src,
                                const std::vector<pose> &src_poses,
                                float src_dt, const skeleton &tgt,
                                const pose &tgt_rest, const config &cfg);

} // namespace toolkit::retarget
