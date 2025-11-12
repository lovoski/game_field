#include "toolkit/opengl/scripts/smplx.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::opengl {

static const std::vector<std::string> model_type_names = {"smpl", "smplx"};
static const std::vector<std::string> gender_type_names = {"NEUTRAL", "MALE",
                                                           "FEMALE"};
static const std::vector<std::string> SMPL_JOINT_NAMES = {
    "pelvis",        "left_hip",       "right_hip",    "spine1",
    "left_knee",     "right_knee",     "spine2",       "left_ankle",
    "right_ankle",   "spine3",         "left_foot",    "right_foot",
    "neck",          "left_collar",    "right_collar", "head",
    "left_shoulder", "right_shoulder", "left_elbow",   "right_elbow",
    "left_wrist",    "right_wrist",    "left_hand",    "right_hand",
};
// static const std::vector<std::string> SMPL_JOINT_NAMES = {
//     "Hips",      "LeftUpLeg",    "RightUpLeg",    "Spine",
//     "LeftLeg",   "RightLeg",     "Spine1",        "LeftFoot",
//     "RightFoot", "Spine2",       "LeftToeBase",   "RightToeBase",
//     "Spine3",    "LeftShoulder", "RightShoulder", "Neck",
//     "LeftArm",   "RightArm",     "LeftForeArm",   "RightForeArm",
//     "LeftHand",  "RightHand",    "LeftHand_End",  "RightHand_End",
// };
static const std::vector<int> SMPL_JOINT_PARENTS = {
    -1, 0, 0, 0,  1,  2,  3,  4,  5,  6,  7,  8,
    9,  9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21};
static const std::vector<std::vector<int>> SMPL_J_REGRESSOR_INDICES = {
    {705,  785,  836,  866,  884,  940,  1212, 1449, 1540,
     2913, 2914, 3021, 3129, 3143, 3144, 3145, 4107, 4193,
     4322, 4352, 4801, 4919, 4922, 5285, 6372, 6470, 6564},
    {835, 866, 871, 1137, 1206, 3089, 3142, 4745},
    {833, 853, 4297, 4323, 4623, 4690, 4814, 4919, 4969, 6512, 6562},
    {676, 677, 889, 1336, 2913, 3022, 4288, 4290, 6545},
    {1008, 1010, 1016, 1019, 1043, 1047, 1055, 1523},
    {4495, 4500, 4505, 4532, 4533, 4535, 4536, 4541, 4634, 4994},
    {596, 1269, 1330, 2847, 3511, 4084, 4109, 4752, 6312, 6342},
    {3209, 3215, 3221, 3326, 3327, 3381, 3395, 3433, 3469},
    {6706, 6728, 6780, 6791, 6816, 6832, 6833, 6869},
    {642, 723, 736, 1197, 1337, 1349, 3014, 3037, 3500, 4130, 4225, 4828},
    {3336, 3338, 3344, 3357, 3359, 3362},
    {6715, 6736, 6745, 6752, 6759, 6762},
    {570, 573, 813, 3059, 3470, 4059, 4292, 4315},
    {588, 701, 760, 895, 1293, 1296, 1299, 2837, 2871, 2931, 2957, 3470, 3727,
     4963, 5239},
    {152, 3076, 4076, 4221, 4246, 4382, 4502, 4703, 4775, 4776, 4781, 6343,
     6429, 6465},
    {142, 172, 185, 201, 307, 3685, 3699, 3795, 3819},
    {637, 1255, 1256, 1873, 1891, 2878, 2944, 3000, 3008, 3009, 6534},
    {695, 1463, 4123, 4124, 6403, 6460, 6468},
    {1566, 1620, 1654, 1657, 1658, 1664, 1694, 1725},
    {5090, 5120, 5126, 5128, 5194, 5361},
    {2104, 2107, 2111, 2206, 2208, 2230, 2241},
    {5565, 5566, 5569, 5571, 5667, 5668, 5669},
    {1510, 1844, 2135, 2139, 2174, 2260, 2271, 2278, 2699, 2770},
    {4888, 5595, 5636, 5654, 5720, 5732, 5736, 5739, 6166, 6232}};
static const std::vector<std::vector<float>> SMPL_J_REGRESSOR_WEIGHTS = {
    {0.03420865163207054,  0.0010256130481138825, 0.017024220898747444,
     0.01890232414007187,  0.04245106130838394,   0.0038961756508797407,
     0.009360259398818016, 0.09357372671365738,   0.0547783188521862,
     0.006286854390054941, 0.060348622500896454,  0.06938552856445312,
     0.04249421879649162,  0.09061764925718307,   0.010837012901902199,
     0.03884423151612282,  0.005334244109690189,  0.044522181153297424,
     0.03817061707377434,  0.014777230098843575,  0.02017548866569996,
     0.04129422828555107,  0.10098818689584732,   1.892295949801337e-05,
     0.05323054641485214,  0.001917354529723525,  0.08553653210401535},
    {0.07919987291097641, 0.08850079029798508, 0.018555665388703346,
     0.0012232393492013216, 0.32295218110084534, 0.3520911633968353,
     0.1193920224905014, 0.018085060641169548},
    {0.008271533995866776, 0.011767112649977207, 0.03441965579986572,
     0.005048851016908884, 0.03460593894124031, 0.11860638856887817,
     0.24991610646247864, 0.09665679931640625, 0.010348271578550339,
     0.3337232768535614, 0.0966360792517662},
    {0.20182949304580688, 0.032792579382658005, 0.004854965023696423,
     0.1617196798324585, 0.1982906311750412, 0.04744437709450722,
     0.10771963745355606, 0.19581308960914612, 0.049535565078258514},
    {0.046333037316799164, 0.4297682046890259, 0.14593708515167236,
     0.017059873789548874, 0.1816549003124237, 0.007530384697020054,
     0.04485096409916878, 0.12686555087566376},
    {0.308098167181015, 0.08282994478940964, 0.0686255544424057, 0.095703125,
     0.1808198243379593, 0.019820362329483032, 0.13164851069450378,
     0.06182250380516052, 0.033467743545770645, 0.017164263874292374},
    {0.01763443648815155, 0.4233384430408478, 0.04851839691400528,
     0.01734030432999134, 0.013065936975181103, 0.06373350322246552,
     0.009267754852771759, 0.2587796747684479, 0.052845362573862076,
     0.09547620266675949},
    {0.0422021746635437, 0.006427186541259289, 0.003026788355782628,
     0.034142859280109406, 0.21185415983200073, 0.03816879168152809,
     0.08772260695695877, 0.5037899017333984, 0.07266556471586227},
    {0.017554352059960365, 0.11091770976781845, 0.028868921101093292,
     0.1510290652513504, 0.25578469038009644, 0.3177783489227295,
     0.03507423400878906, 0.08299266546964645},
    {0.21395602822303772, 0.060981303453445435, 0.048487767577171326,
     0.013061510398983955, 0.003389740129932761, 0.05008465051651001,
     0.04109308868646622, 0.07787405699491501, 0.0013069638516753912,
     0.23040707409381866, 0.05957149714231491, 0.19978632032871246},
    {0.3284965753555298, 0.037143342196941376, 0.1735321432352066,
     0.053450074046850204, 0.019565081223845482, 0.3878127932548523},
    {0.12176878750324249, 0.30136144161224365, 0.2751164436340332,
     0.042249299585819244, 0.048419658094644547, 0.21108438074588776},
    {0.07807307690382004, 0.02188274636864662, 0.17773395776748657,
     0.2547757029533386, 0.0648009330034256, 0.029973343014717102,
     0.12182540446519852, 0.2509348392486572},
    {0.16889715194702148, 0.00046294950880110264, 0.05195970833301544,
     0.3247309923171997, 0.1113436371088028, 0.24853213131427765,
     0.023036694154143333, 0.0006462882156483829, 0.002081227255985141,
     0.006520160473883152, 0.022036131471395493, 0.03331071510910988,
     0.002572303405031562, 0.002173659158870578, 0.0016962544759735465},
    {0.005889756605029106, 0.0031161587685346603, 0.09179165214300156,
     0.06179548799991608, 0.06726185232400894, 0.061326928436756134,
     0.00014729842951055616, 0.028111131861805916, 0.08105925470590591,
     0.304904043674469, 0.038707584142684937, 0.11803010106086731,
     0.08431309461593628, 0.05354566127061844},
    {0.03798815235495567, 0.29390382766723633, 0.059481181204319,
     0.046347808092832565, 0.018274998292326927, 0.22460894286632538,
     0.016742253676056862, 0.15496443212032318, 0.1476883739233017},
    {0.07228735834360123, 0.01739097386598587, 0.0033528043422847986,
     0.06756480783224106, 0.172500878572464, 0.02213909476995468,
     0.21043813228607178, 0.05194716528058052, 0.049834150820970535,
     0.32932981848716736, 0.003214817028492689},
    {0.02526649460196495, 0.0018985634669661522, 0.07650198042392731,
     0.013434802182018757, 0.09136474877595901, 0.36714085936546326,
     0.42439255118370056},
    {0.06388702988624573, 0.001318484079092741, 0.13791008293628693,
     0.35664698481559753, 0.22623685002326965, 0.11176221817731857,
     0.05803539231419563, 0.044202953577041626},
    {0.22084811329841614, 0.19964835047721863, 0.09163093566894531,
     0.07941336929798126, 0.24966385960578918, 0.15879537165164948},
    {0.38022786378860474, 0.1441940814256668, 0.08651071786880493,
     0.07735174149274826, 0.16368477046489716, 0.13764797151088715,
     0.010382830165326595},
    {0.11147531121969223, 0.0667659118771553, 0.2842082679271698,
     0.31626155972480774, 0.07217872887849808, 0.012502001598477364,
     0.13660822808742523},
    {0.0001099589208024554, 0.0006566296797245741, 0.08168461918830872,
     0.19624139368534088, 0.4259335994720459, 0.05223793163895607,
     0.04340915381908417, 0.15296950936317444, 0.00932695996016264,
     0.03743024542927742},
    {0.0007068814011290669, 0.3358944058418274, 0.023210708051919937,
     0.36257830262184143, 0.014246612787246704, 0.18791858851909637,
     0.0009372223285026848, 0.04159576818346977, 0.005636296700686216,
     0.027275199070572853}};

void smplx::start() { model_path = "D:\\0tasks\\smplx_archive\\models"; }
void smplx::init1() {}
void smplx::destroy() {}

void smplx::setup_smplx_model(entt::registry &registry, cnpy::npz_t &data) {}

template <typename T>
void fill_positions(const std::vector<T> &arr, std::vector<math::vector3> &dst,
                    int num) {
  dst.resize(num, math::vector3::Zero());
  for (int i = 0; i < num; i++) {
    dst[i] << arr[3 * i + 0], arr[3 * i + 1], arr[3 * i + 2];
  }
}

template <typename T>
void setup_vertices(mesh_data &mesh_comp, const std::vector<T> v_template,
                    int num_vertices) {
  mesh_comp.vertices.resize(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    mesh_comp.vertices[i].position << v_template[i * 3 + 0],
        v_template[i * 3 + 1], v_template[i * 3 + 2], 1.0f;
  }
}

template <typename T>
void setup_indices(mesh_data &mesh_comp, const std::vector<T> f,
                   int num_faces) {
  mesh_comp.indices.resize(num_faces * 3);
  for (int i = 0; i < 3 * num_faces; i++) {
    mesh_comp.indices[i] = f[i];
  }
  std::vector<int> accum_count(mesh_comp.vertices.size(), 0);
  for (int i = 0; i < num_faces; i++) {
    int vid0 = f[3 * i + 0], vid1 = f[3 * i + 1], vid2 = f[3 * i + 2];
    math::vector3 e01 =
        (mesh_comp.vertices[vid1].position - mesh_comp.vertices[vid0].position)
            .head<3>();
    math::vector3 e12 =
        (mesh_comp.vertices[vid2].position - mesh_comp.vertices[vid1].position)
            .head<3>();
    math::vector4 face_normal;
    face_normal << (e01.cross(e12)).normalized(), 0.0f;
    mesh_comp.vertices[vid0].normal += face_normal;
    mesh_comp.vertices[vid1].normal += face_normal;
    mesh_comp.vertices[vid2].normal += face_normal;
    accum_count[vid0]++;
    accum_count[vid1]++;
    accum_count[vid2]++;
  }
  for (int i = 0; i < mesh_comp.vertices.size(); i++)
    mesh_comp.vertices[i].normal /= accum_count[i];
}

template <typename T>
void setup_skin_weights(mesh_data &mesh_comp, const std::vector<T> weights,
                        const int num_vertices, const int num_joints) {
  for (int i = 0; i < num_vertices; i++) {
    // find top 4 weighted joints
    for (int j = 0; j < num_joints; j++) {
      if (weights[i * num_joints + j] == 0.0f)
        continue;
      float min_weight = 1e5f;
      int min_weight_idx = 0;
      for (int k = 0; k < 4; k++) {
        if (mesh_comp.vertices[i].bone_weights[k] < min_weight) {
          min_weight = mesh_comp.vertices[i].bone_weights[k];
          min_weight_idx = k;
        }
      }
      if (min_weight < weights[i * num_joints + j]) {
        mesh_comp.vertices[i].bone_weights[min_weight_idx] =
            weights[i * num_joints + j];
        mesh_comp.vertices[i].bone_indices[min_weight_idx] = j;
      }
    }
  }
  // normalize skin weights
  for (int i = 0; i < mesh_comp.vertices.size(); i++) {
    float weight_sum = 0.0f;
    for (int j = 0; j < 4; j++)
      weight_sum += mesh_comp.vertices[i].bone_weights[j];
    if (weight_sum > 0.0f)
      for (int j = 0; j < 4; j++)
        mesh_comp.vertices[i].bone_weights[j] /= weight_sum;
  }
}

template <typename T>
void setup_shapedirs(mesh_data &mesh_comp, const std::vector<T> data,
                     const int num_vertices, const int num_bs) {
  // data of shape (num_vertices, 3, num_bs)
  int num_bs_active =
      std::min(10, num_bs); // smpl by default use 10 channels only
  mesh_comp.blendshapes.resize(num_bs_active);
  for (int i = 0; i < num_bs_active; i++) {
    mesh_comp.blendshapes[i].name = str_format("shapedir%d", i);
    mesh_comp.blendshapes[i].verts.resize(num_vertices);
    for (int j = 0; j < num_vertices; j++) {
      mesh_comp.blendshapes[i].verts[j].offset_pos
          << data[j * 3 * num_bs + 0 * num_bs + i],
          data[j * 3 * num_bs + 1 * num_bs + i],
          data[j * 3 * num_bs + 2 * num_bs + i], 0.0;
      mesh_comp.blendshapes[i].verts[j].offset_normal = math::vector4::Zero();
    }
  }
}

template <typename T>
void setup_posedirs(mesh_data &mesh_comp, const std::vector<T> data,
                    const int num_vertices, const int num_bs) {
  // data of shape (num_vertices, 3, num_bs)
  // smpl by default use 207 posedirs channels
  for (int i = 0; i < num_bs; i++) {
    assets::blend_shape bs;
    bs.weight = 0.0f;
    bs.verts.resize(num_vertices);
    bs.name = str_format("posedir%d", i);
    for (int j = 0; j < num_vertices; j++) {
      bs.verts[j].offset_pos << data[j * 3 * num_bs + 0 * num_bs + i],
          data[j * 3 * num_bs + 1 * num_bs + i],
          data[j * 3 * num_bs + 2 * num_bs + i], 0.0;
      bs.verts[j].offset_normal = math::vector4::Zero();
    }
    mesh_comp.blendshapes.emplace_back(bs);
  }
}

void smplx::setup_smpl_model(entt::registry &registry, cnpy::npz_t &data) {
  num_betas = 10;
  beta_cache.resize(num_betas, 0.0f);

  auto &entity_trans = registry.get<transform>(entity);
  entity_trans.set_world_pos(math::vector3::Zero());
  entity_trans.force_update_hierarchy();

  auto &mesh_comp = registry.emplace_or_replace<mesh_data>(entity);
  mesh_comp.model_name = model_type;
  mesh_comp.mesh_name = gender_type;
  auto &bundle_data = registry.emplace_or_replace<skinned_mesh_bundle>(entity);
  bundle_data.mesh_entities.push_back(entity);

  // fill data
  auto &v_template = data["v_template"];
  auto &f = data["f"];
  if (v_template.word_size == 4)
    setup_vertices(mesh_comp, v_template.as_vec<float>(), v_template.shape[0]);
  else if (v_template.word_size == 8)
    setup_vertices(mesh_comp, v_template.as_vec<double>(), v_template.shape[0]);
  if (f.word_size == 4)
    setup_indices(mesh_comp, f.as_vec<std::uint32_t>(), f.shape[0]);
  else if (f.word_size == 8)
    setup_indices(mesh_comp, f.as_vec<std::uint64_t>(), f.shape[0]);

  auto &weights = data["weights"];
  if (weights.word_size == 4)
    setup_skin_weights(mesh_comp, weights.as_vec<float>(), weights.shape[0],
                       weights.shape[1]);
  else if (weights.word_size == 8)
    setup_skin_weights(mesh_comp, weights.as_vec<double>(), weights.shape[0],
                       weights.shape[1]);

  auto &shapedirs = data["shapedirs"];
  if (shapedirs.word_size == 4)
    setup_shapedirs(mesh_comp, shapedirs.as_vec<float>(), shapedirs.shape[0],
                    shapedirs.shape[2]);
  else if (shapedirs.word_size == 8)
    setup_shapedirs(mesh_comp, shapedirs.as_vec<double>(), shapedirs.shape[0],
                    shapedirs.shape[2]);
  auto &posedirs = data["posedirs"];
  if (posedirs.word_size == 4)
    setup_posedirs(mesh_comp, posedirs.as_vec<float>(), posedirs.shape[0],
                   posedirs.shape[2]);
  else if (posedirs.word_size == 8)
    setup_posedirs(mesh_comp, posedirs.as_vec<double>(), posedirs.shape[0],
                   posedirs.shape[2]);

  // skinned mesh related
  auto &actor_comp = registry.emplace_or_replace<anim::actor>(entity);
  // TODO: can't use emplace_or_replace on a script, could be a problem with
  // script life-cycle
  auto vis_skel_script = registry.try_get<anim::vis_skeleton>(entity);
  if (vis_skel_script == nullptr)
    registry.emplace<anim::vis_skeleton>(entity);
  actor_comp.joint_active.resize(SMPL_JOINT_NAMES.size(), true);
  for (int i = 0; i < bone_entities.size(); i++)
    registry.destroy(bone_entities[i]);
  bone_entities.clear();
  for (int i = 0; i < SMPL_JOINT_NAMES.size(); i++) {
    bone_entities.push_back(registry.create());
    auto &bone_trans = registry.emplace<transform>(bone_entities[i]);
    auto &bone_node = registry.emplace<anim::bone_node>(bone_entities[i]);
    bone_trans.name = SMPL_JOINT_NAMES[i];
    actor_comp.ordered_entities.push_back(bone_entities[i]);
    actor_comp.name_to_entity[SMPL_JOINT_NAMES[i]] = bone_entities[i];
    if (SMPL_JOINT_PARENTS[i] != -1) {
      bone_trans.set_parent(bone_entities[SMPL_JOINT_PARENTS[i]]);
    } else {
      bone_trans.set_parent(entity);
    }
  }
  bundle_data.bone_entities = bone_entities;
  apply_smpl_betas(registry, beta_cache);
  mesh_comp.update_buffers();
}

void smplx::apply_smpl_betas(entt::registry &registry, std::vector<float> betas) {
  if (betas.size() != 10) {
    spdlog::error("SMPL betas must be 10");
    return;
  }
  if (bone_entities.size() != 24) {
    spdlog::error("SMPL must have 24 joints, currently {0}",
                  bone_entities.size());
    return;
  }
  auto &root_trans = registry.get<transform>(bone_entities[0]);
  auto &ent_trans = registry.get<transform>(entity);
  auto &mesh_comp = registry.get<opengl::mesh_data>(entity);
  joint_rest_world_pos.resize(SMPL_JOINT_NAMES.size());
  for (int i = 0; i < SMPL_JOINT_NAMES.size(); i++) {
    joint_rest_world_pos[i] = math::vector3::Zero();
    for (int j = 0; j < SMPL_J_REGRESSOR_INDICES[i].size(); j++) {
      math::vector3 vert_pos =
          mesh_comp.vertices[SMPL_J_REGRESSOR_INDICES[i][j]].position.head<3>();
      for (int k = 0; k < 10; k++) {
        vert_pos += betas[k] * mesh_comp.blendshapes[k]
                                   .verts[SMPL_J_REGRESSOR_INDICES[i][j]]
                                   .offset_pos.head<3>();
      }
      joint_rest_world_pos[i] += vert_pos * SMPL_J_REGRESSOR_WEIGHTS[i][j];
    }
  }
  ent_trans.set_world_transform(math::matrix4::Identity());
  for (int i = 0; i < SMPL_JOINT_NAMES.size(); i++) {
    auto &bone_trans = registry.get<transform>(bone_entities[i]);
    bone_trans.set_local_transform(math::matrix4::Identity());
  }
  ent_trans.force_update_hierarchy();
  for (int i = 0; i < SMPL_JOINT_NAMES.size(); i++) {
    auto &bone_trans = registry.get<transform>(bone_entities[i]);
    if (SMPL_JOINT_PARENTS[i] != -1) {
      bone_trans.set_local_pos(joint_rest_world_pos[i] -
                               joint_rest_world_pos[SMPL_JOINT_PARENTS[i]]);
    } else {
      bone_trans.set_local_pos(joint_rest_world_pos[i]);
    }
  }
  ent_trans.force_update_hierarchy();
  for (int i = 0; i < bone_entities.size(); i++) {
    auto &bone_trans = registry.get<transform>(bone_entities[i]);
    auto &bone_node = registry.get<anim::bone_node>(bone_entities[i]);
    bone_node.name = bone_trans.name;
    bone_node.offset_matrix = bone_trans.matrix().inverse();
  }
  for (int i = 0; i < 10; i++) {
    for (int k = 0; k < mesh_comp.blendshapes.size(); k++) {
      if (mesh_comp.blendshapes[k].name == str_format("shapedir%d", i))
        mesh_comp.blendshapes[k].weight = betas[i];
    }
  }
}
void smplx::apply_smplx_betas(entt::registry &registry, std::vector<float> betas) {}

void smplx::preupdate(entt::registry &registry, float dt) {}

void smplx::draw_gui(entt::registry &registry, entt::entity entity) {
  ImGui::Text(str_format("model_path: %s", model_path.c_str()).c_str());
  if (ImGui::Button("Setup Model Dir", {-1, 30}))
    open_folder_dialog("Select model directory", model_path);
  gui::combo("Model Type", model_index, model_type_names,
             [&](int current) { model_type = model_type_names[current]; });
  gui::combo("Gender Type", gender_index, gender_type_names,
             [&](int current) { gender_type = gender_type_names[current]; });
  if (ImGui::Button("Load Model", {-1, 30})) {
    std::string model_filepath = join_path(
        model_path, model_type == "smplx" ? "smplx_modified" : model_type,
        str_format("%s_%s.npz", uppper_case(model_type).c_str(),
                   gender_type.c_str()));
    if (std::filesystem::exists(model_filepath)) {
      spdlog::info("Load file from {0}", model_filepath);
      smpl_data = cnpy::npz_load(model_filepath);
      if (model_type == "smplx")
        setup_smplx_model(registry, smpl_data);
      else if (model_type == "smpl")
        setup_smpl_model(registry, smpl_data);
    } else {
      spdlog::error("File {0} doesn't exist", model_filepath);
    }
  }
  for (int i = 0;
       i < (num_betas > beta_cache.size() ? beta_cache.size() : num_betas);
       i++) {
    ImGui::DragFloat(str_format("beta%d", i).c_str(), beta_cache.data() + i,
                     0.01f, -10.0f, 10.0f);
  }
  if (ImGui::Button("Apply Betas", {-1, 30}))
    if (num_betas == 10)
      apply_smpl_betas(registry, beta_cache);
}

void smplx::draw_to_scene(entt::registry &registry, transform &cam_trans,
                          camera &cam_comp) {}

}; // namespace toolkit::opengl