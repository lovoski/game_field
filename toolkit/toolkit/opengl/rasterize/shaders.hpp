#pragma once

#include <string>

extern std::string gbuffer_geometry_pass_vs;
extern std::string gbuffer_geometry_pass_fs;
extern std::string defered_phong_pass_vs;
extern std::string defered_phong_pass_fs;

extern std::string shadow_vs;
extern std::string shadow_fs;

extern std::string quad_vs;
extern std::string static_mesh_light_mask_fs;
extern std::string shadow_mask_fs;
extern std::string csm_selection_mask_fs;

// https://blog.simonrodriguez.fr/articles/2016/07/implementing_fxaa.html
extern std::string fxaa_fs;
