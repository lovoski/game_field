#pragma once

#include <string>

extern std::string gbuffer_geometry_pass_vs;
extern std::string gbuffer_geometry_pass_fs;
extern std::string gbuffer_geometry_pass_gs;

extern std::string defered_default_pass_fs;

extern std::string shadow_vs;
extern std::string shadow_fs;

extern std::string quad_vs;
extern std::string static_mesh_light_mask_fs;
extern std::string shadow_mask_fs;
extern std::string csm_selection_mask_fs;

// https://github.com/orangeduck/GenoViewPython/blob/main/resources/fxaa.fs
extern std::string fxaa_fs;
