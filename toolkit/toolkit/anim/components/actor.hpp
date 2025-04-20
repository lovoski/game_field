#pragma once

#include "toolkit/system.hpp"

namespace toolkit::anim {

struct actor {};

entt::entity create_bvh_actor(entt::registry &registry, std::string filepath);

}; // namespace toolkit::anim