#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

namespace toolkit::bullet {

struct sphere_collider {
  float radius;
};
struct capsule_collider {
  float height, radius;
};
struct convex_hull_collider {
  std::vector<math::vector3> vertices;
};

struct rigid_body_component {
  btRigidBody *rigid_body = nullptr;
  btCollisionShape *collision_shape = nullptr;
  float mass = 0.0f;

  // For easy access to transform
  math::vector3 position() const;
  math::quat rotation() const;
  ~rigid_body_component();
};

}; // namespace toolkit::bullet