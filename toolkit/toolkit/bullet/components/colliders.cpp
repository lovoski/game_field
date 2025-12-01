#include "toolkit/bullet/components/colliders.hpp"

namespace toolkit::bullet {

math::vector3 rigid_body_component::position() const {
  if (!rigid_body)
    return math::vector3::Zero();
  const btTransform &transform = rigid_body->getWorldTransform();
  const btVector3 &origin = transform.getOrigin();
  return math::vector3(origin.x(), origin.y(), origin.z());
}

math::quat rigid_body_component::rotation() const {
  if (!rigid_body)
    return math::quat::Identity();
  const btTransform &transform = rigid_body->getWorldTransform();
  const btQuaternion &rotation = transform.getRotation();
  return math::quat(rotation.w(), rotation.x(), rotation.y(), rotation.z());
}

rigid_body_component::~rigid_body_component() {
  if (rigid_body) {
    delete rigid_body->getMotionState();
    delete rigid_body;
  }
  if (collision_shape) {
    delete collision_shape;
  }
}

}; // namespace toolkit::bullet