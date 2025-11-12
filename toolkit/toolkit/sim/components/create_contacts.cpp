#include "toolkit/sim/components/colliders.hpp"

namespace toolkit::sim {

bool sphere_sphere_contact(sphere_collider *c1, sphere_collider *c2,
                           collider_contact &contact) {
  math::vector3 dist = c1->world_pos - c2->world_pos;
  float dist_sqd = dist.squaredNorm();
  float min_dist = c1->radius + c2->radius;
  if (dist_sqd < (min_dist * min_dist)) {
    contact.normal = (c2->world_pos - c1->world_pos).normalized();
    float penetration = min_dist - dist.norm();
    contact.contact_point1 = c1->world_pos + contact.normal * c1->radius;
    contact.contact_point2 = c2->world_pos - contact.normal * c2->radius;
    return true;
  } else
    return false;
}

bool capsule_capsule_contact(capsule_collider *c1, capsule_collider *c2,
                             collider_contact &contact) {
  return false;
}

bool sphere_capsule_contact(sphere_collider *c1, capsule_collider *c2,
                            collider_contact &contact) {
  return false;
}

std::vector<collider_contact> colliders_get_contacts(base_collider *c1,
                                                     base_collider *c2) {
  std::vector<collider_contact> results;
  collider_contact contact;
  // calculate everything analytically
  if (c1->type == collider_type::SPHERE && c2->type == collider_type::SPHERE) {
    if (sphere_sphere_contact(dynamic_cast<sphere_collider *>(c1),
                              dynamic_cast<sphere_collider *>(c2), contact))
      results.push_back(contact);
    return results;
  } else if (c1->type == collider_type::SPHERE &&
             c2->type == collider_type::CAPSULE) {
    if (sphere_capsule_contact(dynamic_cast<sphere_collider *>(c1),
                               dynamic_cast<capsule_collider *>(c2), contact))
      results.push_back(contact);
    return results;
  } else if (c1->type == collider_type::CAPSULE &&
             c2->type == collider_type::SPHERE) {
    if (sphere_capsule_contact(dynamic_cast<sphere_collider *>(c2),
                               dynamic_cast<capsule_collider *>(c1), contact))
      results.push_back(contact);
    return results;
  } else if (c1->type == collider_type::CAPSULE &&
             c2->type == collider_type::CAPSULE) {
    if (capsule_capsule_contact(dynamic_cast<capsule_collider *>(c1),
                                dynamic_cast<capsule_collider *>(c2), contact))
      results.push_back(contact);
    return results;
  }
  return results;
}

}; // namespace toolkit::sim