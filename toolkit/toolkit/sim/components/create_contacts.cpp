#include "toolkit/sim/components/colliders.hpp"

namespace toolkit::sim {

math::vector3 closest_point_on_segment(const math::vector3 &p,
                                       const math::vector3 &a,
                                       const math::vector3 &b) {
  math::vector3 ab = b - a;
  float ab_sq_len = ab.squaredNorm();
  if (ab_sq_len < 1e-6f)
    return a;
  float t = (p - a).dot(ab) / ab_sq_len;
  t = std::clamp(t, 0.0f, 1.0f);
  return a + ab * t;
}

bool sphere_sphere_contact(sphere_collider *c1, sphere_collider *c2,
                           collider_contact &contact) {
  math::vector3 dist = c2->world_pos - c1->world_pos;
  float dist_sqd = dist.squaredNorm();
  float min_dist = c1->radius + c2->radius;
  if (dist_sqd < (min_dist * min_dist)) {
    contact.normal = dist.normalized();
    float penetration = min_dist - dist.norm();
    contact.contact_point1 = c1->world_pos + contact.normal * c1->radius;
    contact.contact_point2 = c2->world_pos - contact.normal * c2->radius;
    return true;
  } else
    return false;
}

bool capsule_capsule_contact(capsule_collider *c1, capsule_collider *c2,
                             collider_contact &contact) {
  // Get segment endpoints for capsule 1
  float c1_half_len = c1->cap_distance / 2.0f;
  math::vector3 c1_a = c1->world_pos - c1->world_dir * c1_half_len;
  math::vector3 c1_b = c1->world_pos + c1->world_dir * c1_half_len;
  math::vector3 d1 = c1_b - c1_a; // Direction vector of segment 1
  // Get segment endpoints for capsule 2
  float c2_half_len = c2->cap_distance / 2.0f;
  math::vector3 c2_a = c2->world_pos - c2->world_dir * c2_half_len;
  math::vector3 c2_b = c2->world_pos + c2->world_dir * c2_half_len;
  math::vector3 d2 = c2_b - c2_a; // Direction vector of segment 2
  // Vector from start of segment 2 to start of segment 1
  math::vector3 r = c1_a - c2_a;
  // Pre-calculate dot products
  float a = d1.squaredNorm(); // a = d1.d1
  float e = d2.squaredNorm(); // e = d2.d2
  float f = d2.dot(r);
  float b = d1.dot(d2); // b = d1.d2
  float s = 0.0f, t = 0.0f;
  float denom = a * e - b * b;
  // Calculate s (parameter for segment 1)
  if (denom > 1e-6f) {
    // Segments are not parallel
    s = std::clamp((b * f - d1.dot(r) * e) / denom, 0.0f, 1.0f);
  } else {
    // Segments are parallel, just pick s=0
    s = 0.0f;
  }
  // Calculate t (parameter for segment 2) using the clamped s
  // t = (b*s + f) / e
  float t_num = b * s + f;
  if (e < 1e-6f) {
    // Segment 2 is a point
    t = 0.0f;
  } else {
    t = std::clamp(t_num / e, 0.0f, 1.0f);
  }
  // We now have t, but s was calculated assuming t was optimal.
  // Recalculate s based on the clamped t.
  // s = (b*t - d1.dot(r)) / a
  float s_num = b * t - d1.dot(r);
  if (a < 1e-6f) {
    // Segment 1 is a point
    s = 0.0f;
  } else {
    s = std::clamp(s_num / a, 0.0f, 1.0f);
  }
  // Get the two closest points on the segments
  math::vector3 closest_c1 = c1_a + d1 * s;
  math::vector3 closest_c2 = c2_a + d2 * t;
  // Check for collision
  math::vector3 dist_vec = closest_c2 - closest_c1;
  float dist_sqd = dist_vec.squaredNorm();
  float min_dist = c1->cap_radius + c2->cap_radius;
  if (dist_sqd < (min_dist * min_dist)) {
    float distance = sqrt(dist_sqd);
    if (distance < 1e-6f) {
      // Segments are touching/overlapping, hard to find a good normal.
      // Pick a normal perpendicular to one of the capsules.
      contact.normal =
          c1->world_dir.cross(math::vector3(1.0f, 0.0f, 0.0f)).normalized();
      if (contact.normal.squaredNorm() < 0.1f) { // If parallel to X-axis
        contact.normal =
            c1->world_dir.cross(math::vector3(0.0f, 1.0f, 0.0f)).normalized();
      }
    } else {
      contact.normal = dist_vec / distance;
    }
    contact.contact_point1 = closest_c1 + contact.normal * c1->cap_radius;
    contact.contact_point2 = closest_c2 - contact.normal * c2->cap_radius;
    return true;
  }
  return false;
}

bool sphere_capsule_contact(sphere_collider *c1, capsule_collider *c2,
                            collider_contact &contact) {
  // Find the endpoints of the capsule's central line segment
  float cap_half_len = c2->cap_distance / 2.0f;
  math::vector3 cap_a = c2->world_pos - c2->world_dir * cap_half_len;
  math::vector3 cap_b = c2->world_pos + c2->world_dir * cap_half_len;
  // Find the closest point on the capsule's segment to the sphere's center
  math::vector3 sphere_center = c1->world_pos;
  math::vector3 closest_point_on_cap_seg =
      closest_point_on_segment(sphere_center, cap_a, cap_b);
  // Check the distance between the sphere center and this closest point
  math::vector3 dist_vec = closest_point_on_cap_seg - sphere_center;
  float dist_sqd = dist_vec.squaredNorm();
  float min_dist = c1->radius + c2->cap_radius;
  if (dist_sqd < (min_dist * min_dist)) {
    // Collision detected
    float distance = sqrt(dist_sqd);
    if (distance < 1e-6f) {
      // Sphere center is on the capsule segment. This is deep penetration.
      // We need to pick a valid normal. We can't get one from dist_vec.
      // Let's use a normal perpendicular to the capsule's direction.
      contact.normal =
          c2->world_dir.cross(math::vector3(1.0f, 0.0f, 0.0f)).normalized();
      if (contact.normal.squaredNorm() < 0.1f) { // If parallel to X-axis
        contact.normal =
            c2->world_dir.cross(math::vector3(0.0f, 1.0f, 0.0f)).normalized();
      }
    } else {
      // Normal points from sphere center (c1) to capsule segment (c2)
      contact.normal = dist_vec.normalized();
    }
    // Following the convention from sphere_sphere_contact
    contact.contact_point1 = sphere_center + contact.normal * c1->radius;
    contact.contact_point2 =
        closest_point_on_cap_seg - contact.normal * c2->cap_radius;
    return true;
  }
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