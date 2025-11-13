#include "toolkit/sim/components/algo.hpp"

namespace toolkit::sim {

math::vector3 support_point_of_minkowski_difference(base_collider *c1,
                                                    base_collider *c2,
                                                    math::vector3 direction) {
  auto support1 = c1->get_support(direction);
  auto support2 = c2->get_support(-direction);
  return support1 - support2;
}

void add_to_simplex(gjk_simplex &simplex, math::vector3 point) {
  switch (simplex.num) {
  case 1: {
    simplex.b = simplex.a;
    simplex.a = point;
  } break;
  case 2: {
    simplex.c = simplex.b;
    simplex.b = simplex.a;
    simplex.a = point;
  } break;
  case 3: {
    simplex.d = simplex.c;
    simplex.c = simplex.b;
    simplex.b = simplex.a;
    simplex.a = point;
  } break;
  default: {
    assert(0);
  } break;
  }
  ++simplex.num;
}

math::vector3 triple_cross(math::vector3 &a, math::vector3 &b,
                           math::vector3 &c) {
  return (a.cross(b)).cross(c);
}

bool do_simplex_2(gjk_simplex &simplex, math::vector3 &direction) {
  math::vector3 a = simplex.a; // the last point added
  math::vector3 b = simplex.b;
  math::vector3 ao = -a;
  math::vector3 ab = b - a;
  if (ab.dot(ao) >= 0.0) {
    simplex.a = a;
    simplex.b = b;
    simplex.num = 2;
    direction = triple_cross(ab, ao, ab);
  } else {
    simplex.a = a;
    simplex.num = 1;
    direction = ao;
  }
  return false;
}

bool do_simplex_3(gjk_simplex &simplex, math::vector3 &direction) {
  math::vector3 a = simplex.a; // the last point added
  math::vector3 b = simplex.b;
  math::vector3 c = simplex.c;
  math::vector3 ao = -a;
  math::vector3 ab = b - a;
  math::vector3 ac = c - a;
  math::vector3 abc = ab.cross(ac);
  if ((abc.cross(ac)).dot(ao) >= 0.0) {
    if (ac.dot(ao) >= 0.0) {
      // AC region
      simplex.a = a;
      simplex.b = c;
      simplex.num = 2;
      direction = triple_cross(ac, ao, ac);
    } else {
      if (ab.dot(ao) >= 0.0) {
        // AB region
        simplex.a = a;
        simplex.b = b;
        simplex.num = 2;
        direction = triple_cross(ab, ao, ab);
      } else {
        // A region
        simplex.a = a;
        direction = ao;
      }
    }
  } else {
    if ((ab.cross(abc)).dot(ao) >= 0.0) {
      if (ab.dot(ao) >= 0.0) {
        // AB region
        simplex.a = a;
        simplex.b = b;
        simplex.num = 2;
        direction = triple_cross(ab, ao, ab);
      } else {
        // A region
        simplex.a = a;
        direction = ao;
      }
    } else {
      if (abc.dot(ao) >= 0.0) {
        // ABC region ("up")
        simplex.a = a;
        simplex.b = b;
        simplex.c = c;
        simplex.num = 3;
        direction = abc;
      } else {
        // ABC region ("down")
        simplex.a = a;
        simplex.b = c;
        simplex.c = b;
        simplex.num = 3;
        direction = -abc;
      }
    }
  }
  return false;
}

bool do_simplex_4(gjk_simplex &simplex, math::vector3 &direction) {
  math::vector3 a = simplex.a; // the last point added
  math::vector3 b = simplex.b;
  math::vector3 c = simplex.c;
  math::vector3 d = simplex.d;
  math::vector3 ao = -a;
  math::vector3 ab = b - a;
  math::vector3 ac = c - a;
  math::vector3 ad = d - a;
  math::vector3 abc = ab.cross(ac);
  math::vector3 acd = ac.cross(ad);
  math::vector3 adb = ad.cross(ab);
  unsigned char plane_information = 0x0;
  if (abc.dot(ao) >= 0.0) {
    plane_information |= 0x1;
  }
  if (acd.dot(ao) >= 0.0) {
    plane_information |= 0x2;
  }
  if (adb.dot(ao) >= 0.0) {
    plane_information |= 0x4;
  }
  switch (plane_information) {
  case 0x0: {
    // Intersection
    return true;
  } break;
  case 0x1: {
    // Triangle ABC
    if ((abc.cross(ac)).dot(ao) >= 0.0) {
      if (ac.dot(ao) >= 0.0) {
        // AC region
        simplex.a = a;
        simplex.b = c;
        simplex.num = 2;
        direction = triple_cross(ac, ao, ac);
      } else {
        if (ab.dot(ao) >= 0.0) {
          // AB region
          simplex.a = a;
          simplex.b = b;
          simplex.num = 2;
          direction = triple_cross(ab, ao, ab);
        } else {
          // A region
          simplex.a = a;
          direction = ao;
        }
      }
    } else {
      if ((ab.cross(abc)).dot(ao) >= 0.0) {
        if (ab.dot(ao) >= 0.0) {
          // AB region
          simplex.a = a;
          simplex.b = b;
          simplex.num = 2;
          direction = triple_cross(ab, ao, ab);
        } else {
          // A region
          simplex.a = a;
          direction = ao;
        }
      } else {
        // ABC region
        simplex.a = a;
        simplex.b = b;
        simplex.c = c;
        simplex.num = 3;
        direction = abc;
      }
    }
  } break;
  case 0x2: {
    // Triangle ACD
    if ((acd.cross(ad)).dot(ao) >= 0.0) {
      if (ad.dot(ao) >= 0.0) {
        // AD region
        simplex.a = a;
        simplex.b = d;
        simplex.num = 2;
        direction = triple_cross(ad, ao, ad);
      } else {
        if (ac.dot(ao) >= 0.0) {
          // AC region
          simplex.a = a;
          simplex.b = c;
          simplex.num = 2;
          direction = triple_cross(ab, ao, ab);
        } else {
          // A region
          simplex.a = a;
          direction = ao;
        }
      }
    } else {
      if ((ac.cross(acd)).dot(ao) >= 0.0) {
        if (ac.dot(ao) >= 0.0) {
          // AC region
          simplex.a = a;
          simplex.b = c;
          simplex.num = 2;
          direction = triple_cross(ac, ao, ac);
        } else {
          // A region
          simplex.a = a;
          direction = ao;
        }
      } else {
        // ACD region
        simplex.a = a;
        simplex.b = c;
        simplex.c = d;
        simplex.num = 3;
        direction = acd;
      }
    }
  } break;
  case 0x3: {
    // Line AC
    if (ac.dot(ao) >= 0.0) {
      simplex.a = a;
      simplex.b = c;
      simplex.num = 2;
      direction = triple_cross(ac, ao, ac);
    } else {
      simplex.a = a;
      simplex.num = 1;
      direction = ao;
    }

  } break;
  case 0x4: {
    // Triangle ADB
    if ((adb.cross(ab)).dot(ao) >= 0.0) {
      if (ab.dot(ao) >= 0.0) {
        // AB region
        simplex.a = a;
        simplex.b = b;
        simplex.num = 2;
        direction = triple_cross(ab, ao, ab);
      } else {
        if (ad.dot(ao) >= 0.0) {
          // AD region
          simplex.a = a;
          simplex.b = d;
          simplex.num = 2;
          direction = triple_cross(ad, ao, ad);
        } else {
          // A region
          simplex.a = a;
          direction = ao;
        }
      }
    } else {
      if ((ad.cross(adb)).dot(ao) >= 0.0) {
        if (ad.dot(ao) >= 0.0) {
          // AD region
          simplex.a = a;
          simplex.b = d;
          simplex.num = 2;
          direction = triple_cross(ad, ao, ad);
        } else {
          // A region
          simplex.a = a;
          direction = ao;
        }
      } else {
        // ADB region
        simplex.a = a;
        simplex.b = d;
        simplex.c = b;
        simplex.num = 3;
        direction = adb;
      }
    }
  } break;
  case 0x5: {
    // Line AB
    if (ab.dot(ao) >= 0.0) {
      simplex.a = a;
      simplex.b = b;
      simplex.num = 2;
      direction = triple_cross(ab, ao, ab);
    } else {
      simplex.a = a;
      simplex.num = 1;
      direction = ao;
    }
  } break;
  case 0x6: {
    // Line AD
    if (ad.dot(ao) >= 0.0) {
      simplex.a = a;
      simplex.b = d;
      simplex.num = 2;
      direction = triple_cross(ad, ao, ad);
    } else {
      simplex.a = a;
      simplex.num = 1;
      direction = ao;
    }
  } break;
  case 0x7: {
    // Point A
    simplex.a = a;
    simplex.num = 1;
    direction = ao;
  } break;
  }
  return false;
}

bool do_simplex(gjk_simplex &simplex, math::vector3 &direction) {
  switch (simplex.num) {
  case 2:
    return do_simplex_2(simplex, direction);
  case 3:
    return do_simplex_3(simplex, direction);
  case 4:
    return do_simplex_4(simplex, direction);
  }
  assert(0);
  return false;
}

bool gjk_collides(base_collider *c1, base_collider *c2, gjk_simplex &simplex) {
  gjk_simplex tmp_simplex;
  tmp_simplex.a = support_point_of_minkowski_difference(
      c1, c2, math::vector3(0.0f, 0.0f, 1.0f));
  tmp_simplex.num = 1;
  math::vector3 direction = -1.0f * tmp_simplex.a;
  for (int i = 0; i < 100; i++) {
    auto next_point = support_point_of_minkowski_difference(c1, c2, direction);
    if (next_point.dot(direction) < 1.0f) {
      // no intersection
      return false;
    }
    add_to_simplex(tmp_simplex, next_point);
    if (do_simplex(tmp_simplex, direction)) {
      // intersection
      simplex = tmp_simplex;
      return true;
    }
  }
  // didn't converge
  spdlog::error("GJK did not converge");
  return false;
}

}; // namespace toolkit::sim