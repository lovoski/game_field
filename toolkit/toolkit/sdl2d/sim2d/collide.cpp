/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erin Catto makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */
#include "toolkit/sdl2d/sim2d/system.hpp"

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

using namespace toolkit::math;

namespace toolkit::sdl2d {

enum Axis { FACE_A_X, FACE_A_Y, FACE_B_X, FACE_B_Y };

enum EdgeNumbers { NO_EDGE = 0, EDGE1, EDGE2, EDGE3, EDGE4 };

struct ClipVertex {
  ClipVertex() { fp.value = 0; }
  vector2 v;
  feature_pair fp;
};

template <typename T> inline void _Swap(T &a, T &b) {
  T tmp = a;
  a = b;
  b = tmp;
}

void Flip(feature_pair &fp) {
  _Swap(fp.e.inEdge1, fp.e.inEdge2);
  _Swap(fp.e.outEdge1, fp.e.outEdge2);
}

inline vector2 abs(const vector2 &vec) {
  vector2 v = vec;
  v(0) = std::fabs(v(0));
  v(1) = std::fabs(v(1));
  return v;
}

inline matrix2 abs(const matrix2 &mat) {
  matrix2 m = mat;
  m(0, 0) = std::fabs(m(0, 0));
  m(0, 1) = std::fabs(m(0, 1));
  m(1, 0) = std::fabs(m(1, 0));
  m(1, 1) = std::fabs(m(1, 1));
  return m;
}

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
                      const vector2 &normal, float offset, char clipEdge) {
  // Start with no output points
  int numOut = 0;

  // Calculate the distance of end points to the line
  float distance0 = normal.dot(vIn[0].v) - offset;
  float distance1 = normal.dot(vIn[1].v) - offset;

  // If the points are behind the plane
  if (distance0 <= 0.0f)
    vOut[numOut++] = vIn[0];
  if (distance1 <= 0.0f)
    vOut[numOut++] = vIn[1];

  // If the points are on different sides of the plane
  if (distance0 * distance1 < 0.0f) {
    // Find intersection point of edge and plane
    float interp = distance0 / (distance0 - distance1);
    vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
    if (distance0 > 0.0f) {
      vOut[numOut].fp = vIn[0].fp;
      vOut[numOut].fp.e.inEdge1 = clipEdge;
      vOut[numOut].fp.e.inEdge2 = NO_EDGE;
    } else {
      vOut[numOut].fp = vIn[1].fp;
      vOut[numOut].fp.e.outEdge1 = clipEdge;
      vOut[numOut].fp.e.outEdge2 = NO_EDGE;
    }
    ++numOut;
  }

  return numOut;
}

void ComputeIncidentEdge(ClipVertex c[2], const vector2 &h, const vector2 &pos,
                         const matrix2 &Rot, const vector2 &normal) {
  // The normal is from the reference box. Convert it
  // to the incident boxe's frame and flip sign.
  matrix2 RotT = Rot.transpose();
  vector2 n = -(RotT * normal);
  vector2 nAbs = abs(n);

  if (nAbs.x() > nAbs.y()) {
    if (n.x() > 0.0f) {
      c[0].v = vector2(h.x(), -h.y());
      c[0].fp.e.inEdge2 = EDGE3;
      c[0].fp.e.outEdge2 = EDGE4;

      c[1].v = vector2(h.x(), h.y());
      c[1].fp.e.inEdge2 = EDGE4;
      c[1].fp.e.outEdge2 = EDGE1;
    } else {
      c[0].v = vector2(-h.x(), h.y());
      c[0].fp.e.inEdge2 = EDGE1;
      c[0].fp.e.outEdge2 = EDGE2;

      c[1].v = vector2(-h.x(), -h.y());
      c[1].fp.e.inEdge2 = EDGE2;
      c[1].fp.e.outEdge2 = EDGE3;
    }
  } else {
    if (n.y() > 0.0f) {
      c[0].v = vector2(h.x(), h.y());
      c[0].fp.e.inEdge2 = EDGE4;
      c[0].fp.e.outEdge2 = EDGE1;

      c[1].v = vector2(-h.x(), h.y());
      c[1].fp.e.inEdge2 = EDGE1;
      c[1].fp.e.outEdge2 = EDGE2;
    } else {
      c[0].v = vector2(-h.x(), -h.y());
      c[0].fp.e.inEdge2 = EDGE2;
      c[0].fp.e.outEdge2 = EDGE3;

      c[1].v = vector2(h.x(), -h.y());
      c[1].fp.e.inEdge2 = EDGE3;
      c[1].fp.e.outEdge2 = EDGE4;
    }
  }

  c[0].v = pos + Rot * c[0].v;
  c[1].v = pos + Rot * c[1].v;
}

bool collide(std::vector<contact> &contacts, body *b1, body *b2) {
  contacts.clear();
  vector2 hA = 0.5f * b1->size;
  vector2 hB = 0.5f * b2->size;
  vector2 posA = b1->position;
  vector2 posB = b2->position;
  matrix2 RotA = from_angle(b1->rotation), RotB = from_angle(b2->rotation);
  matrix2 RotAT = RotA.transpose(), RotBT = RotB.transpose();
  vector2 dp = posB - posA;
  vector2 dA = RotAT * dp;
  vector2 dB = RotBT * dp;
  matrix2 C = RotAT * RotB;
  matrix2 absC = abs(C);
  matrix2 absCT = absC.transpose();

  // box A faces
  vector2 faceA = abs(dA) - hA - absC * hB;
  if (faceA.x() > 0.0f || faceA.y() > 0.0f)
    return false;
  // box B faces
  vector2 faceB = abs(dB) - absCT * hA - hB;
  if (faceB.x() > 0.0f || faceB.y() > 0.0f)
    return false;

  // find best axis
  Axis axis;
  float separation;
  vector2 normal;

  // Box A faces
  axis = FACE_A_X;
  separation = faceA.x();
  normal = dA.x() > 0.0f ? vector2(RotA.col(0)) : vector2(-RotA.col(0));
  const float relativeTol = 0.95f;
  const float absoluteTol = 0.01f;
  if (faceA.y() > relativeTol * separation + absoluteTol * hA.y()) {
    axis = FACE_A_Y;
    separation = faceA.y();
    normal = dA.y() > 0.0f ? vector2(RotA.col(1)) : vector2(-RotA.col(1));
  }
  // Box B faces
  if (faceB.x() > relativeTol * separation + absoluteTol * hB.x()) {
    axis = FACE_B_X;
    separation = faceB.x();
    normal = dB.x() > 0.0f ? vector2(RotB.col(0)) : vector2(-RotB.col(0));
  }

  if (faceB.y() > relativeTol * separation + absoluteTol * hB.y()) {
    axis = FACE_B_Y;
    separation = faceB.y();
    normal = dB.y() > 0.0f ? vector2(RotB.col(1)) : vector2(-RotB.col(1));
  }

  // Setup clipping plane data based on the separating axis
  vector2 frontNormal, sideNormal;
  ClipVertex incidentEdge[2];
  float front, negSide, posSide;
  char negEdge, posEdge;

  // Compute the clipping lines and the line segment to be clipped.
  switch (axis) {
  case FACE_A_X: {
    frontNormal = normal;
    front = posA.dot(frontNormal) + hA.x();
    sideNormal = RotA.col(1);
    float side = posA.dot(sideNormal);
    negSide = -side + hA.y();
    posSide = side + hA.y();
    negEdge = EDGE3;
    posEdge = EDGE1;
    ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
  } break;

  case FACE_A_Y: {
    frontNormal = normal;
    front = posA.dot(frontNormal) + hA.y();
    sideNormal = RotA.col(0);
    float side = posA.dot(sideNormal);
    negSide = -side + hA.x();
    posSide = side + hA.x();
    negEdge = EDGE2;
    posEdge = EDGE4;
    ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
  } break;

  case FACE_B_X: {
    frontNormal = -normal;
    front = posB.dot(frontNormal) + hB.x();
    sideNormal = RotB.col(1);
    float side = posB.dot(sideNormal);
    negSide = -side + hB.y();
    posSide = side + hB.y();
    negEdge = EDGE3;
    posEdge = EDGE1;
    ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
  } break;

  case FACE_B_Y: {
    frontNormal = -normal;
    front = posB.dot(frontNormal) + hB.y();
    sideNormal = RotB.col(0);
    float side = posB.dot(sideNormal);
    negSide = -side + hB.x();
    posSide = side + hB.x();
    negEdge = EDGE2;
    posEdge = EDGE4;
    ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
  } break;
  }

  // clip other face with 5 box planes (1 face plane, 4 edge planes)
  ClipVertex clipPoints1[2];
  ClipVertex clipPoints2[2];
  int np;
  // Clip to box side 1
  np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide,
                         negEdge);

  if (np < 2)
    return false;
  // Clip to negative box side 1
  np =
      ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

  if (np < 2)
    return false;

  for (int i = 0; i < 2; ++i) {
    float separation = frontNormal.dot(clipPoints2[i].v) - front;

    if (separation <= 0) {
      contact new_contact;
      new_contact.separation = separation;
      new_contact.normal = normal;
      // slide contact point onto reference face (easy to cull)
      new_contact.position = clipPoints2[i].v - separation * frontNormal;
      new_contact.feature = clipPoints2[i].fp;
      if (axis == FACE_B_X || axis == FACE_B_Y)
        Flip(new_contact.feature);
      contacts.emplace_back(new_contact);
    }
  }
  return true;
}

}; // namespace toolkit::sdl2d