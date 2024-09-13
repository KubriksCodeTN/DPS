#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

namespace collision2d {
template <typename N>
using Point = Eigen::Matrix<N, 2, 1>;

template <typename N>
using Polygon = std::vector<Point<N>>;

/**
 * Computes potential separating axes for a convex polygon.
 */
template <typename N>
void separatingAxes(const Polygon<N> &a, std::vector<Point<N>> &axes) {
  for (auto i = 0u; i < a.size(); ++i) {
    const auto current = a[i];
    const auto next = a[(i + 1) % a.size()];
    const auto edge = (next - current).normalized();
    // axis is the normal of an edge
    axes.emplace_back(Point<N>{-edge[1], edge[0]});
  }
}

/**
 * Projects the polygon onto the given axis and returns the maximum and minimum
 * coordinate on the axis. Note that the axis has to be normalized.
 */
template <typename N>
void project(const Polygon<N> &a, const Point<N> &axis, N &minProj,
             N &maxProj) {
  maxProj = -std::numeric_limits<N>::infinity();
  minProj = std::numeric_limits<N>::infinity();
  for (const Point<N> &v : a) {
    const N proj = axis.dot(v);
    if (proj < minProj) minProj = proj;
    if (proj > maxProj) maxProj = proj;
  }
}

/**
 * Check for collision between polygons a and b via the Separating Axis Theorem.
 */
template <typename N>
bool intersect(const Polygon<N> &a, const Polygon<N> &b) {
  // compute separating axes
  std::vector<Point<N>> axes;
  separatingAxes(a, axes);
  separatingAxes(b, axes);
  for (const auto &axis : axes) {
    N aMaxProj, aMinProj, bMaxProj, bMinProj;
    project(a, axis, aMinProj, aMaxProj);
    project(b, axis, bMinProj, bMaxProj);
    // check if projections overlap
    if (aMinProj > bMaxProj || bMinProj > aMaxProj) return false;
  }

  return true;
}

// /**
//  * Efficient test for a point to be in a convex polygon.
//  *
//  * Robert Nowak "An Efficient Test for a Point to Be in a Convex Polygon"
//  * http://demonstrations.wolfram.com/AnEfficientTestForAPointToBeInAConvexPolygon/
//  * Wolfram Demonstrations Project
//  * Published: March 7 2011
//  */
// template <typename N>
// bool intersect2(const Point<N> &point, const Polygon<N> &polygon,
//                 const N &epsilon = 1e-4) {
//   bool angle = false;  // stores the sign of the last angle
//   for (auto i = 0u; i < polygon.size(); ++i) {
//     const auto &a = polygon[i] - point;
//     const auto &b = polygon[(i + 1) % polygon.size()] - point;
//     const bool newAngle = b(0) * a(1) - a(0) * b(1) > -epsilon;
//     if (i > 0 && angle != newAngle) return false;
//     angle = newAngle;
//   }
//   return true;
// }

/**
 * Tests whether point intersects with convex polygon.
 * Source: https://stackoverflow.com/a/8721483
 */
template <typename N>
bool intersect(const Point<N> &point, const Polygon<N> &polygon,
               const N &epsilon = 1e-4) {
  bool result = false;
  std::size_t i, j;
  for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if ((polygon[i].y() > point.y()) != (polygon[j].y() > point.y()) &&
        (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) /
                          (polygon[j].y() - polygon[i].y() + epsilon) +
                      polygon[i].x())) {
      result = !result;
    }
  }
  return result;
}
}  // namespace collision2d
