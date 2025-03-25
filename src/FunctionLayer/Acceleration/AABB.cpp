#include "AABB.h"

Point3f minP(const Point3f &p1, const Point3f &p2)
{
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

Point3f maxP(const Point3f &p1, const Point3f &p2)
{
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}

AABB AABB::Union(const AABB &other) const
{
  Point3f min = minP(other.pMin, pMin), max = maxP(other.pMax, pMax);
  return AABB{min, max};
}

void AABB::Expand(const AABB &other)
{
  pMin = minP(pMin, other.pMin);
  pMax = maxP(pMax, other.pMax);
}

AABB AABB::Union(const Point3f &other) const
{
  Point3f min = minP(other, pMin), max = maxP(other, pMax);
  return AABB{min, max};
}

void AABB::Expand(const Point3f &other)
{
  pMin = minP(pMin, other);
  pMax = maxP(pMax, other);
}

bool AABB::Overlap(const AABB &other) const
{
  for (int dim = 0; dim < 3; ++dim)
  {
    if (pMin[dim] > other.pMax[dim] || pMax[dim] < other.pMin[dim])
    {
      return false;
    }
  }
  return true;
}

bool AABB::RayIntersect(const Ray &ray, float *tMin, float *tMax) const
{
  // 实现AABB与光线求交
  // 遍历xyz三个轴
  float xMin = pMin[0], xMax = pMax[0],
        yMin = pMin[1], yMax = pMax[1],
        zMin = pMin[2], zMax = pMax[2];
  Point3f origin = ray.origin;
  Vector3f direction = ray.direction;

  float tNear = FLT_MAX;
  float tFar = -FLT_MAX;

  // 遍历 x 轴
  if (direction[0] != 0.0f)
  {
    float t1 = (xMin - origin[0]) / direction[0];
    float t2 = (xMax - origin[0]) / direction[0];
    float tMinX = std::min(t1, t2);
    float tMaxX = std::max(t1, t2);
    tNear = std::max(tNear, tMinX);
    tFar = std::min(tFar, tMaxX);
  }
  else
  {
    if (origin[0] < xMin || origin[0] > xMax)
    {
      return false;
    }
  }

  // 遍历 y 轴
  if (direction[1] != 0.0f)
  {
    float t1 = (yMin - origin[1]) / direction[1];
    float t2 = (yMax - origin[1]) / direction[1];
    float tMinY = std::min(t1, t2);
    float tMaxY = std::max(t1, t2);
    tNear = std::max(tNear, tMinY);
    tFar = std::min(tFar, tMaxY);
  }
  else
  {
    if (origin[1] < yMin || origin[1] > yMax)
    {
      return false;
    }
  }

  // 遍历 z 轴
  if (direction[2] != 0.0f)
  {
    float t1 = (zMin - origin[2]) / direction[2];
    float t2 = (zMax - origin[2]) / direction[2];
    float tMinZ = std::min(t1, t2);
    float tMaxZ = std::max(t1, t2);
    tNear = std::max(tNear, tMinZ);
    tFar = std::min(tFar, tMaxZ);
  }
  else
  {
    if (origin[2] < zMin || origin[2] > zMax)
    {
      return false;
    }
  }

  if (tNear > tFar)
  {
    return false;
  }

  if (tMin)
  {
    *tMin = tNear;
  }
  if (tMax)
  {
    *tMax = tFar;
  }
  return false;
}

Point3f AABB::Center() const
{
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}