#include "AABB.h"
#include "Logger.h"
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
  Logger::showLogMulComments(0, __FILE__, __LINE__, "开始执行AABB求交");

  Point3f origin = ray.origin;
  Vector3f direction = ray.direction;

  float tNear = -FLT_MAX;
  float tFar = FLT_MAX;
  const float epsilon = 1e-6f;

  // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始遍历坐标轴,当前光束的方向为", direction[0], " ", direction[1], " ", direction[2]);

  for (int i = 0; i < 3; i++)
  { // 遍历 x, y, z 轴
    if (std::abs(ray.direction[i]) < epsilon)
    {
      // 光线平行于当前轴，检查起点是否在 AABB 内
      if (ray.origin[i] < pMin[i] || ray.origin[i] > pMax[i])
      {
        return false;
      }
    }
    else
    {
      // 计算交点时间
      float t1 = (pMin[i] - ray.origin[i]) / ray.direction[i];
      float t2 = (pMax[i] - ray.origin[i]) / ray.direction[i];
      Logger::showLogMulComments(0, __FILE__, __LINE__, "当前包围盒的边界为:", pMin[i], " ", pMax[i] );
      Logger::showLogMulComments(0, __FILE__, __LINE__, "当前的射线的起点为:", ray.origin[i]);
      Logger::showLogMulComments(0, __FILE__, __LINE__, "当前的距离分别为:", pMin[i] - ray.origin[i], " ", pMax[i] - ray.origin[i]);

      if (t1 > t2)
        std::swap(t1, t2); // 确保 t1 <= t2
      // Logger::showLogMulComments(0, __FILE__, __LINE__, "当前的t1和t2分别为:", t1, " ", t2);
      // 如果 t2 < 0，光线从背面进入，跳过该轴
      if (t2 < 0)
        continue;

      // 如果 t1 < 0 但 t2 >= 0，光线起点在 AABB 内
      if (t1 < 0)
        t1 = 0;
      // 更新全局区间
      tNear = std::max(tNear, t1);
      tFar = std::min(tFar, t2);
      Logger::showLogMulComments(0, __FILE__, __LINE__, "当前的tNear和tFar分别为:", tNear, " ", tFar);

      // 检查是否无重叠
      if (tNear > tFar)
        return false;
    }
  }

  // Logger::showLogMulComments(0, __FILE__, __LINE__, "计算结果为", tNear, " ", tFar);

  if (tMin)
    *tMin = tNear;

  if (tMax)
    *tMax = tFar;

  Logger::showLogMulComments(0, __FILE__, __LINE__, "求交结束");

  return true;
}

Point3f AABB::Center() const
{
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}