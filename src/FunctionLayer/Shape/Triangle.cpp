#include "Triangle.h"
#include <FunctionLayer/Acceleration/Linear.h>
//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh)
{
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const
{
  Point3f origin = ray.origin;
  Vector3f direction = ray.direction;
  // 获得mesh法向量
  // 计算边向量
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  Vector3f edge0 = vtx1 - vtx0;
  Vector3f edge1 = vtx2 - vtx0;
  // 获得法向量
  Vector3f paralNormal = normalize(cross(edge0, edge1));

  double b = dot(paralNormal, direction);
  // 是否平行
  if (fabs(b) < 1e-8)
    return false; // miss

  // 计算重心
  Vector3f T = origin - vtx0, D = direction;
  Vector3f Q = cross(T, edge0), P = cross(D, edge1);
  double PE = dot(P, edge0), invPE = 1.0 / PE;
  double t = dot(Q, edge1) * invPE, u_ = dot(P, T) * invPE, v_ = dot(Q, D) * invPE;

  // 交点超出光线范围
  if (t < ray.tNear || t > ray.tFar)
    return false;
  // 解算赋值
  if (0.f <= u_ && 0.f <= v_ && u_ + v_ <= 1.f)
  {
    ray.tFar = t;
    *primID = this->primID;
    *u = u_;
    *v = v_;
    return true;
  }
  return false;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const
{
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json)
{
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const
{
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i)
  {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i)
  {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const
{
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const
{
  //* 填充光线与三角网格求交得到的交点信息
  intersection->distance = distance;
  intersection->shape = this;
  //* 在三角形内部用插值计算交点、法线以及纹理坐标
  auto faceInfo = meshData->faceBuffer[primID];
  float w = 1.f - u - v;

  //* 1. 在三角形内部用插值计算交点坐标
  Point3f pw = transform.toWorld(
              meshData->vertexBuffer[faceInfo[0].vertexIndex]),
          pu = transform.toWorld(
              meshData->vertexBuffer[faceInfo[1].vertexIndex]),
          pv = transform.toWorld(
              meshData->vertexBuffer[faceInfo[2].vertexIndex]);
  intersection->position = Point3f{w * pw[0] + u * pu[0] + v * pv[0],
                                   w * pw[1] + u * pu[1] + v * pv[1],
                                   w * pw[2] + u * pu[2] + v * pv[2]};

  //* 2. 在三角形内部用插值计算法线
  if (meshData->normalBuffer.size() != 0)
  {
    Vector3f nw = transform.toWorld(
                 meshData->normalBuffer[faceInfo[0].normalIndex]),
             nu = transform.toWorld(
                 meshData->normalBuffer[faceInfo[1].normalIndex]),
             nv = transform.toWorld(
                 meshData->normalBuffer[faceInfo[2].normalIndex]);
    intersection->normal = normalize(w * nw + u * nu + v * nv);
  }
  else
  {
    intersection->normal = normalize(cross(pu - pw, pv - pw));
  }

  //* 3. 在三角形内部用插值计算纹理坐标
  Vector2f tw, tu, tv;
  if (meshData->texcodBuffer.size() != 0)
  {
    tw = meshData->texcodBuffer[faceInfo[0].texcodIndex],
    tu = meshData->texcodBuffer[faceInfo[1].texcodIndex],
    tv = meshData->texcodBuffer[faceInfo[2].texcodIndex];
    intersection->texCoord = w * tw + u * tu + v * tv;
  }
  else
  {
    tw = Vector2f{.0f, .0f};
    tu = Vector2f{.1f, .0f};
    tv = Vector2f{.1f, .1f};
    intersection->texCoord = Vector2f{.0f, .0f};
  }

  //* 4. 在三角形内部用插值计算交点的切线和副切线
  Vector3f edge0 = pu - pw;
  Vector3f edge1 = pv - pw;
  Vector2f duv02 = tw - tv, duv12 = tu - tv;
  double uvDet = duv02[0] * duv12[1] - duv02[1] * duv12[0];
  double invDet = 1.0 / uvDet;
  Vector3f dpdu = (duv12[1] * edge0 - duv02[1] * edge1) * invDet;
  Vector3f dpdv = (-duv12[0] * edge0 + duv02[0] * edge1) * invDet;
  intersection->dpdu = dpdu,
  intersection->dpdv = dpdv;
  intersection->tangent = normalize(intersection->dpdu);
  intersection->bitangent =
      normalize(cross(intersection->tangent, intersection->normal));

  auto debugPrint = [](Intersection *intersection)
  {
    std::cout << "开始打印交点" << std::endl;
    std::cout << "距离" << intersection->distance << std::endl;
    std::cout << "交点" << std::endl;
    intersection->position.debugPrint();
    std::cout
        << "法线/切线/副切线" << std::endl;
    intersection->normal.debugPrint();
    intersection->tangent.debugPrint();
    intersection->bitangent.debugPrint();
    std::cout << "纹理坐标" << std::endl;
    intersection->texCoord.debug_print();
    std::cout << "uv切线" << std::endl;
    intersection->dpdu.debugPrint();
    intersection->dpdv.debugPrint();
  };

  // debugPrint(intersection);
}

void TriangleMesh::initInternalAcceleration()
{
  // 1. 创建加速结构实例（如BVH/KD-Tree）
  acceleration = Acceleration::createAcceleration();

  // 2. 遍历所有三角形，将每个三角形加入加速结构
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID)
  {
    // 获取三角形三个顶点的索引
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex;
    int vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex;
    int vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;

    // 创建Triangle对象并绑定到加速结构
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  // 3. 构建加速结构（如BVH的分割和平衡）
  acceleration->build();
  // 4. 用加速结构的包围盒作为整个网格的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")