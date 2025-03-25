#include "BVH.h"
#include "vector"
#include "algorithm"

BVH::BVHNode *BVH::buildRecursive(int start, int end, std::vector<std::shared_ptr<Shape>> &orderedshapes, std::vector<std::shared_ptr<Shape>> &shapeInfo, std::vector<std::shared_ptr<Shape>> boundingList)
{
    BVHNode *node = new BVHNode();
    auto normalStop = [&]() -> bool
    {
        int nShapes = end - start;
        return nShapes < bvhLeafMaxSize;
    };
    auto normalOpe = [&]()
    {
        node->splitAxis = -1;
        node->firstShapeOffset = start;
        node->nShape = end - start;
        // 放置叶子节点时记录顺序
        for (int i = start; i < end; i++)
        {
            orderedshapes.push_back(shapes[shapeInfo[i]->geometryID]);
        }
    };
    if (normalStop())
    {
        // 叶子节点
        normalOpe();
    }
    else
    {
        // 记录本节点的包围盒
        node->boundingList = boundingList;

        // 非叶子节点，选择分割轴 此时boundingList已经排序了
        BVH::SplitInfo target = getSplitInfo(boundingList,staticSplit);
        int splitPoint = target.splitPoint;
        int splitAxis = target.splitAxis;
        node->splitAxis = splitAxis;

        // 分割物体
        std::vector<std::shared_ptr<Shape>> leftBoundingList, rightBoundingList;
        for (int i = start; i < splitPoint; ++i)
        {
            leftBoundingList.push_back(boundingList[i]);
        }
        for (int i = splitPoint; i < end; ++i)
        {
            leftBoundingList.push_back(boundingList[i]);
        }

        // 递归构建左右子树
        node->childList.push_back(buildRecursive(start, splitAxis, orderedshapes, shapeInfo, leftBoundingList));
        node->childList.push_back(buildRecursive(splitAxis, end, orderedshapes, shapeInfo, rightBoundingList));
    }

    return node;
}

// 静态切分 axis不变 开始从这个轴的中心点不断二分
BVH::SplitInfo staticSplit(const std::vector<std::shared_ptr<Shape>> &boundingList)
{
    std::vector<AABB> boxes;
    for (const auto &shape : boundingList)
    {
        boxes.push_back(shape->getAABB());
    }
    BVH::SplitInfo result = {};
    return result;
}

// 最长切分
BVH::SplitInfo longestExtentSplit(const std::vector<std::shared_ptr<Shape>> &boxes)
{
    BVH::SplitInfo result = {};
    return result;
}

// SAH切分
BVH::SplitInfo sahSplit(const std::vector<std::shared_ptr<Shape>> &boxes)
{
    BVH::SplitInfo result = {};
    return result;
}

// 接口
BVH::SplitInfo BVH::getSplitInfo(const std::vector<std::shared_ptr<Shape>> &boxes, SplitCallback callback)
{
    return callback(boxes);
}

void BVH::build()
{
    AABB sceneBox;
    // shapes是父类自带的 表示场景中所有的几何体
    for (const auto &shape : shapes)
    {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();
        sceneBox.Expand(shape->getAABB());
    }
    // 创建待排序数组 初始为空 留待填充
    std::vector<std::shared_ptr<Shape>> orderedshapes = std::vector<std::shared_ptr<Shape>>(shapes.size());
    std::vector<std::shared_ptr<Shape>> boundingList = shapes;
    // 递归构建
    root = buildRecursive(0, shapes.size(), orderedshapes, shapes, boundingList);
    shapes = orderedshapes;
}
bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const
{
    // 完成BVH求交
    Point3f origin = ray.origin;
    Vector3f direction = ray.direction;

    // 和树求交

    // 找到叶子结点

    // 叶子结点内遍历
    return false;
}
