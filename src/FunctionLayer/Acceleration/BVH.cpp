#include "BVH.h"
#include "Logger.h"
#include "vector"
#include "algorithm"
#include <fstream>

BVH::BVHNode *BVH::buildRecursive(int start, int end, std::vector<std::shared_ptr<Shape>> &orderedshapes, std::vector<ShapeInfo> &shapeInfo)
{
    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始建立BVH树");
    BVHNode *node = new BVHNode();
    AABB totalBounds;
    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始创建包围盒");

    for (int i = start; i < end; i++)
        totalBounds.Expand(shapeInfo[i].bounds);

    node->boundingBox = totalBounds;

    auto normalStop = [&](int upperbound) -> bool
    {
        int nShapes = end - start;
        return 0 < nShapes && nShapes <= upperbound;
    };
    auto normalOpe = [&]()
    {
        node->splitAxis = -1;
        // 逐渐添加有序序列 在子节点内部是有序的
        node->firstShapeOffset = orderedshapes.size();
        node->nShape = end - start;
        nodeCount++;
        leafCount++;
        // 放置叶子节点时记录顺序
        for (int i = start; i < end; i++)
        {
            orderedshapes.push_back(shapes[shapeInfo[i].geomId]);
        }
    };

    if (normalStop(1))
    {
        // 叶子节点
        normalOpe();
        Logger::showLogMulComments(0, __FILE__, __LINE__, "创建了叶子结点");
    }
    else
    {
        // 记录本节点中心点形成的总包围盒
        AABB currentCenterBounds;
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "查看当前区间", start, " ", end);

        for (int i = start; i < end; i++)
        {
            currentCenterBounds.Expand(shapeInfo[i].center);
        }

        // 求切分轴 求切分点
        BVH::SplitInfo target = getSplitInfo(shapeInfo, currentCenterBounds, {start, end}, staticSplit);
        int splitPoint = target.splitPoint;
        int splitAxis = target.splitAxis;
        node->splitAxis = splitAxis;

        // 无法划分的情况
        if (node->splitAxis == -1)
        {
            node->firstShapeOffset = orderedshapes.size();
            node->nShape = end - start;

            for (int i = start; i < end; i++)
            {
                orderedshapes.push_back(shapes[shapeInfo[i].geomId]);
            }
        }

        // 递归构建左右子树
        if (node->splitAxis != -1)
        {
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始递归构建");

            node->childList[0] = buildRecursive(start, splitPoint, orderedshapes, shapeInfo);
            node->childList[1] = buildRecursive(splitPoint, end, orderedshapes, shapeInfo);
        }
        nodeCount++;
        indexCount++;
    }
    return node;
}

// 静态切分 axis不变 开始从这个轴的中心点不断二分
BVH::SplitInfo BVH::staticSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval)
{
    // 切分轴固定为dim=1 不断求中点切分
    int dim = 1;
    int splitPoint = -1;
    int splitAxis = -1;
    int start = interval.first, end = interval.second;
    double pmid = 0.5 * (centerBound.pMin[dim] + centerBound.pMax[dim]);
    splitPoint = std::partition(shapeInfo.begin() + start, shapeInfo.begin() + end, [&](const BVH::ShapeInfo &pi)
                                { return pi.center[dim] < pmid; }) -
                 shapeInfo.begin();

    splitAxis = dim;
    if (splitPoint == start || splitPoint == end)
        splitAxis = -1;

    BVH::SplitInfo result = {};
    result.splitAxis = splitAxis;
    result.splitPoint = splitPoint;
    return result;
}

// 最长中点切分
BVH::SplitInfo BVH::longestExtentSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval)
{
    // 求切分轴
    int dim = -1;
    double maxD = 0;
    for (int i = 0; i < 3; i++)
    {
        double D = centerBound.pMax[i] - centerBound.pMin[i];
        if (maxD < D)
            maxD = D, dim = i;
    }
    // Logger::showLogMulComments(0, __FILE__, __LINE__, "计算结束,得到的dim为", dim);

    int splitPoint = -1;
    int splitAxis = -1;
    int start = interval.first, end = interval.second;
    double pmid = 0.5 * (centerBound.pMin[dim] + centerBound.pMax[dim]);
    splitPoint = std::partition(shapeInfo.begin() + start, shapeInfo.begin() + end, [&](const BVH::ShapeInfo &pi)
                                { return pi.center[dim] < pmid; }) -
                 shapeInfo.begin();

    splitAxis = dim;
    if (splitPoint == start || splitPoint == end)
        splitAxis = -1;

    BVH::SplitInfo result = {};
    result.splitAxis = splitAxis;
    result.splitPoint = splitPoint;
    Logger::showLogMulComments(0, __FILE__, __LINE__, "退出最长中点切分方法,计算结果:轴为", splitAxis, "点为", splitPoint);

    return result;
}
// 等量对象切分
BVH::SplitInfo BVH::equalShapeSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval)
{

    // 求切分轴
    int dim = -1;
    double maxD = 0;
    for (int i = 0; i < 3; i++)
    {
        double D = centerBound.pMax[i] - centerBound.pMin[i];
        if (maxD < D)
            maxD = D, dim = i;
    }

    int splitPoint = -1;
    int splitAxis = -1;
    int start = interval.first, end = interval.second;
    int mid = (start + end) >> 1;
    std::nth_element(shapeInfo.begin() + start,
                     shapeInfo.begin() + mid,
                     shapeInfo.begin() + end,
                     [&](const BVH::ShapeInfo &a, const BVH::ShapeInfo &b)
                     {
                         return a.center[dim] < b.center[dim];
                     });

    splitPoint = mid;
    splitAxis = dim;
    if (splitPoint == start || splitPoint == end)
        splitAxis = -1;

    BVH::SplitInfo result = {};
    result.splitAxis = splitAxis;
    result.splitPoint = splitPoint;
    return result;
}

// SAH切分
BVH::SplitInfo BVH::sahSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval)
{
    BVH::SplitInfo result = {};
    return result;
}

// 接口
BVH::SplitInfo BVH::getSplitInfo(std::vector<ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval, SplitCallback callback)
{
    // 挺难绷的语法，成员函数指针
    return callback(shapeInfo, centerBound, interval);
}

void BVH::build()
{
    Logger::showLogMulComments(0, __FILE__, __LINE__, "调用了build函数");

    std::vector<ShapeInfo> shapeInfoList;
    int count = 0;
    // shapes是父类自带的 表示场景中所有的几何体
    for (const auto &shape : shapes)
    {
        shape->initInternalAcceleration();
        boundingBox.Expand(shape->getAABB());
        shapeInfoList.push_back(ShapeInfo(count++, shape->getAABB()));
    }
    // 创建待排序数组 初始为空 留待填充
    std::vector<std::shared_ptr<Shape>> orderedshapes;

    // 递归构建 是从 [0,n)
    root = buildRecursive(0, shapes.size(), orderedshapes, shapeInfoList);
    Logger::showLogMulComments(0, __FILE__, __LINE__, "构建结束,当前节点总个数为:", nodeCount, ",索引节点个数为:", indexCount, ",叶子节点个数为:", leafCount);

    shapes.swap(orderedshapes);
}

// 扁平化树进行求交
void BVH::Flatten(BVHNode *node, int &dfsOrder)
{
    if (node == nullptr)
        return;
    auto &lnode = linearBvhNodes[dfsOrder++];
    lnode.boundingBox = node->boundingBox;
    if (node->nShape > 0 && node->splitAxis != -1)
    {
        // leaf
        lnode.firstdShapeOffset = node->firstShapeOffset;
        lnode.nShape = node->nShape;
    }
    else
    {
        // interior
        lnode.splitAxis = node->splitAxis;
        Flatten(node->childList[0], dfsOrder);
        lnode.secondChildOrder = dfsOrder;
        Flatten(node->childList[1], dfsOrder);
    }
}

bool BVH::traverse(BVH::BVHNode *node, Ray &R, int *geomID, int *primID, float *u, float *v) const
{
    // 没东西直接返回
    if (!node)
    {
        Logger::showLogMulComments(0, __FILE__, __LINE__, "节点为空,结束DFS遍历");
        return false;
    }
    Logger::showLogMulComments(0, __FILE__, __LINE__, "当前节点子节点的个数", node->childList.size());

    // 检查光线是否与当前节点的AABB相交
    float tMin = R.tNear, tMax = R.tFar;
    if (!node->boundingBox.RayIntersect(R, &tMin, &tMax))
    {
        return false;
    }

    if (node->splitAxis == -1)
    {
        bool localHit = false;
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "是子节点,开始查找内部形状");

        for (int i = 0; i < node->nShape; i++)
        {
            int idx = node->firstShapeOffset + i;

            if (shapes[idx]->rayIntersectShape(R, primID, u, v))
            {
                localHit = true;
                // 更新 geomID（如果是多几何体场景）
                *geomID = idx;
            }
        }
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "在shape", node->firstShapeOffset, "上是否产生了交点呢:", flag);
        return localHit;
    }
    else
    {
        // 优化：按光线方向选择遍历顺序
        int axis = node->splitAxis;
        int sequence = -1;
        if (R.direction[axis] > 0)
        {
            sequence = 0;
        }
        else
        {
            sequence = 1;
        }
        BVHNode *firstChild = node->childList[sequence];
        BVHNode *secondChild = node->childList[!sequence];

        return traverse(firstChild, R, geomID, primID, u, v) || traverse(secondChild, R, geomID, primID, u, v);
    }
}

bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const
{

    // 完成BVH求交
    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始进行BVH的求交");

    // 和树求交
    bool flag = traverse(root, ray, geomID, primID, u, v);

    Logger::showLogMulComments(0, __FILE__, __LINE__, "结束DFS遍历");

    return flag;
}