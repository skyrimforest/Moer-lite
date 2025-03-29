#include "BVH.h"
#include "Logger.h"
#include "vector"
#include "algorithm"
#include <fstream>

BVH::BVHNode *BVH::buildRecursive(int start, int end, std::vector<std::shared_ptr<Shape>> &orderedshapes, std::vector<ShapeInfo> &shapeInfo)
{
    if (end - start <= 0)
    {
        Logger::showLogMulComments(0, __FILE__, __LINE__, "区间不正常,停止运行!!!此时开始和结尾为", start, " ", end);
        return nullptr;
    }

    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始建立BVH树");
    BVHNode *node = new BVHNode();
    AABB totalBounds;
    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始创建包围盒");

    for (int i = start; i < end; i++)
        totalBounds.Expand(shapeInfo[i].bounds);

    if (fabs(totalBounds.pMax[0]) > 1e10)
    {
        Logger::showLogMulComments(0, __FILE__, __LINE__, "数字不正常,停止运行!!!此时开始和结尾为", start, " ", end);
        return nullptr;
    }
    // Logger::showLogMulComments(0, __FILE__, __LINE__, "完成包围盒合并,最小值结果为", totalBounds.pMin[0], totalBounds.pMin[1], totalBounds.pMin[2]);
    // Logger::showLogMulComments(0, __FILE__, __LINE__, "完成包围盒合并,最大值结果为", totalBounds.pMax[0], totalBounds.pMax[1], totalBounds.pMax[2]);

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
    // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始判断是索引还是叶子节点");

    if (normalStop(1))
    {
        // 叶子节点
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始创建叶子结点");
        normalOpe();
        Logger::showLogMulComments(0, __FILE__, __LINE__, "创建了叶子结点");
    }
    else
    {
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始创建索引结点");

        // 非叶子节点，选择分割轴 此时shapeInfo已经排序了
        // 记录本节点中心点形成的总包围盒
        AABB currentCenterBounds;
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "查看当前区间", start, " ", end);

        for (int i = start; i < end; i++)
        {
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "查看当前节点", i, "的中点坐标", shapeInfo[i].center[0], " ", shapeInfo[i].center[1], " ", shapeInfo[i].center[2]);
            currentCenterBounds.Expand(shapeInfo[i].center);
        }

        // Logger::showLogMulComments(0, __FILE__, __LINE__, "查看中点包围盒", currentCenterBounds.pMax[0], "和", currentCenterBounds.pMin[0]);

        // 求切分轴 求切分点
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始寻找最佳切分轴和轴上的最佳切分点");
        BVH::SplitInfo target = getSplitInfo(shapeInfo, currentCenterBounds, {start, end}, longestExtentSplit);
        int splitPoint = target.splitPoint;
        int splitAxis = target.splitAxis;
        node->splitAxis = splitAxis;
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "计算结束,点为:", splitPoint, "轴为:", splitAxis);

        // 无法划分的情况
        if (node->splitAxis == -1)
        {
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "无法划分! 开始重新划入子节点...");
            node->firstShapeOffset = orderedshapes.size();
            node->nShape = end - start;

            for (int i = start; i < end; i++)
            {
                orderedshapes.push_back(shapes[shapeInfo[i].geomId]);
            }
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "重新划分结束...");
        }

        // 递归构建左右子树
        if (node->splitAxis != -1)
        {
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "开始递归构建");

            node->childList.push_back(buildRecursive(start, splitPoint, orderedshapes, shapeInfo));
            node->childList.push_back(buildRecursive(splitPoint, end, orderedshapes, shapeInfo));
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
    // Logger::showLogMulComments(0, __FILE__, __LINE__, "进入最长中点切分方法");
    // 求切分轴
    int dim = -1;
    double maxD = -FLT_MAX;
    for (int i = 0; i < 3; i++)
    {
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "当前轴为", i, "当前centerBound的值为", centerBound.pMax[i], "和", centerBound.pMin[i]);
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
    AABB sceneBox;
    std::vector<ShapeInfo> shapeInfoList;
    int count = 0;
    // shapes是父类自带的 表示场景中所有的几何体
    for (const auto &shape : shapes)
    {
        shape->initInternalAcceleration();
        sceneBox.Expand(shape->getAABB());

        shapeInfoList.push_back(ShapeInfo(count++, shape->getAABB()));
    }
    // 创建待排序数组 初始为空 留待填充
    std::vector<std::shared_ptr<Shape>> orderedshapes;
    // Logger::showLogMulComments(0, __FILE__, __LINE__, "求得的shapeinfo", shapeInfoList[2].geomId, " ", shapeInfoList[3].center[0]);

    // 递归构建 是从 [0,n)
    root = buildRecursive(0, shapes.size(), orderedshapes, shapeInfoList);
    Logger::showLogMulComments(0, __FILE__, __LINE__, "构建结束,当前节点总个数为:", nodeCount, ",索引节点个数为:", indexCount, ",叶子节点个数为:", leafCount);

    shapes.swap(orderedshapes);
}
// todo 扁平化树进行求交
// void Flatten(std::shared_ptr<BVH::BVHNode> node, int &dfsOrder)
// {
//     if (node == nullptr)
//         return;
//     auto &lnode = linearBvhNodes[dfsOrder++];
//     lnode.bounds = node->bounds;
//     if (node->nEntites > 0)
//     {
//         // leaf
//         lnode.firstdEntityOffset = node->entityOffset;
//         lnode.nEntites = node->nEntites;
//     }
//     else
//     {
//         // interior
//         lnode.splitAxis = node->splitAxis;
//         Flatten(node->children[0], dfsOrder);
//         lnode.secondChildOrder = dfsOrder;
//         Flatten(node->children[1], dfsOrder);
//     }
// }

bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const
{
    // 完成BVH求交
    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始进行BVH的求交");

    Point3f origin = ray.origin;
    Vector3f direction = ray.direction;
    Ray R(ray);
    // 和树求交
    // int currentNode = 0;
    std::function<bool(BVHNode *)> traverse = [&](BVHNode *node) -> bool
    {
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "进入DFS内部");
        Vector3f invDir(1 / ray.direction[0], 1 / ray.direction[1], 1 / ray.direction[2]);
        bool isNegDir[3] = {ray.direction[0] < 0, ray.direction[1] < 0, ray.direction[2] < 0};
        bool flag = false;
        // 没东西直接返回

        if (!node)
        {
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "节点为空,结束DFS遍历");
            return flag;
        }
        Logger::showLogMulComments(0, __FILE__, __LINE__, "求交前本节点包围盒最小值为:", node->boundingBox.pMin[0], " ", node->boundingBox.pMin[1], " ", node->boundingBox.pMin[2]);
        Logger::showLogMulComments(0, __FILE__, __LINE__, "求交前本节点包围盒最大值为:", node->boundingBox.pMax[0], " ", node->boundingBox.pMax[1], " ", node->boundingBox.pMax[2]);
        if (fabs(node->boundingBox.pMax[0]) > 1e10)
        {
            Logger::showLogMulComments(0, __FILE__, __LINE__, "数字不正常,停止运行!!!此时节点信息为", node->firstShapeOffset, " ", node->nShape, " ", node->splitAxis);
            return false;
        }
        // 相交了
        if (node->boundingBox.RayIntersect(R))
        {
            // Logger::showLogMulComments(0, __FILE__, __LINE__, "灯光和盒子相交");
            if (node->splitAxis == -1)
            {
                // 是子节点
                // Logger::showLogMulComments(0, __FILE__, __LINE__, "是子节点,开始查找内部形状");
                if (node->nShape > 0)
                {

                    for (int i = 0; i < node->nShape; i++)
                    {
                        int idx = node->firstShapeOffset + i;

                        if (!shapes.empty())
                            flag = shapes[idx]->rayIntersectShape(R, primID, u, v);
                    }
                    // Logger::showLogMulComments(0, __FILE__, __LINE__, "在shape", node->firstShapeOffset, "上是否产生了交点呢:", flag);
                }
                else
                {
                    // 如果是多叉树需要按照交点距离排序再遍历
                    // 是索引节点
                    // Logger::showLogMulComments(0, __FILE__, __LINE__, "是索引节点,开始递归查找");
                    if (isNegDir[node->splitAxis])
                    {
                        traverse(node->childList[0]);
                        traverse(node->childList[1]);
                    }
                    else
                    {
                        traverse(node->childList[1]);
                        traverse(node->childList[0]);
                    }
                }
            }
        }
        // 没相交啥也不用干
        // Logger::showLogMulComments(0, __FILE__, __LINE__, "没相交,退出DFS了");

        return flag;
    };

    //* 有交点，需要填充intersection数据结构
    // ray.tFar = rtcRayHit.ray.tfar;
    // *geomID = rtcRayHit.hit.geomID;
    // *primID = rtcRayHit.hit.primID;
    // *u = rtcRayHit.hit.u;
    // *v = rtcRayHit.hit.v;

    Logger::showLogMulComments(0, __FILE__, __LINE__, "开始进行DFS遍历");
    bool flag = traverse(root);
    Logger::showLogMulComments(0, __FILE__, __LINE__, "结束DFS遍历");

    return flag;
}