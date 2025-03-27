#include "BVH.h"
#include "vector"
#include "algorithm"
#include <fstream>

void BVH::showLog(std::string comment, int level = 0, const std::string &file = __FILE__, int line = __LINE__)
{
    //  DEBUG = 0,
    // INFO = 1,
    // WARNING = 2,
    // ERROR = 3
    int debugMode = 1;

    if (debugMode == 0)
    {
        return;
    }

    const std::string COLOR_RESET = "\033[0m";
    const std::string COLOR_RED = "\033[31m";
    const std::string COLOR_GREEN = "\033[32m";
    const std::string COLOR_YELLOW = "\033[33m";
    const std::string COLOR_BLUE = "\033[34m";
    const std::string COLOR_MAGENTA = "\033[35m";
    const std::string COLOR_CYAN = "\033[36m";

    std::string color;
    switch (level)
    {
    case 0:
        color = COLOR_CYAN;
        break;
    case 1:
        color = COLOR_GREEN;
        break;
    case 2:
        color = COLOR_YELLOW;
        break;
    case 3:
        color = COLOR_RED;
        break;
    default:
        color = COLOR_RESET;
    }

    std::cout << color << "[" << file << ":" << line << "] " << comment << COLOR_RESET << std::endl;
    // 输出到文件
    std::ofstream logFile("log.txt", std::ios::app); // 以追加模式打开文件
    if (logFile.is_open())
    {
        // 文件中不包含颜色控制字符
        logFile << "[" << file << ":" << line << "] " << comment << std::endl;
        logFile.close();
    }
    else
    {
        std::cerr << "无法打开日志文件 log.txt" << std::endl;
    }
}

// 重载 showLog 函数，使用可变参数模板
template <typename... Args>
void showLogMulComments(int level = 0, const std::string &file = __FILE__, int line = __LINE__, Args &&...args)
{
    int debugMode = 1;

    if (debugMode == 0)
    {
        return;
    }

    std::ostringstream oss;
    (oss << ... << args);
    std::string comment = oss.str();

    BVH::showLog(comment, level, file, line);
}
BVH::BVHNode *BVH::buildRecursive(int start, int end, std::vector<std::shared_ptr<Shape>> &orderedshapes, std::vector<ShapeInfo> &shapeInfo)
{
    showLogMulComments(0, __FILE__, __LINE__, "开始建立BVH树");
    BVHNode *node = new BVHNode();
    AABB totalBounds;
    showLogMulComments(0, __FILE__, __LINE__, "开始创建包围盒");

    for (int i = start; i < end; i++)
        totalBounds.Expand(shapeInfo[i].bounds);
    showLogMulComments(0, __FILE__, __LINE__, "完成包围盒合并");

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
        // 放置叶子节点时记录顺序
        for (int i = start; i < end; i++)
        {
            orderedshapes.push_back(shapes[shapeInfo[i].geomId]);
        }
    };
    showLogMulComments(0, __FILE__, __LINE__, "开始判断是索引还是叶子节点");

    if (normalStop(1))
    {
        // 叶子节点
        showLogMulComments(0, __FILE__, __LINE__, "开始创建叶子结点");
        normalOpe();
        showLogMulComments(0, __FILE__, __LINE__, "创建了叶子结点");
    }
    else
    {
        showLogMulComments(0, __FILE__, __LINE__, "开始创建索引结点");

        // 非叶子节点，选择分割轴 此时shapeInfo已经排序了
        // 记录本节点中心点形成的总包围盒
        AABB currentCenterBounds;
        for (int i = start; i < end; i++)
            currentCenterBounds.Expand(shapeInfo[i].center);
        // 求切分轴 求切分点
        showLogMulComments(0, __FILE__, __LINE__, "开始寻找最佳切分轴和轴上的最佳切分点");
        BVH::SplitInfo target = getSplitInfo(shapeInfo, currentCenterBounds, {start, end}, longestExtentSplit);
        int splitPoint = target.splitPoint;
        int splitAxis = target.splitAxis;
        node->splitAxis = splitAxis;
        showLogMulComments(0, __FILE__, __LINE__, "计算结束,点为:", splitPoint, "轴为:", splitAxis);

        // 无法划分的情况
        if (splitAxis == -1)
        {
            showLogMulComments(0, __FILE__, __LINE__, "无法划分! 开始重新划入子节点...");
            node->firstShapeOffset = orderedshapes.size();
            node->nShape = end - start;

            for (int i = start; i < end; i++)
            {
                orderedshapes.push_back(shapes[shapeInfo[i].geomId]);
            }
            showLogMulComments(0, __FILE__, __LINE__, "重新划分结束...");
        }

        // 递归构建左右子树
        if (splitPoint != -1)
        {
            showLogMulComments(0, __FILE__, __LINE__, "开始递归构建");

            node->childList.push_back(buildRecursive(start, splitPoint, orderedshapes, shapeInfo));
            node->childList.push_back(buildRecursive(splitPoint, end, orderedshapes, shapeInfo));
        }
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
    showLogMulComments(0, __FILE__, __LINE__, "进入最长中点切分方法");

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
    showLogMulComments(0, __FILE__, __LINE__, "退出最长中点切分方法,计算结果:轴为", splitAxis, "点为", splitPoint);

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
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();
        sceneBox.Expand(shape->getAABB());

        shapeInfoList.push_back(ShapeInfo(count++, shape->getAABB()));
    }
    // 创建待排序数组 初始为空 留待填充
    std::vector<std::shared_ptr<Shape>> orderedshapes;
    std::vector<std::shared_ptr<Shape>> boundingList(shapes.size());

    // 递归构建
    root = buildRecursive(0, shapes.size(), orderedshapes, shapeInfoList);
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
    Point3f origin = ray.origin;
    Vector3f direction = ray.direction;
    Ray R(ray);
    // 和树求交
    // int currentNode = 0;
    std::function<bool(BVHNode *)> traverse = [&](BVHNode *node) -> bool
    {
        Vector3f invDir(1 / ray.direction[0], 1 / ray.direction[1], 1 / ray.direction[2]);
        bool isNegDir[3] = {ray.direction[0] < 0, ray.direction[1] < 0, ray.direction[2] < 0};
        bool flag = false;
        // 没东西直接返回
        if (!node)
            return flag;
        // 相交了
        if (node->boundingBox.RayIntersect(R))
        {

            if (node->splitAxis == -1)
            {
                // 是子节点
                if (root->nShape > 0)
                {
                    for (int i = 0; i < node->nShape; i++)
                    {
                        int idx = node->firstShapeOffset + i;

                        if (!shapes.empty())
                            flag = shapes[idx]->rayIntersectShape(R, primID, u, v);
                    }
                }
                else
                {
                    // 如果是多叉树需要按照交点距离排序再遍历
                    // 是索引节点
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
            // 没相交啥也不用干
        }
        return flag;
    };
    return traverse(root);
}