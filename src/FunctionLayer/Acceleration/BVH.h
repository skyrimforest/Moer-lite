#pragma once
#include "Acceleration.h"

class BVH : public Acceleration
{
public:
    struct BVHNode
    {
        //* BVH节点结构设计
        // 叶子结点 -1 XYZ对应0 1 2
        int splitAxis;
        // 索引节点存储后代
        std::vector<BVHNode *> childList;
        // 索引节点存储包围盒
        std::vector<std::shared_ptr<Shape>> boundingList;
        // 包围盒
        AABB boundingBox;
        // 叶子结点存储开始
        int firstShapeOffset;
        // 叶子结点存储了几个对象 如果是索引节点记为0
        int nShape = 0;
    };
    struct SplitInfo
    {
        int splitPoint;
        // 叶子结点 -1 XYZ对应0 1 2
        int splitAxis;
    };
    struct ShapeInfo
    {
        ShapeInfo() {}
        ShapeInfo(int _EntityId, const AABB &_bounds) : geomId(_EntityId), bounds(_bounds), center(_bounds.Center()) {}
        int geomId;
        AABB bounds;
        Point3f center;
    };
    struct LinearBvhNode
    {
        AABB boundingBox;
        union
        {
            int firstdEntityOffset; // for leaves to enumerate
            int secondChildOrder;   // for interior nodes to traverse
        };
        int nEntites = 0;
        int splitAxis;
    };
    BVH() = default;
    void build() override;
    bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;

    // 递归构造BVH树
    BVHNode *buildRecursive(int start, int end, std::vector<std::shared_ptr<Shape>> &orderedshapes, std::vector<ShapeInfo> &shapeInfo);
    // 接口函数，接收回调函数指针 用于排序并返回切分点和轴
    using SplitCallback = SplitInfo (*)(std::vector<ShapeInfo> &, AABB &, std::pair<int, int> interval);
    SplitInfo getSplitInfo(std::vector<ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval, SplitCallback callback);
    // DFS序展开Bvh
    void Flatten(std::shared_ptr<BVHNode> node, int &dfsOrder);
    static BVH::SplitInfo sahSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval);
    static BVH::SplitInfo equalShapeSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval);
    static BVH::SplitInfo longestExtentSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval);
    static BVH::SplitInfo staticSplit(std::vector<BVH::ShapeInfo> &shapeInfo, AABB &centerBound, std::pair<int, int> interval);
    static void showLog(std::string comment, int level, const std::string &file, int line);

protected:
    std::vector<LinearBvhNode> linearBvhNodes;
    static constexpr int bvhLeafMaxSize = 64;
    BVHNode *root;
};