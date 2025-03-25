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
        // 叶子结点存储开始
        int firstShapeOffset;
        // 叶子结点存储了几个对象
        int nShape = 0;
    };
    struct SplitInfo
    {
        int splitPoint;
        // 叶子结点 -1 XYZ对应0 1 2
        int splitAxis;
    };
    BVH() = default;
    void build() override;
    bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;

    // 递归构造BVH树
    BVHNode *buildRecursive(int start, int end, std::vector<std::shared_ptr<Shape>> &orderedshapes, std::vector<std::shared_ptr<Shape>> &shapeInfo, std::vector<std::shared_ptr<Shape>> boundingList);
    // 接口函数，接收回调函数指针 用于排序并返回切分点和轴
    using SplitCallback = SplitInfo (*)(const std::vector<std::shared_ptr<Shape>> &);
    SplitInfo getSplitInfo(const std::vector<std::shared_ptr<Shape>> &boundingList, SplitCallback callback);

protected:
    static constexpr int bvhLeafMaxSize = 64;
    struct BVHNode;
    BVHNode *root;
};