#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>

// Simple coordinate object to represent points and vectors
struct XY
{
    float x;
    float y;
    int NodeId;
    bool *serviced;

    XY(float x, float y, int _ID, bool *_serviced ) : x(x), y(y), NodeId(_ID), serviced(_serviced){ }
    XY() : x(0), y(0), NodeId(0){}
};

// Axis-aligned bounding box with half dimension and center
struct AABB
{
    XY center;
    int halfDimension;

    AABB(XY center, int halfDimension) : center(center), halfDimension(halfDimension) {}

    bool containsPoint(XY point);
    bool intersectsAABB(AABB other);
};

// QuadTree
class QuadTree{
    private:
        const int QT_CAPACITY = 10;
        AABB boundary;
        bool divided;
        std::vector<XY> points;
        QuadTree* northWest;
        QuadTree* northEast;
        QuadTree* southWest;
        QuadTree* southEast;

    public:
        inline bool static TRUE_VALUE = true;

        QuadTree(AABB _boundary) : boundary(_boundary), points(0, XY(0,0,0, &TRUE_VALUE)), divided(false) {}
        QuadTree() : boundary(XY(50,50,0, &TRUE_VALUE), 50), points(0, XY(0,0,0, &TRUE_VALUE)), divided(false) {}

        // METHODS
        bool insert(XY point);
        void subdivide();
        void query(AABB range, std::vector<XY> *found);
        std::vector<XY> getPoints();
};  

#endif