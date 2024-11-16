#ifndef SSTREE_H
#define SSTREE_H

#include <vector>
#include <limits>
#include <algorithm>
#include <numeric>
#include "Point.h"
#include "Data.h"

class SSNode {
private:
    size_t maxPointsPerNode;
    Point centroid;
    float radius;
    bool isLeaf;
    SSNode* parent;
    std::vector<SSNode*> children;
    std::vector<Data*> _data;
    
    // For searching
    SSNode* findClosestChild(const Point& target);

    // For insertion
    void updateBoundingEnvelope();
    size_t directionOfMaxVariance();
    SSNode* split();
    size_t findSplitIndex(size_t coordinateIndex);
    std::vector<Point> getEntriesCentroids();
    size_t minVarianceSplit(const std::vector<float>& values);
    
public:
    SSNode(const Point& centroid, float radius=0.0f, bool isLeaf=true, SSNode* parent=nullptr)
        : centroid(centroid), radius(radius), isLeaf(isLeaf), parent(parent) {}

    // Checks if a point is inside the bounding sphere
    bool intersectsPoint(const Point& point) const;

    // Getters
    const Point& getCentroid() const { return centroid; }
    float getRadius() const { return radius; }
    const std::vector<SSNode*>& getChildren() const { return children; }
    const std::vector<Data  *>& getData    () const { return    _data; }
    bool getIsLeaf() const { return isLeaf; }
    SSNode* getParent() const { return parent; }

    // Insertion
    SSNode* searchParentLeaf(SSNode* node, const Point& target);
    SSNode* insert(SSNode* node, Data* _data);

    // Search
    SSNode* search(SSNode* node, Data* _data);
};

class SSTree {
private:
    SSNode* root;
    size_t maxPointsPerNode;

public:
    SSTree(size_t maxPointsPerNode)
        : root(nullptr), maxPointsPerNode(maxPointsPerNode) {}

    void insert(Data* _data);
    SSNode* search(Data* _data);
    SSNode* getRoot() const;
};

#endif // SSTREE_H
