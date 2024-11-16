#include <iostream>
#include <vector>
#include <unordered_set>
#include <random>
#include "Point.h"
#include "Data.h"
#include "SSTree.h"

constexpr size_t NUM_POINTS = 10000;
constexpr size_t MAX_POINTS_PER_NODE = 20;

/*
 * Helper functions
 */
std::vector<Data*> generateRandomData(size_t numPoints) {
    std::vector<Data*> data;
    for (size_t i = 0; i < numPoints; ++i) {
        Point embedding = Point::random();
        std::string imagePath = "eda_" + std::to_string(i) + ".jpg";
        Data* dataPoint = new Data(embedding, imagePath);
        data.push_back(dataPoint);
    }
    return data;
}

void collectDataDFS(SSNode* node, std::unordered_set<Data*>& treeData) {
    if (node->getIsLeaf()) {
        for (const auto& d : node->getData()) {
            treeData.insert(d);
        }
    } else {
        for (const auto& child : node->getChildren()) {
            collectDataDFS(child, treeData);
        }
    }
}

/*
 * Testing functions
 */

// Test 1: Check if all data is present in the tree
bool allDataPresent(const SSTree& tree, const std::vector<Data*>& data) {
    std::unordered_set<Data*> dataSet(data.begin(), data.end());
    std::unordered_set<Data*> treeData;

    collectDataDFS(tree.getRoot(), treeData);
    for (const auto& d : dataSet) {
        if (treeData.find(d) == treeData.end()) {
            return false;
        }
    }
    for (const auto& d : treeData) {
        if (dataSet.find(d) == dataSet.end()) {
            return false;
        }
    }
    return true;
}

// Test 2: Check if all leaves are at the same level
bool leavesAtSameLevelDFS(SSNode* node, int level, int& leafLevel) {
    if (node->getIsLeaf()) {
        if (leafLevel == -1) leafLevel = level;
        return leafLevel == level;
    }
    for (const auto& child : node->getChildren()) {
        if (!leavesAtSameLevelDFS(child, level + 1, leafLevel)) return false;
    }
    return true;
}

bool leavesAtSameLevel(SSNode* root) {
    int leafLevel = -1;
    return leavesAtSameLevelDFS(root, 0, leafLevel);
}

// Test 3: Check if no node exceeds the maximum number of children
bool noNodeExceedsMaxChildrenDFS(SSNode* node, size_t maxPointsPerNode) {
    if (node->getChildren().size() > maxPointsPerNode) return false;
    for (const auto& child : node->getChildren()) {
        if (!noNodeExceedsMaxChildrenDFS(child, maxPointsPerNode)) return false;
    }
    return true;
}

bool noNodeExceedsMaxChildren(SSNode* root, size_t maxPointsPerNode) {
    return noNodeExceedsMaxChildrenDFS(root, maxPointsPerNode);
}

// Test 4: Check if all points are inside the bounding sphere of their respective nodes
bool sphereCoversAllPointsDFS(SSNode* node) {
    if (!node->getIsLeaf()) return true;
    const Point& centroid = node->getCentroid();
    float radius = node->getRadius();
    for (const auto& data : node->getData()) {
        if (Point::distance(centroid, data->getEmbedding()) > radius) return false;
    }
    return true;
}

bool dfsSphereCoversAllPoints(SSNode* node) {
    if (node->getIsLeaf()) {
        return sphereCoversAllPointsDFS(node);
    } else {
        for (const auto& child : node->getChildren()) {
            if (!dfsSphereCoversAllPoints(child)) return false;
        }
    }
    return true;
}

bool sphereCoversAllPoints(SSNode* root) {
    return dfsSphereCoversAllPoints(root);
}

// Test 5: Check if all children are inside the bounding sphere of their parent node
bool sphereCoversAllChildrenSpheresDFS(SSNode* node) {
    if (node->getIsLeaf()) return true;
    const Point& centroid = node->getCentroid();
    float radius = node->getRadius();
    for (const auto& child : node->getChildren()) {
        const Point& childCentroid = child->getCentroid();
        float childRadius = child->getRadius();
        if (Point::distance(centroid, childCentroid) + childRadius > radius) return false;
    }
    return true;
}

bool dfsSphereCoversAllChildrenSpheres(SSNode* node) {
    if (!sphereCoversAllChildrenSpheresDFS(node)) return false;
    for (const auto& child : node->getChildren()) {
        if (!dfsSphereCoversAllChildrenSpheres(child)) return false;
    }
    return true;
}

bool sphereCoversAllChildrenSpheres(SSNode* root) {
    return dfsSphereCoversAllChildrenSpheres(root);
}

int main() {
    auto data = generateRandomData(NUM_POINTS);
    SSTree tree(MAX_POINTS_PER_NODE);
    for (const auto& d : data) {
        tree.insert(d);
    }

    // Realizar pruebas
    bool allPresent     = allDataPresent(tree, data);
    bool sameLevel      = leavesAtSameLevel(tree.getRoot());
    bool noExceed       = noNodeExceedsMaxChildren(tree.getRoot(), MAX_POINTS_PER_NODE);
    bool spherePoints   = sphereCoversAllPoints(tree.getRoot());
    bool sphereChildren = sphereCoversAllChildrenSpheres(tree.getRoot());
    
    std::cout << "Todos los datos presentes: " << (allPresent ? "Sí" : "No") << std::endl;
    std::cout << "Nodos hojas en el mismo nivel: " << (sameLevel ? "Sí" : "No") << std::endl;
    std::cout << "No se supera el límite de hijos por nodo: " << (noExceed ? "Sí" : "No") << std::endl;
    std::cout << "Hiper-esfera cubre todos los puntos de los nodos hoja: " << (spherePoints ? "Sí" : "No") << std::endl;
    std::cout << "Hiper-esfera cubre todas las hiper-esferas internas de los nodos internos: " << (sphereChildren ? "Sí" : "No") << std::endl;
    std::cout << "Happy ending! :D" << std::endl;

    return 0;
}
