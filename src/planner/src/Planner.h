#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <Eigen/Core>

#include "Node.h"


struct CompareNode {
    bool operator()(Node* const& a, Node* const& b) {
        return a->getFCost() > b->getFCost();
    }
};

class Planner
{
private:
    int width;
    int height;
    float resolution;
    Eigen::Vector2i start;
    Eigen::Vector2i end;

public:
    std::vector<std::vector<Node>> map;

    Planner(); // null and 0 init
    void Set(const std::vector<signed char>& intMap, int width, int height, int threshold,float resolution); // store data and convert 1D int map to 2D bool map
    void AddStart(const Eigen::Vector2i& start);
    void AddEnd(const Eigen::Vector2i& end);
    std::vector<Eigen::Vector2f> PlanPath();
    std::vector<Eigen::Vector2f> SmoothPath(const std::vector<Eigen::Vector2f>& path);
};

#endif // PLANNER_H
