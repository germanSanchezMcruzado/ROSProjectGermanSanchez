#include "Planner.h"
#include <cmath>
#include <queue>

Planner::Planner() : width(0), height(0) {
}

void Planner::Set(const std::vector<signed char>& intMap, int width, int height, int threshold, float resolution) { 
    this->resolution = resolution;
    this->width = width;
    this->height = height;

    map.resize(this->height);
    for (int i = 0; i < this->height; i++) {
        map[i].resize(this->width);
        for (int j = 0; j < this->width; j++) {
            map[i][j] = Node(0, 0, nullptr, intMap[i * this->width + j] > threshold, j, i);
        }
    }
}

void Planner::AddStart(const Eigen::Vector2i& start) {
    int start_x = start.x() / this->resolution;
    int start_y = start.y() / this->resolution;
    this->start = Eigen::Vector2i(start_x, start_y);
}

void Planner::AddEnd(const Eigen::Vector2i& end) {
    int end_x = end.x() / this->resolution;
    int end_y = end.y() / this->resolution;
    this->end = Eigen::Vector2i(end_x, end_y);
}

std::vector<Eigen::Vector2f> Planner::PlanPath() {
    // Initialization
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open; // Use priority queue for open list
    std::vector<Node*> closed;

    Node* startNode = &map[start.y()][start.x()];
    Node* endNode = &map[end.y()][end.x()];

    // Recalculate g and h costs and reset parents
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            Node& node = map[i][j];
            node.gcost = std::numeric_limits<float>::infinity(); // Use infinity for uninitialized g costs
            float dx = std::abs(node.x - endNode->x);
            float dy = std::abs(node.y - endNode->y);
            node.hcost = dx + dy; // Use Manhattan distance for heuristic
            node.parent = nullptr; // Reset parent pointer
        }
    }

    startNode->gcost = 0; // Start node has g cost of 0
    open.push(startNode);

    while (!open.empty()) {
        // Find node with the lowest f cost
        Node* current = open.top();
        open.pop();

        if (current == endNode) {
            // Reconstruct path
            std::vector<Eigen::Vector2i> path;
            while (current != nullptr) {
                int x = current->x;
                int y = current->y;
                if (x != -1 && y != -1) {
                    path.push_back(Eigen::Vector2i(x, y));
                }
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            std::vector<Eigen::Vector2f> returning;
            int c = -1;
            int inv = 5;
            for (const auto& pos : path) {
                c++;
                if(c % inv != 0) continue;
                returning.push_back(Eigen::Vector2f(pos.x() * this->resolution, pos.y() * this->resolution));
            }

            // Smooth the path
            return SmoothPath(returning);
        }

        closed.push_back(current); // Add current node to closed list

        // Generate neighbors
        std::vector<Node*> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue; // Skip the current node
                
                int nx = current->x + dx;
                int ny = current->y + dy;

                if (nx >= 0 && nx < width && ny >= 0 && ny < height && !map[ny][nx].blocked) {
                    neighbors.push_back(&map[ny][nx]);
                }
            }
        }


        // Process each neighbor
        for (Node* neighbor : neighbors) {
            if (std::find(closed.begin(), closed.end(), neighbor) != closed.end()) {
                continue; // Skip neighbor if it's already in the closed list
            }
            float dx = neighbor->x - current->x;
            float dy = neighbor->y - current->y;
            float squaredDistance = dx * dx + dy * dy;
            float tentative_gcost = current->gcost + std::sqrt(squaredDistance); 
            if (tentative_gcost < neighbor->gcost) {
                neighbor->gcost = tentative_gcost;
                neighbor->parent = current;
                open.push(neighbor); // Add neighbor to open list
            }
        }
    }

    // No path found
    return std::vector<Eigen::Vector2f>();
}

std::vector<Eigen::Vector2f> Planner::SmoothPath(const std::vector<Eigen::Vector2f>& path) {
    std::vector<Eigen::Vector2f> smoothedPath;

    if (path.empty()) {
        return smoothedPath; // Return empty path if input path is empty
    }

    const float minDistanceSq = 0.2 * 0.2; // Minimum squared distance between points (adjust as needed)

    smoothedPath.push_back(path.front()); // Always keep the first point

    for (size_t i = 1; i < path.size(); ++i) {
        // Compute squared distance between current point and last point in smoothed path
        float dx = path[i].x() - smoothedPath.back().x();
        float dy = path[i].y() - smoothedPath.back().y();
        float distanceSq = dx * dx + dy * dy;

        // If squared distance is greater than threshold, add current point to smoothed path
        if (distanceSq >= minDistanceSq) {
            smoothedPath.push_back(path[i]);
        }
    }

    return smoothedPath;
}





