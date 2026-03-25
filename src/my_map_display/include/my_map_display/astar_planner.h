#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "my_map_display/occupancy_grid.h"
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Eigen>
#include <unordered_map>

struct Node
{
    int x, y;
    double g;
    double h;
    int came_from_x, came_from_y;

    Node(int ix = 0, int iy = 0) : x(ix), y(iy), g(0), h(0), came_from_x(-1), came_from_y(-1) {}
    double f() const { return g + h; }
    bool operator>(const Node& other) const { return f() > other.f(); }
};

class AStarPlanner
{
public:
    AStarPlanner(const OccupancyGrid& grid);
    bool plan(double start_x, double start_y, double goal_x, double goal_y);
    std::vector<Eigen::Vector2d> getPath() const { return path_; }

private:
    const OccupancyGrid& grid_;
    std::vector<Eigen::Vector2d> path_;
    std::unordered_map<int, std::unordered_map<int, Node>> all_nodes_;

    double heuristic(int x1, int y1, int x2, int y2) const;
    bool isFree(int x, int y) const;
    void reconstructPath(const Node& goal_node);
};

#endif