#include "my_map_display/astar_planner.h"
#include <functional>
#include <algorithm>
#include <ros/ros.h>

AStarPlanner::AStarPlanner(const OccupancyGrid& grid) : grid_(grid) {}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const
{
    double dx = (x1 - x2) * grid_.getResolution();
    double dy = (y1 - y2) * grid_.getResolution();
    return std::sqrt(dx*dx + dy*dy);
}

bool AStarPlanner::isFree(int x, int y) const
{
    return !grid_.isOccupied(x, y);
}

bool AStarPlanner::plan(double start_x, double start_y, double goal_x, double goal_y)
{
    int start_ix, start_iy, goal_ix, goal_iy;
    if (!grid_.worldToGrid(start_x, start_y, start_ix, start_iy) ||
        !grid_.worldToGrid(goal_x, goal_y, goal_ix, goal_iy))
    {
        ROS_ERROR("Start or goal is outside the grid!");
        return false;
    }

    if (!isFree(start_ix, start_iy) || !isFree(goal_ix, goal_iy))
    {
        ROS_ERROR("Start or goal is occupied!");
        return false;
    }

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<int, std::unordered_map<int, double>> best_g;
    all_nodes_.clear();

    Node start_node(start_ix, start_iy);
    start_node.g = 0;
    start_node.h = heuristic(start_ix, start_iy, goal_ix, goal_iy);
    open_set.push(start_node);
    best_g[start_ix][start_iy] = start_node.g;
    all_nodes_[start_ix][start_iy] = start_node;

    const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};

    while (!open_set.empty())
    {
        Node current = open_set.top();
        open_set.pop();

        if (current.x == goal_ix && current.y == goal_iy)
        {
            reconstructPath(current);
            return true;
        }

        if (current.g > best_g[current.x][current.y])
            continue;

        for (int i = 0; i < 8; ++i)
        {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx < 0 || nx >= grid_.getWidth() || ny < 0 || ny >= grid_.getHeight())
                continue;
            if (!isFree(nx, ny))
                continue;

            double step_cost;
            if (dx[i] != 0 && dy[i] != 0)
                step_cost = std::sqrt(2) * grid_.getResolution();
            else
                step_cost = grid_.getResolution();

            double tentative_g = current.g + step_cost;

            if (best_g.find(nx) == best_g.end() || best_g[nx].find(ny) == best_g[nx].end() ||
                tentative_g < best_g[nx][ny])
            {
                best_g[nx][ny] = tentative_g;
                Node neighbor(nx, ny);
                neighbor.g = tentative_g;
                neighbor.h = heuristic(nx, ny, goal_ix, goal_iy);
                neighbor.came_from_x = current.x;
                neighbor.came_from_y = current.y;
                open_set.push(neighbor);
                all_nodes_[nx][ny] = neighbor;
            }
        }
    }

    ROS_WARN("A* failed to find a path!");
    return false;
}

void AStarPlanner::reconstructPath(const Node& goal_node)
{
    path_.clear();
    Node current = goal_node;
    while (current.came_from_x != -1)
    {
        double wx = grid_.getOriginX() + (current.x + 0.5) * grid_.getResolution();
        double wy = grid_.getOriginY() + (current.y + 0.5) * grid_.getResolution();
        path_.push_back(Eigen::Vector2d(wx, wy));

        int px = current.came_from_x;
        int py = current.came_from_y;
        auto it_x = all_nodes_.find(px);
        if (it_x == all_nodes_.end()) break;
        auto it_y = it_x->second.find(py);
        if (it_y == it_x->second.end()) break;
        current = it_y->second;
    }
    double start_wx = grid_.getOriginX() + (current.x + 0.5) * grid_.getResolution();
    double start_wy = grid_.getOriginY() + (current.y + 0.5) * grid_.getResolution();
    path_.push_back(Eigen::Vector2d(start_wx, start_wy));

    std::reverse(path_.begin(), path_.end());
}