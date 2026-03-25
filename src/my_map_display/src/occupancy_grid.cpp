#include "my_map_display/occupancy_grid.h"
#include <iostream>
#include <ros/ros.h>
#include <cmath>

OccupancyGrid::OccupancyGrid(double min_x, double max_x, double min_y, double max_y,
                             double resolution, double robot_radius)
    : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y),
      resolution_(resolution), robot_radius_(robot_radius)
{
    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));
    grid_.resize(width_, std::vector<bool>(height_, false));
    ROS_INFO("OccupancyGrid created: width=%d, height=%d, range x=[%.2f,%.2f] y=[%.2f,%.2f]",
             width_, height_, min_x_, max_x_, min_y_, max_y_);
}

void OccupancyGrid::buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double ground_threshold)
{
    // 清空栅格
    for (auto& row : grid_)
        std::fill(row.begin(), row.end(), false);

    int inflate = static_cast<int>(std::ceil(robot_radius_ / resolution_));
    int obstacle_points = 0;
    int marked_cells = 0;
    int failed = 0;

    for (const auto& point : cloud->points)
    {
        // 忽略地面点（z 小于阈值）
        if (point.z < ground_threshold)
            continue;

        obstacle_points++;

        int ix, iy;
        if (!worldToGrid(point.x, point.y, ix, iy))
        {
            failed++;
            // 只打印前10个失败点的坐标，避免刷屏
            if (failed <= 10)
                ROS_WARN("Point (%.2f, %.2f, %.2f) out of grid range [%.2f,%.2f] x [%.2f,%.2f]",
                         point.x, point.y, point.z, min_x_, max_x_, min_y_, max_y_);
            continue;
        }

        // 标记障碍物及其膨胀区域
        for (int dx = -inflate; dx <= inflate; ++dx)
            for (int dy = -inflate; dy <= inflate; ++dy)
            {
                int nx = ix + dx;
                int ny = iy + dy;
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
                {
                    double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
                    if (dist <= robot_radius_ + resolution_/2)
                    {
                        if (!grid_[nx][ny])
                        {
                            grid_[nx][ny] = true;
                            marked_cells++;
                        }
                    }
                }
            }
    }

    ROS_INFO("Obstacle points: %d, failed: %d, marked cells: %d", obstacle_points, failed, marked_cells);
}

bool OccupancyGrid::isOccupied(int ix, int iy) const
{
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_)
        return true;  // 超出边界视为占用（不可通行）
    return grid_[ix][iy];
}

bool OccupancyGrid::worldToGrid(double x, double y, int& ix, int& iy) const
{
    ix = static_cast<int>(std::floor((x - min_x_) / resolution_));
    iy = static_cast<int>(std::floor((y - min_y_) / resolution_));
    return (ix >= 0 && ix < width_ && iy >= 0 && iy < height_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OccupancyGrid::toPointCloud() const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int ix = 0; ix < width_; ++ix)
        for (int iy = 0; iy < height_; ++iy)
            if (grid_[ix][iy])
            {
                pcl::PointXYZ pt;
                pt.x = min_x_ + (ix + 0.5) * resolution_;
                pt.y = min_y_ + (iy + 0.5) * resolution_;
                pt.z = 0.0;
                cloud->push_back(pt);
            }
    cloud->width = cloud->size();
    cloud->height = 1;
    return cloud;
}