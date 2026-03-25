#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

class OccupancyGrid
{
public:
    OccupancyGrid(double min_x, double max_x, double min_y, double max_y, 
                  double resolution, double robot_radius);
    void buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double ground_threshold = 0.05);
    bool isOccupied(int ix, int iy) const;
    bool worldToGrid(double x, double y, int& ix, int& iy) const;
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return min_x_; }
    double getOriginY() const { return min_y_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud() const;

private:
    double min_x_, max_x_, min_y_, max_y_;
    double resolution_;
    double robot_radius_;
    int width_, height_;
    std::vector<std::vector<bool>> grid_;
};

#endif