#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include "my_map_display/occupancy_grid.h"
#include "my_map_display/astar_planner.h"

class PathPlannerNode
{
public:
    PathPlannerNode() : nh_("~"), path_published_(false)
    {
        cloud_sub_ = nh_.subscribe("/my_map", 1, &PathPlannerNode::cloudCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);   // latch=true
        grid_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/occupancy_grid", 1);

        nh_.param("map_min_x", min_x_, -0.5);
        nh_.param("map_max_x", max_x_, 3.5);
        nh_.param("map_min_y", min_y_, -0.5);
        nh_.param("map_max_y", max_y_, 3.5);

        nh_.param("resolution", resolution_, 0.1);
        nh_.param("robot_radius", robot_radius_, 0.0);

        nh_.param("start_x", start_x_, 0.0);
        nh_.param("start_y", start_y_, 0.0);

        nh_.param("goal_x", goal_x_, 3.0);
        nh_.param("goal_y", goal_y_, 3.0);

        ROS_INFO("Path planner node initialized.");
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 已经规划过就不再重复规划
        if (path_published_)
            return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        if (cloud->empty())
        {
            ROS_WARN("Received empty cloud.");
            return;
        }

        OccupancyGrid grid(min_x_, max_x_, min_y_, max_y_, resolution_, robot_radius_);
        grid.buildFromPointCloud(cloud);

        // 发布栅格点云（调试用）
        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud = grid.toPointCloud();

        sensor_msgs::PointCloud2 grid_msg;
        pcl::toROSMsg(*grid_cloud, grid_msg);

        grid_msg.header.frame_id = "world";
        grid_msg.header.stamp = ros::Time::now();

        grid_pub_.publish(grid_msg);

        // A* 路径规划
        AStarPlanner planner(grid);

        if (planner.plan(start_x_, start_y_, goal_x_, goal_y_))
        {
            std::vector<Eigen::Vector2d> path_world = planner.getPath();

            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "world";
            path_msg.header.stamp = ros::Time::now();

            for (const auto& pt : path_world)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "world";
                pose.header.stamp = ros::Time::now();

                pose.pose.position.x = pt.x();
                pose.pose.position.y = pt.y();
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                path_msg.poses.push_back(pose);
            }

            path_pub_.publish(path_msg);

            path_published_ = true;

            ROS_INFO("Path published once with %lu poses.", path_msg.poses.size());
        }
        else
        {
            ROS_ERROR("Failed to find a path.");
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber cloud_sub_;
    ros::Publisher path_pub_;
    ros::Publisher grid_pub_;

    double min_x_, max_x_, min_y_, max_y_;
    double resolution_, robot_radius_;

    double start_x_, start_y_;
    double goal_x_, goal_y_;

    bool path_published_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_node");

    PathPlannerNode node;

    ros::spin();

    return 0;
}