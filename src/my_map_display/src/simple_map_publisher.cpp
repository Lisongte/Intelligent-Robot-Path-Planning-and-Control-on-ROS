#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_map_publisher");
    ros::NodeHandle nh;

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_map", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 生成几个柱状障碍物
    std::vector<Eigen::Vector3d> obstacles;
    obstacles.push_back(Eigen::Vector3d(1.0, 1.0, 0.8));   // 靠近起点，但保持安全距离
    obstacles.push_back(Eigen::Vector3d(2.0, 1.2, 1.5));   // 中部偏右
    obstacles.push_back(Eigen::Vector3d(1.2, 2.2, 1.0));   // 中部偏上
    obstacles.push_back(Eigen::Vector3d(2.5, 2.5, 0.6));   // 靠近终点，但留出空间

    double resolution = 0.1; // 点云分辨率

    // 生成每个障碍物（圆柱体）
    for (const auto& obs : obstacles)
    {
        double radius = 0.5;
double height = obs.z();
int radius_cells = static_cast<int>(std::ceil(radius / resolution));

// 使用整数循环遍历格子
for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        // 计算格子中心相对于障碍物中心的偏移
        double x_offset = (dx + 0.5) * resolution;
        double y_offset = (dy + 0.5) * resolution;
        // 检查是否在圆内（使用平方比较避免开方）
        if (x_offset*x_offset + y_offset*y_offset <= radius*radius) {
            double world_x = obs.x() + x_offset;
            double world_y = obs.y() + y_offset;
            // 生成从地面到高度的所有点
            for (double z = 0; z <= height; z += resolution) {
                pcl::PointXYZ pt;
                pt.x = world_x;
                pt.y = world_y;
                pt.z = z;
                cloud.push_back(pt);
            }
        }
    }
}
    }

    // 添加一个地面平面（可选）
    for (double x = -0.5; x <= 3.5; x += resolution)
        for (double y = -0.5; y <= 3.5; y += resolution)
        {
            pcl::PointXYZ pt;
            pt.x = x;
            pt.y = y;
            pt.z = 0.0;
            cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    ROS_INFO("Generated cloud with %lu points", cloud.points.size());

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok())
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "world";
        output.header.stamp = ros::Time::now();

        map_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}