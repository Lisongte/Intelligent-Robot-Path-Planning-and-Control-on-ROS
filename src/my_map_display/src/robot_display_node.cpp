#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_display");
    ros::NodeHandle nh;

    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::MarkerArray>("robot_marker", 1);

    ros::Rate rate(10);

    while (ros::ok())
    {
        visualization_msgs::MarkerArray robot;

        // ===================== 车身 =====================
        visualization_msgs::Marker body;
        body.header.frame_id = "world";
        body.header.stamp = ros::Time::now();
        body.ns = "robot";
        body.id = 0;
        body.type = visualization_msgs::Marker::CUBE;
        body.action = visualization_msgs::Marker::ADD;

        body.pose.position.x = 0.0;
        body.pose.position.y = 0.0;
        body.pose.position.z = 0.12;

        body.pose.orientation.w = 1.0;

        body.scale.x = 0.4;
        body.scale.y = 0.25;
        body.scale.z = 0.12;

        body.color.r = 0.2;
        body.color.g = 0.6;
        body.color.b = 1.0;
        body.color.a = 1.0;

        robot.markers.push_back(body);

        // ===================== 四个轮子 =====================
        for(int i=0;i<4;i++)
        {
            visualization_msgs::Marker wheel;
            wheel.header.frame_id = "world";
            wheel.header.stamp = ros::Time::now();
            wheel.ns = "robot";
            wheel.id = i+1;
            wheel.type = visualization_msgs::Marker::CUBE;
            wheel.action = visualization_msgs::Marker::ADD;

            wheel.scale.x = 0.08;
            wheel.scale.y = 0.04;
            wheel.scale.z = 0.08;

            wheel.color.r = 0.1;
            wheel.color.g = 0.1;
            wheel.color.b = 0.1;
            wheel.color.a = 1.0;

            wheel.pose.orientation.w = 1.0;

            wheel.pose.position.x = (i<2)?0.12:-0.12;
            wheel.pose.position.y = (i%2==0)?0.15:-0.15;
            wheel.pose.position.z = 0.04;

            robot.markers.push_back(wheel);
        }

        // ===================== 车头箭头 =====================
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "robot";
        arrow.id = 10;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        arrow.pose.position.x = 0.2;
        arrow.pose.position.y = 0.0;
        arrow.pose.position.z = 0.18;

        arrow.pose.orientation.w = 1.0;

        arrow.scale.x = 0.15;
        arrow.scale.y = 0.03;
        arrow.scale.z = 0.03;

        arrow.color.r = 1.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        robot.markers.push_back(arrow);

        marker_pub.publish(robot);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}