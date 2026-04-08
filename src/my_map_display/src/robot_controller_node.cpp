#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>

struct Pose2D {
    double x;
    double y;
    double theta;
};

class RobotController {
public:
    RobotController() : nh_("~") {
        // 发布机器人 Marker
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("robot_marker", 1);

        // 订阅规划路径
        path_sub = nh_.subscribe("/planned_path", 1, &RobotController::pathCallback, this);

        // 控制器参数
        nh_.param("Kp_dist", Kp_dist_, 1.0);
        nh_.param("Kp_angle", Kp_angle_, 3.0);
        nh_.param("wheel_base", wheel_base_, 0.4);
        nh_.param("arrival_tolerance", tol_, 0.05);
        nh_.param("control_freq", control_freq_, 20.0);

        // 初始机器人状态
        nh_.param("init_x", robot_pose_.x, 0.0);
        nh_.param("init_y", robot_pose_.y, 0.0);
        nh_.param("init_theta", robot_pose_.theta, 0.0);

        // 定时器
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_freq_),
                                 &RobotController::controlLoop, this);

        last_time_ = ros::Time(0);
        current_target_idx_ = 0;

        ROS_INFO("Robot controller started: Kp_dist=%.2f, Kp_angle=%.2f, wheel_base=%.2f, freq=%.1f",
                 Kp_dist_, Kp_angle_, wheel_base_, control_freq_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub;
    ros::Subscriber path_sub;
    ros::Timer timer_;

    Pose2D robot_pose_;
    std::vector<Pose2D> path_points_;
    int current_target_idx_;

    double Kp_dist_, Kp_angle_, wheel_base_, tol_, control_freq_;
    ros::Time last_time_;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        path_points_.clear();
        for (const auto &pose_stamped : msg->poses) {
            Pose2D p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            p.theta = 0.0;
            path_points_.push_back(p);
        }
        current_target_idx_ = 0;
        ROS_INFO("Received new path with %zu points", path_points_.size());
    }

    

    void controlLoop(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    double dt = (last_time_.isZero()) ? (1.0 / control_freq_) : (now - last_time_).toSec();
    last_time_ = now;

    if (path_points_.empty() || current_target_idx_ >= path_points_.size()) {
        publishRobotMarker();
        return;
    }

    // 当前目标点
    const Pose2D &target = path_points_[current_target_idx_];
    double dx = target.x - robot_pose_.x;
    double dy = target.y - robot_pose_.y;
    double dist = std::hypot(dx, dy);

    // 到达目标点判断
    if (dist < tol_) {
        robot_pose_.x = target.x;  // 修正位置
        robot_pose_.y = target.y;
        // 朝向收敛
        double desired_theta = 0.0; // 可以设为0或者路径最后方向
        double alpha = std::atan2(std::sin(desired_theta - robot_pose_.theta),
                                  std::cos(desired_theta - robot_pose_.theta));
        double w = Kp_angle_ * alpha * 0.5; // 缓慢收敛
        // 更新朝向
        robot_pose_.theta += w * dt;
        robot_pose_.theta = std::atan2(std::sin(robot_pose_.theta), std::cos(robot_pose_.theta));

        ROS_INFO_THROTTLE(1.0, "Reached target[%d]: x=%.2f y=%.2f theta=%.2f",
                          current_target_idx_, robot_pose_.x, robot_pose_.y, robot_pose_.theta);

        current_target_idx_++;
        publishRobotMarker();
        return;
    }

    // 远离目标点时，正常Unicycle控制器
    double desired_theta = std::atan2(dy, dx);
    double alpha = std::atan2(std::sin(desired_theta - robot_pose_.theta),
                              std::cos(desired_theta - robot_pose_.theta));

    double angle_threshold = M_PI / 2; // ±90°允许前进
    double v = Kp_dist_ * dist * std::cos(alpha);

    if (std::abs(alpha) > angle_threshold) v = 0.01; // 角度大时缓慢前进

    // 最小速度保证
    double min_v = 0.02;
    if (v > 0 && v < min_v) v = min_v;

    // 限速
    double max_v = 1.0, max_w = 3.0;
    v = std::max(-max_v, std::min(v, max_v));
    double w = Kp_angle_ * alpha;
    w = std::max(-max_w, std::min(w, max_w));

    // 积分更新机器人状态
    robot_pose_.x += v * std::cos(robot_pose_.theta) * dt;
    robot_pose_.y += v * std::sin(robot_pose_.theta) * dt;
    robot_pose_.theta += w * dt;
    robot_pose_.theta = std::atan2(std::sin(robot_pose_.theta), std::cos(robot_pose_.theta));

    // 打印关键信息
    ROS_INFO_THROTTLE(0.2, "Pose: x=%.2f y=%.2f theta=%.2f | Target[%d]: x=%.2f y=%.2f dist=%.2f | v=%.2f w=%.2f",
                      robot_pose_.x, robot_pose_.y, robot_pose_.theta,
                      current_target_idx_, target.x, target.y, dist,
                      v, w);

    publishRobotMarker();
}




    void publishRobotMarker() {
        visualization_msgs::MarkerArray robot;

        // 车身
        visualization_msgs::Marker body;
        body.header.frame_id = "world";
        body.header.stamp = ros::Time::now();
        body.ns = "robot";
        body.id = 0;
        body.type = visualization_msgs::Marker::CUBE;
        body.action = visualization_msgs::Marker::ADD;
        body.pose.position.x = robot_pose_.x;
        body.pose.position.y = robot_pose_.y;
        body.pose.position.z = 0.12;
        body.pose.orientation.w = std::cos(robot_pose_.theta/2.0);
        body.pose.orientation.z = std::sin(robot_pose_.theta/2.0);
        body.scale.x = 0.4;
        body.scale.y = 0.25;
        body.scale.z = 0.12;
        body.color.r = 0.2;
        body.color.g = 0.6;
        body.color.b = 1.0;
        body.color.a = 1.0;
        robot.markers.push_back(body);

        // 四个轮子
        double wx[2] = {0.12, -0.12};
        double wy[2] = {0.15, -0.15};
        int id = 1;
        for(int i=0;i<2;i++){
            for(int j=0;j<2;j++){
                visualization_msgs::Marker wheel;
                wheel.header.frame_id = "world";
                wheel.header.stamp = ros::Time::now();
                wheel.ns = "robot";
                wheel.id = id++;
                wheel.type = visualization_msgs::Marker::CUBE;
                wheel.action = visualization_msgs::Marker::ADD;
                wheel.scale.x = 0.08;
                wheel.scale.y = 0.04;
                wheel.scale.z = 0.08;
                wheel.color.r = 0.1;
                wheel.color.g = 0.1;
                wheel.color.b = 0.1;
                wheel.color.a = 1.0;

                double rx = wx[i]*std::cos(robot_pose_.theta)-wy[j]*std::sin(robot_pose_.theta);
                double ry = wx[i]*std::sin(robot_pose_.theta)+wy[j]*std::cos(robot_pose_.theta);
                wheel.pose.position.x = robot_pose_.x + rx;
                wheel.pose.position.y = robot_pose_.y + ry;
                wheel.pose.position.z = 0.04;
                wheel.pose.orientation.w = 1.0;
                robot.markers.push_back(wheel);
            }
        }

        // 车头箭头
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "robot";
        arrow.id = 10;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        double arrow_offset = 0.2;
        arrow.pose.position.x = robot_pose_.x + arrow_offset*std::cos(robot_pose_.theta);
        arrow.pose.position.y = robot_pose_.y + arrow_offset*std::sin(robot_pose_.theta);
        arrow.pose.position.z = 0.18;
        arrow.pose.orientation.w = std::cos(robot_pose_.theta/2.0);
        arrow.pose.orientation.z = std::sin(robot_pose_.theta/2.0);
        arrow.scale.x = 0.15;
        arrow.scale.y = 0.03;
        arrow.scale.z = 0.03;
        arrow.color.r = 1.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        robot.markers.push_back(arrow);

        marker_pub.publish(robot);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller_node");
    RobotController rc;
    ros::spin();
    return 0;
}