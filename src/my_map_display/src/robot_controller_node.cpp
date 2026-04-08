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

        // ==================== 控制器参数（虚拟控制点法） ====================
        nh_.param("Kx", Kx_, 2.8);
        nh_.param("Ky", Ky_, 2.8);
        nh_.param("virtual_l", l_, 0.09);           // 虚拟点前视距离（最关键参数）
        nh_.param("arrival_tolerance", tol_, 0.05);
        nh_.param("control_freq", control_freq_, 20.0);
        nh_.param("path_hysteresis", path_hysteresis_, 0.5);  // 路径更新防抖（秒）

        // 初始机器人状态
        nh_.param("init_x", robot_pose_.x, 0.0);
        nh_.param("init_y", robot_pose_.y, 0.0);
        nh_.param("init_theta", robot_pose_.theta, 0.0);

        // 定时器
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_freq_),
                                 &RobotController::controlLoop, this);

        last_time_ = ros::Time(0);
        last_path_time_ = ros::Time(0);
        current_target_idx_ = 0;

        ROS_INFO("Robot controller started (Virtual Point Control)");
        ROS_INFO("Kx=%.2f, Ky=%.2f, virtual_l=%.3f, tol=%.3f, freq=%.1f, hysteresis=%.2f",
                 Kx_, Ky_, l_, tol_, control_freq_, path_hysteresis_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub;
    ros::Subscriber path_sub;
    ros::Timer timer_;

    Pose2D robot_pose_;
    std::vector<Pose2D> path_points_;
    int current_target_idx_;

    double Kx_, Ky_, l_, tol_, control_freq_, path_hysteresis_;
    ros::Time last_time_;
    ros::Time last_path_time_;

    // ====================== 改进的路径回调（防止反复重置） ======================
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            path_points_.clear();
            current_target_idx_ = 0;
            ROS_WARN("Received empty path!");
            return;
        }

        // 防抖：太频繁的路径更新直接忽略
        double dt_path = (ros::Time::now() - last_path_time_).toSec();
        if (dt_path < path_hysteresis_ && !path_points_.empty()) {
            ROS_WARN_THROTTLE(1.0, "Path update too frequent (%.2fs), ignoring", dt_path);
            return;
        }
        last_path_time_ = ros::Time::now();

        // 构建新路径
        std::vector<Pose2D> new_path;
        new_path.reserve(msg->poses.size());
        for (const auto &pose_stamped : msg->poses) {
            Pose2D p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            p.theta = 0.0;
            new_path.push_back(p);
        }

        // 找到新路径中离机器人最近的点，作为新的起点（避免回头）
        int best_idx = 0;
        double min_dist = 1e10;
        for (int i = 0; i < static_cast<int>(new_path.size()); ++i) {
            double d = std::hypot(new_path[i].x - robot_pose_.x,
                                  new_path[i].y - robot_pose_.y);
            if (d < min_dist) {
                min_dist = d;
                best_idx = i;
            }
        }

        // 只保留最近点及之后的路径
        path_points_.clear();
        for (int i = best_idx; i < static_cast<int>(new_path.size()); ++i) {
            path_points_.push_back(new_path[i]);
        }

        current_target_idx_ = 0;

        ROS_INFO("Received new path with %zu points, started from original index %d (closest dist=%.3f)",
                 path_points_.size(), best_idx, min_dist);
    }

    // ====================== 虚拟控制点法（Feedback Linearization） ======================
    void controlLoop(const ros::TimerEvent&) {
        ros::Time now = ros::Time::now();
        double dt = (last_time_.isZero()) ? (1.0 / control_freq_) : (now - last_time_).toSec();
        last_time_ = now;

        if (path_points_.empty() || current_target_idx_ >= static_cast<int>(path_points_.size())) {
            publishRobotMarker();
            return;
        }

        const Pose2D &target = path_points_[current_target_idx_];
        double dx = target.x - robot_pose_.x;
        double dy = target.y - robot_pose_.y;
        double dist = std::hypot(dx, dy);

        // 到达目标点
        if (dist < tol_) {
            robot_pose_.x = target.x;
            robot_pose_.y = target.y;

            // 到达后缓慢对准（可改为0或路径终点方向）
            double desired_theta = 0.0;
            double alpha = std::atan2(std::sin(desired_theta - robot_pose_.theta),
                                      std::cos(desired_theta - robot_pose_.theta));
            robot_pose_.theta += 1.5 * alpha * dt;   // 缓慢收敛
            robot_pose_.theta = std::atan2(std::sin(robot_pose_.theta), std::cos(robot_pose_.theta));

            ROS_INFO_THROTTLE(1.0, "Reached target[%d]  dist=%.3f", current_target_idx_, dist);
            current_target_idx_++;
            publishRobotMarker();
            return;
        }

        // =============== 虚拟控制点控制器 ===============
        double x = robot_pose_.x;
        double y = robot_pose_.y;
        double theta = robot_pose_.theta;

        double x_d = target.x;
        double y_d = target.y;

        double ux = Kx_ * (x_d - x);     // v_x_d = 0
        double uy = Ky_ * (y_d - y);     // v_y_d = 0

        double v =  std::cos(theta) * ux + std::sin(theta) * uy;
        double w = (-std::sin(theta) * ux + std::cos(theta) * uy) / l_;

        // 限幅 + 最小速度
        const double max_v = 0.8;
        const double max_w = 2.5;
        v = std::max(-max_v, std::min(v, max_v));
        w = std::max(-max_w, std::min(w, max_w));

        if (dist > tol_ && std::abs(v) < 0.02) {
            v = 0.02 * (v > 0 ? 1.0 : -1.0);
        }

        // 积分更新机器人状态（Unicycle模型）
        robot_pose_.x += v * std::cos(theta) * dt;
        robot_pose_.y += v * std::sin(theta) * dt;
        robot_pose_.theta += w * dt;
        robot_pose_.theta = std::atan2(std::sin(robot_pose_.theta), std::cos(robot_pose_.theta));

        // 日志
        ROS_INFO_THROTTLE(0.2, "Pose: x=%.2f y=%.2f theta=%.2f | Target[%d]: x=%.2f y=%.2f dist=%.3f | v=%.3f w=%.3f",
                          robot_pose_.x, robot_pose_.y, robot_pose_.theta,
                          current_target_idx_, target.x, target.y, dist, v, w);

        publishRobotMarker();
    }

    void publishRobotMarker() {
        // ================== 原有的 Marker 发布代码（完全不变） ==================
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
        body.pose.orientation.w = std::cos(robot_pose_.theta / 2.0);
        body.pose.orientation.z = std::sin(robot_pose_.theta / 2.0);
        body.scale.x = 0.4;
        body.scale.y = 0.25;
        body.scale.z = 0.12;
        body.color.r = 0.2; body.color.g = 0.6; body.color.b = 1.0; body.color.a = 1.0;
        robot.markers.push_back(body);

        // 四个轮子
        double wx[2] = {0.12, -0.12};
        double wy[2] = {0.15, -0.15};
        int id = 1;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                visualization_msgs::Marker wheel;
                wheel.header.frame_id = "world";
                wheel.header.stamp = ros::Time::now();
                wheel.ns = "robot";
                wheel.id = id++;
                wheel.type = visualization_msgs::Marker::CUBE;
                wheel.action = visualization_msgs::Marker::ADD;
                wheel.scale.x = 0.08; wheel.scale.y = 0.04; wheel.scale.z = 0.08;
                wheel.color.r = 0.1; wheel.color.g = 0.1; wheel.color.b = 0.1; wheel.color.a = 1.0;

                double rx = wx[i] * std::cos(robot_pose_.theta) - wy[j] * std::sin(robot_pose_.theta);
                double ry = wx[i] * std::sin(robot_pose_.theta) + wy[j] * std::cos(robot_pose_.theta);
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
        arrow.pose.position.x = robot_pose_.x + arrow_offset * std::cos(robot_pose_.theta);
        arrow.pose.position.y = robot_pose_.y + arrow_offset * std::sin(robot_pose_.theta);
        arrow.pose.position.z = 0.18;
        arrow.pose.orientation.w = std::cos(robot_pose_.theta / 2.0);
        arrow.pose.orientation.z = std::sin(robot_pose_.theta / 2.0);
        arrow.scale.x = 0.15; arrow.scale.y = 0.03; arrow.scale.z = 0.03;
        arrow.color.r = 1.0; arrow.color.g = 1.0; arrow.color.b = 0.0; arrow.color.a = 1.0;
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