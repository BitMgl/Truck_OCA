#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>

#include "em_planner_ros/PlanningResult.h"
#include "em_planner_ros/TrajectoryPoint.h"
#include "em_planner_ros/Localization.h"
#include "em_planner_ros/ObstacleList.h"
#include "em_planner_ros/Obstacle.h"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// 创建 tf 广播器
// tf::TransformBroadcaster broadcaster;

class PathVisualizer
{
public:
    PathVisualizer()
    {
        ros::NodeHandle nh;

        // 订阅qp路径规划话题
        qp_path_sub_ = nh.subscribe("/qp_planning_result", 1, &PathVisualizer::qp_pathCallback, this);
        // 订阅dp路径规划话题（之后补充）
        dp_path_sub_ = nh.subscribe("/dp_planning_result", 1, &PathVisualizer::dp_pathCallback, this);
        // 订阅参考参考线话题
        refline_sub_ = nh.subscribe("/reference_line", 1, &PathVisualizer::reflineCallback, this);
        // 订阅障碍物话题
        obs_sub_ = nh.subscribe("/obstacle_list", 1, &PathVisualizer::obsCallback, this);
        // 订阅自车话题
        ego_sub_ = nh.subscribe("/localization", 1, &PathVisualizer::egoCallback, this);

        // 发布 Marker 以便 RViz 显示路径
        marker_qp_path_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_qp_path", 10);
        marker_dp_path_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_dp_path", 10);
        marker_refline_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_refline", 10);
        marker_obs_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_obs", 10);
        marker_ego_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_ego", 10);

        // 创建一个定时器来定期发布地图坐标变换
        timer_ = nh.createTimer(ros::Duration(0.1), &PathVisualizer::publishMapTransform, this); // 每0.1秒发布一次
    }

    // 发布变换
    void publishMapTransform(const ros::TimerEvent&)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));  // map 原点位置
        tf::Quaternion q;
        q.setRPY(0, 0, 0);  // 假设没有旋转
        transform.setRotation(q);

        // 调试输出
        ROS_INFO("Publishing map -> base_link transform");

        // 发布 map 到 base_link 变换
        broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

    void rotate(std::vector<double>& ego_x, std::vector<double>& ego_y, double x,
                double y, double x_center, double y_center, double theta)
    {
        double rad = theta;  // 弧度
        double d = 2.236;    // 计算点之间的距离
        double alpha = std::atan2(y - y_center, x - x_center);  // 计算点 a 相对于点 b 的方位角
        alpha += rad;        // 增加角度
        x = x_center + d * std::cos(alpha);  // 更新点 a 的 x 坐标
        y = y_center + d * std::sin(alpha);  // 更新点 a 的 y 坐标
        ego_x.push_back(x);
        ego_y.push_back(y);
    }

    void qp_pathCallback(const em_planner_ros::PlanningResult::ConstPtr& msg)
    {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.ns = "qp_path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.1; // 线宽
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.lifetime = ros::Duration(0); // 永久显示

        for (const auto& point : msg->trajectory)
        {
            geometry_msgs::Point p;
            p.x = point.xg;
            p.y = point.yg;
            p.z = 0.0;
            line_strip.points.push_back(p);
        }

        marker_qp_path_pub_.publish(line_strip);
    }

    void dp_pathCallback(const em_planner_ros::PlanningResult::ConstPtr& msg)
    {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.ns = "qp_path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.1; // 线宽
        line_strip.color.r = 0.5;
        line_strip.color.g = 0.5;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.lifetime = ros::Duration(0); // 永久显示

        for (const auto& point : msg->trajectory)
        {
            geometry_msgs::Point p;
            p.x = point.xg;
            p.y = point.yg;
            p.z = 0.0;
            line_strip.points.push_back(p);
        }

        marker_dp_path_pub_.publish(line_strip);
    }

    void reflineCallback(const em_planner_ros::PlanningResult::ConstPtr& msg)
    {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.ns = "ref_line";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.1; // 线宽
        line_strip.color.r = 0.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.lifetime = ros::Duration(0); // 永久显示

        for (const auto& point : msg->trajectory)
        {
            geometry_msgs::Point p;
            p.x = point.xg;
            p.y = point.yg;
            p.z = 0.0;
            line_strip.points.push_back(p);
        }

        marker_refline_pub_.publish(line_strip);
    }

    void obsCallback(const em_planner_ros::ObstacleList::ConstPtr& msg){
    for (const auto& obs : msg->obstacle_list)
    {
        // 获取障碍物位置和尺寸
        double x_center = obs.traj_p.xg;
        double y_center = obs.traj_p.yg;
        double theta = obs.traj_p.global_angle;
        double width = obs.width;
        double length = obs.length;
        double height = obs.height;

        // 创建 Marker 消息
        visualization_msgs::Marker cube_marker;
        cube_marker.header.frame_id = "map";  // 使用 RViz 中的坐标系
        cube_marker.header.stamp = ros::Time::now();
        cube_marker.ns = "obstacle";
        cube_marker.id = obs.id;  // 每个障碍物用其自身的 ID
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;

        // 设置障碍物的位置
        cube_marker.pose.position.x = x_center;
        cube_marker.pose.position.y = y_center;
        cube_marker.pose.position.z = height / 2;  // 设置在 z 轴上稍微抬高

        // 设置障碍物方向（绕 z 轴旋转）
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);  // 仅绕 Z 轴旋转
        cube_marker.pose.orientation.x = q.x();
        cube_marker.pose.orientation.y = q.y();
        cube_marker.pose.orientation.z = q.z();
        cube_marker.pose.orientation.w = q.w();

        // 设置障碍物的尺寸
        cube_marker.scale.x = length;
        cube_marker.scale.y = width;
        cube_marker.scale.z = height;

        // 设置颜色和透明度
        cube_marker.color.r = 0.0f;
        cube_marker.color.g = 1.0f;  // 绿色
        cube_marker.color.b = 0.0f;
        cube_marker.color.a = 0.7;

        cube_marker.lifetime = ros::Duration(0);  // 永久显示

        // 发布 Marker
        marker_obs_pub_.publish(cube_marker);
    }
    }

    void egoCallback(const em_planner_ros::Localization::ConstPtr& msg){
        // 获取自车位置和尺寸
        double x_center = msg->xg;
        double y_center = msg->yg;
        double theta = msg->yaw;
        double width = 2;
        double length = 4;
        double height = 1.5;

        // 创建 Marker 消息
        visualization_msgs::Marker cube_marker;
        cube_marker.header.frame_id = "map";  // 使用 RViz 中的坐标系
        cube_marker.header.stamp = ros::Time::now();
        cube_marker.ns = "ego_car";
        cube_marker.id = 0;  
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;

        // 设置自车的位置
        cube_marker.pose.position.x = x_center;
        cube_marker.pose.position.y = y_center;
        cube_marker.pose.position.z = height / 2;  // 设置在 z 轴上稍微抬高

        // 设置自车方向（绕 z 轴旋转）
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);  // 仅绕 Z 轴旋转
        cube_marker.pose.orientation.x = q.x();
        cube_marker.pose.orientation.y = q.y();
        cube_marker.pose.orientation.z = q.z();
        cube_marker.pose.orientation.w = q.w();

        // 设置自车的尺寸
        cube_marker.scale.x = length;
        cube_marker.scale.y = width;
        cube_marker.scale.z = height;

        // 设置颜色和透明度
        cube_marker.color.r = 1.0f;
        cube_marker.color.g = 0.5f;  
        cube_marker.color.b = 0.0f;
        cube_marker.color.a = 0.7;

        cube_marker.lifetime = ros::Duration(0);  // 永久显示

        // 发布 Marker
        marker_ego_pub_.publish(cube_marker);
    }

private:
    ros::Subscriber qp_path_sub_;
    ros::Subscriber dp_path_sub_;
    ros::Subscriber refline_sub_;
    ros::Subscriber obs_sub_;
    ros::Subscriber ego_sub_;

    ros::Publisher marker_dp_path_pub_;
    ros::Publisher marker_qp_path_pub_;
    ros::Publisher marker_refline_pub_;
    ros::Publisher marker_obs_pub_;
    ros::Publisher marker_ego_pub_;

    // 定时器对象，用于定期发布变换
    ros::Timer timer_;
    tf::TransformBroadcaster broadcaster_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualizer");
    PathVisualizer pv;
    ros::spin();
    return 0;
}
