#ifndef PURE_PURSUIT_PLANNER_H
#define PURE_PURSUIT_PLANNER_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/local_planner_util.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"

namespace dwa {

class DwaPlanner : public nav_core::BaseLocalPlanner {
public:
    DwaPlanner();
    
    DwaPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    ~DwaPlanner();

    ros::Subscriber laser_sub_;
    sensor_msgs::LaserScan scan_;

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

private:
    base_local_planner::LocalPlannerUtil planner_util_;
    bool is_initialized_;
    bool goal_reached_;
    bool dynamic_obstacle_;
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    double resolution;
    double origin_x;
    double origin_y;
    double inflation_radius_;
    double costmap_score;
    double distance_step_;
    double time_step_;
    double sim_time_;
    double max_velocity_;
    double min_velocity_;
    double max_angular_velocity_;
    double min_angular_velocity_;
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
    geometry_msgs::PoseStamped current_pose_;

    void laserCallback(const sensor_msgs::LaserScan& scan);
    double evaluateTrajectory(double v, double w, const geometry_msgs::PoseStamped& current_pose);   
    geometry_msgs::PoseStamped updatePlan(const geometry_msgs::PoseStamped& current_pose_, std::vector<geometry_msgs::PoseStamped>& global_plan_);
    bool isInBounds(unsigned int mx, unsigned int my);
    double getDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);
    double getYaw(const geometry_msgs::PoseStamped& pose);
};

}  // namespace dwa

#endif  // PURE_PURSUIT_PLANNER_H
