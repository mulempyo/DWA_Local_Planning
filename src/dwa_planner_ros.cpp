// dwa_planner_ros.cpp
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "dwa_planner_ros/dwa_planner_ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_planner_ros::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_planner_ros {

DWAPlannerROS::DWAPlannerROS()
  : initialized_(false), size_x_(0), size_y_(0), goal_reached_(false), rotate(true)
{
    ros::NodeHandle nh;
    //laser_sub_ = nh.subscribe("scan", 1, &DWAPlannerROS::laserCallback, this);
    person_sub_ = nh.subscribe("person_probability", 10, &DWAPlannerROS::personDetect, this);
    amcl_sub_ = nh.subscribe("/safe", 10, &DWAPlannerROS::safeMode, this);
}

DWAPlannerROS::~DWAPlannerROS()
{
    freeMemory();
    if (planner_)
        delete planner_;
    if (costmap_model_)
        delete costmap_model_;
}

void DWAPlannerROS::freeMemory()
{
    if (charmap_)
    {
        for (int i = 0; i < size_x_; i++)
        {
            delete[] charmap_[i];
            charmap_[i] = nullptr;
        }
        delete[] charmap_;
        charmap_ = nullptr;
    }
}

void DWAPlannerROS::allocateMemory()
{
    if(charmap_ != nullptr){
        freeMemory();
    }

    charmap_ = new unsigned char*[size_x_];
    for (int i = 0; i < size_x_; i++)
        charmap_[i] = new unsigned char[size_y_];
}

void DWAPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    { 
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("odom_topic", odom_topic_, std::string("/odometry/filtered"));
        private_nh.param("map_frame", map_frame_, std::string("map"));
        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
        private_nh.param("transform_tolerance", transform_tolerance_, 0.5);
        private_nh.param("max_vel_x", max_vel_x_, 0.55);
        private_nh.param("min_vel_x", min_vel_x_, 0.0);
        private_nh.param("max_vel_theta", max_vel_theta_, 2.5);
        private_nh.param("min_vel_theta", min_vel_theta_, -2.5);
        private_nh.param("acc_lim_x", acc_lim_x_, 0.25);
        private_nh.param("acc_lim_theta", acc_lim_theta_, 1.2);
        private_nh.param("control_period", control_period_, 0.2);

        // 초기화 변수 설정
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();  
        size_x_ = costmap_->getSizeInCellsX();
        size_y_ = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();
        origin_x = costmap_->getOriginX();
        origin_y = costmap_->getOriginY();

        costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        // 로봇의 footprint 설정
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

        // odom helper 초기화
        odom_helper_.setOdomTopic(odom_topic_);

        planner_ = new DWAPlanner(costmap_model_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_,
                                  private_nh);

        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("dwa_global_plan", 1);
        safe_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("/move_base/DWAPlannerROS/safe_mode", 1);
        planner_util_.initialize(tf_, costmap_, global_frame_);

        // 메모리 할당
        allocateMemory();
        ac = std::make_shared<MoveBaseClient>("move_base", true);

        safe1 = {-0.063431f, -0.031137f, 0.0f, 0.0f, 0.0f, 0.19328f, 0.999903f};
        safe2 = {4.273204f, 0.379562f, 0.0f, 0.0f, 0.0f, -0.998399f, 0.056565f};

        // 초기화 플래그 설정
        initialized_ = true;

        ROS_DEBUG("dwa_local_planner plugin initialized.");
    }
    else
    {
        ROS_WARN("dwa_local_planner has already been initialized, doing nothing.");
    }
}

void DWAPlannerROS::safeMode(std_msgs::Float64 safe){

  ROS_WARN("safeMode function in dwa");
  if(safe.data == -1){
    safe_mode = false;
  }

}

void DWAPlannerROS::personDetect(const std_msgs::Float64::ConstPtr& person){
  if(person->data == 1.0){
    person_detect = true;
  }else{
    person_detect = false;
  }
}

void DWAPlannerROS::laserCallback(const sensor_msgs::LaserScan& scan)
{
    if(initialized_){
    dynamic_obstacle_detected = false;
    bool front_obstacle_detected = false;
    bool back_obstacle_detected = false;

    int front_indices[] = {60, 90, 120, 150, 180, 210, 240, 270, 300};
    int back_indices[] = {0, 15, 30, 45, 315, 330, 345};

    costmap_ros_->getRobotPose(current_pose_);
    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;
    double robot_theta = tf2::getYaw(current_pose_.pose.orientation);

    for (int i = 0; i < sizeof(front_indices)/sizeof(front_indices[0]); i++) {
        int index = front_indices[i];
        if (scan.ranges[index] >= scan.range_min && scan.ranges[index] <= (scan.range_min + 0.05)) {
            float scan_angle = scan.angle_min + index * scan.angle_increment;
            front_obstacle_detected = checkObstacle(scan.ranges[index], robot_x, robot_y, robot_theta, scan_angle);
        }
    }

    for (int j = 0; j < sizeof(back_indices)/sizeof(back_indices[0]); j++) {
        int index = back_indices[j];
        if (scan.ranges[index] >= (scan.range_min + 0.1) && scan.ranges[index] <= (scan.range_min + 0.4)) {
            float scan_angle = scan.angle_min + index * scan.angle_increment;
            back_obstacle_detected = checkObstacle(scan.ranges[index], robot_x, robot_y, robot_theta, scan_angle);
        }
    }

    if(front_obstacle_detected || back_obstacle_detected){
        dynamic_obstacle_detected = true;
    }

    std::vector<geometry_msgs::Point> obstacles;
    double angle = scan.angle_min;

    for (const auto& range : scan.ranges) {
        if (range >= scan.range_min && range <= scan.range_max) {
            geometry_msgs::Point obstacle;
            obstacle.x = range * std::cos(angle);
            obstacle.y = range * std::sin(angle);
            obstacle.z = 0.0;
            obstacles.push_back(obstacle);
        }
        angle += scan.angle_increment;
    }

    unsigned int new_size_x = costmap_->getSizeInCellsX();
    unsigned int new_size_y = costmap_->getSizeInCellsY();

    if (charmap_ == NULL || size_x_ != new_size_x || size_y_ != new_size_y)
    {
        freeMemory();
        size_x_ = new_size_x;
        size_y_ = new_size_y;
        allocateMemory();
    }

    for (const auto& obs : obstacles) {
        unsigned int mx, my;
        if (costmap_->worldToMap(obs.x, obs.y, mx, my)) {
            costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }

    for (unsigned int i = 0; i < size_x; ++i) {
        for (unsigned int j = 0; j < size_y; ++j) {
            if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
                bool still_obstacle = false;
                for (const auto& obs : obstacles) {
                    unsigned int mx, my;
                    if (costmap_->worldToMap(obs.x, obs.y, mx, my) && mx == i && my == j) {
                        still_obstacle = true;
                        break;
                    }
                }
                if (!still_obstacle) {
                    costmap_->setCost(i, j, costmap_2d::FREE_SPACE);
                }
            }
        }
    }

    
 }
}

bool DWAPlannerROS::checkObstacle(const double range, const double robot_x, const double robot_y, const double robot_theta, const double scan_angle)
{
    double obstacle_x = robot_x + range * cos(robot_theta + scan_angle);
    double obstacle_y = robot_y + range * sin(robot_theta + scan_angle);
    unsigned int obstacle_mx, obstacle_my;

    if (!costmap_->worldToMap(obstacle_x, obstacle_y, obstacle_mx, obstacle_my)) {
        ROS_WARN("invalid map frame");
        return false;
    }

    unsigned char cost = costmap_->getCost(obstacle_mx, obstacle_my);
    if(cost == costmap_2d::LETHAL_OBSTACLE){
        return false;
    }
    else{
        return true;
    }
}

bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("DWAPlannerROS has not been initialized.");
        return false;
    }

    goal_reached_ = false;
    rotate = true;
    safe_mode = true;

    ROS_WARN("Start planning.");
    return planner_util_.setPlan(plan);
}

bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    // Check if plugin is initialized
    if (!initialized_)
    {
        ROS_ERROR("DWAPlannerROS has not been initialized, please call initialize() before using this planner");
        return false;
    }
    
    // Get the current robot pose
    costmap_ros_->getRobotPose(current_pose_);
    double robot_pose_x = current_pose_.pose.position.x;
    double robot_pose_y = current_pose_.pose.position.y; 
    double robot_pose_theta = tf2::getYaw(current_pose_.pose.orientation);

    // Get the current robot velocity
    geometry_msgs::PoseStamped robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    double robot_vel_x = robot_vel_tf.pose.position.x;
    double robot_vel_theta = tf2::getYaw(robot_vel_tf.pose.orientation);

    // Get the transformed global plan
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    transformed_plan.clear();
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
        ROS_ERROR("Could not get local plan");
        return false;
    }

    // If the plan is empty, return false
    if (transformed_plan.empty()) {
        ROS_WARN("Transformed plan is empty");
        return false;
    }

    global_plan_.resize(transformed_plan.size());
    for(unsigned int i = 0; i < transformed_plan.size(); ++i){
        global_plan_[i] = transformed_plan[i];
    }

    geometry_msgs::PoseStamped lookahead_pose = global_plan_.back(); 
    for (const auto& pose : global_plan_) {
    double dx = pose.pose.position.x - robot_pose_x;
    double dy = pose.pose.position.y - robot_pose_y;
    double distance = hypot(dx, dy);
    if (distance == 0.1) { 
      lookahead_pose = pose;
      break;
     }
    }

    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    double robot_yaw = tf2::getYaw(current_pose_.pose.orientation);

  // Calculate the goal direction from the robot's current pose to the goal pose
    double target_yaw = atan2(lookahead_pose.pose.position.y - robot_pose_y, lookahead_pose.pose.position.x - robot_pose_x); 
    double yaw_error = angles::shortest_angular_distance(robot_yaw,target_yaw);

    // If the robot needs to rotate, handle it
    if (rotate) {
        // Check if the yaw error is within the acceptable threshold to stop rotating
        if (fabs(yaw_error) > 0.2) {
            // Continue rotating
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.3;  // Rotate proportionally to the yaw error
            ROS_WARN("Rotating to correct yaw, yaw_error: %f", fabs(yaw_error));
            return true; 
        } else {
            ROS_WARN("Yaw aligned, proceeding to move.");
            rotate = false;  
        }
    }

     geometry_msgs::PoseStamped safe_1,safe_2;
     safe_1.header.frame_id = "map";
     safe_1.header.stamp = ros::Time::now();
     safe_1.pose.position.x = safe1[0];
     safe_1.pose.position.y = safe1[1];

     safe_2.header.frame_id = "map";
     safe_2.header.stamp = ros::Time::now();
     safe_2.pose.position.x = safe2[0];
     safe_2.pose.position.y = safe2[1];

     geometry_msgs::TransformStamped transformStamped1 = tf_->lookupTransform("base_footprint", "map", ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(safe_1, robot_safe1, transformStamped1);

     geometry_msgs::TransformStamped transformStamped2 = tf_->lookupTransform("base_footprint", "map", ros::Time(0), ros::Duration(1.0));
     tf2::doTransform(safe_2, robot_safe2, transformStamped2);

     double dx1 = robot_pose_x - robot_safe1.pose.position.x;
     double dy1 = robot_pose_y - robot_safe1.pose.position.y;
     double dx2 = robot_pose_x - robot_safe2.pose.position.x;
     double dy2 = robot_pose_y - robot_safe2.pose.position.y;

     if(safe_mode){
        geometry_msgs::PoseStamped safe_pub;
      if(hypot(dx1,dy1) <= (xy_goal_tolerance_)){
         safe_pub = safe_1; 
      }
    
      if(hypot(dx2,dy2) <= (xy_goal_tolerance_)){
         safe_pub = safe_2; 
      }
      safe_pub_.publish(safe_pub);
     }

    // Now proceed with normal DWA planning
    unsigned int start_mx, start_my, goal_mx, goal_my;
    geometry_msgs::PoseStamped goal = global_plan_.back();

    geometry_msgs::PoseStamped current_robot_pose;
    costmap_ros_->getRobotPose(current_robot_pose);   
    double start_wx = current_robot_pose.pose.position.x;
    double start_wy = current_robot_pose.pose.position.y;
    double goal_wx = goal.pose.position.x;
    double goal_wy = goal.pose.position.y;

    if (!costmap_->worldToMap(start_wx, start_wy, start_mx, start_my)){
        ROS_WARN("Cannot convert world current coordinates to map coordinates");
    }

    if(!costmap_->worldToMap(goal_wx, goal_wy, goal_mx, goal_my)) {
        ROS_WARN("Cannot convert world goal coordinates to map coordinates");
    }
    
    // Perform Bresenham's line algorithm to check for obstacles along the straight path
    std::vector<std::pair<int, int>> line_points = planner_->bresenhamLine(start_mx, start_my, goal_mx, goal_my);

    for (const auto& point : line_points) {
        unsigned int mx = point.first;
        unsigned int my = point.second;

        // Get cost from the costmap at each point
        unsigned char cost = costmap_->getCost(mx, my);

        // Check if there's a lethal obstacle
        if (cost == costmap_2d::LETHAL_OBSTACLE) {
            planner_->obstacleFound(true);
            break;
        }
        if(cost == costmap_2d::FREE_SPACE){
            planner_->obstacleFound(false);
            break;
        }
    }

    // Proceed with normal DWA planning
    const unsigned char* charmap = costmap_->getCharMap();

    unsigned int new_size_x = costmap_->getSizeInCellsX();
    unsigned int new_size_y = costmap_->getSizeInCellsY();

    if (charmap_ == nullptr || size_x_ != new_size_x || size_y_ != new_size_y)
    {
        freeMemory();
        size_x_ = new_size_x;
        size_y_ = new_size_y;
        allocateMemory();
    }

    for (unsigned int j = 0; j < size_y_; j++)
    {
        for (unsigned int i = 0; i < size_x_; i++)
            charmap_[i][j] = charmap[i + j * size_x_];
    }

    std::vector<std::vector<double>> reference_path;
    for (const auto& pose : global_plan_)
    {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double theta = tf2::getYaw(pose.pose.orientation);
        reference_path.emplace_back(std::vector<double>{x, y, theta});
    }
    publishGlobalPlan(global_plan_);

    double dwa_cmd_vel_x, dwa_cmd_vel_theta;
    bool success = planner_->computeVelocityCommands(robot_vel_x, robot_vel_theta, robot_pose_x, robot_pose_y, robot_pose_theta,
                                                     reference_path, charmap_, size_x_, size_y_,
                                                     resolution, origin_x, origin_y, dwa_cmd_vel_x, dwa_cmd_vel_theta);   


    if (!success)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }
    else
    {
        if(dynamic_obstacle_detected || person_detect){
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        } else {
            cmd_vel.linear.x = dwa_cmd_vel_x;
            cmd_vel.angular.z = dwa_cmd_vel_theta;

            // Check if goal is reached
            geometry_msgs::PoseStamped robot_pose;
            costmap_ros_->getRobotPose(robot_pose);
            geometry_msgs::PoseStamped global_ = global_plan_.back();

            double dx = robot_pose.pose.position.x - global_.pose.position.x;
            double dy = robot_pose.pose.position.y - global_.pose.position.y;

            if (hypot(dx, dy) <= xy_goal_tolerance_) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                goal_reached_ = true;  
                rotate = true;
                ROS_INFO("Goal reached.");
            }
            return true;
        }
    }
}

bool DWAPlannerROS::isGoalReached()
{
    if (!initialized_)
    {
        ROS_ERROR("DWAPlannerROS has not been initialized, please call initialize() before using this planner");
        return false;
    }

    return goal_reached_;
}


double DWAPlannerROS::getYaw(const geometry_msgs::PoseStamped& pose) {
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

void DWAPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = map_frame_;
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = global_plan;
    global_plan_pub_.publish(gui_path);
}

} // namespace dwa_planner_ros
