#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "Planner.h"

Planner * planner;
ros::Publisher path_pub;

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    planner->Set(msg->data,msg->info.width, msg->info.height,3,msg->info.resolution);
    ROS_INFO("Received map message:");
    ROS_INFO("Map metadata:");
    ROS_INFO("Resolution: %.2f", msg->info.resolution);
    ROS_INFO("Width: %d, Height: %d", msg->info.width, msg->info.height);
    int r = 0;
    int c = 0;
}

void MoveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    planner->AddStart(Eigen::Vector2i(msg->pose.position.x, msg->pose.position.y));
    ROS_INFO("Received move_base_simple_goal message:");
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    planner->AddEnd(Eigen::Vector2i(msg->pose.pose.position.x, msg->pose.pose.position.y));
    ROS_INFO("Received initialpose message:");
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void PlanCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ROS_INFO("Planning path...");
    std::vector<Eigen::Vector2f> path = planner->PlanPath();
    
    // Check if path is empty
    if (path.empty()) {
        ROS_WARN("No path found.");
        return;
    }

    // Create a nav_msgs::Path message
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // Adjust frame_id according to your map frame
    
    // Iterate over each position in the path and add it to the path message
    for (const auto& pos : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pos.x(); // X coordinate
        pose.pose.position.y = pos.y(); // Y coordinate
        ROS_INFO("X: %d Y:%d", pos.x(),pos.y());
        pose.pose.position.z = 0; // Z coordinate
        pose.pose.orientation.w = 1; // Default orientation
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map"; // Adjust frame_id according to your map frame
        path_msg.poses.push_back(pose);
    }

    // Publish the path message
    path_pub.publish(path_msg);
    
    ROS_INFO("Published path to RViz.");
}


int main(int argc, char **argv) {

    planner = new Planner();

    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 10, MapCallback);
    ros::Subscriber move_base_sub = nh.subscribe("/move_base_simple/goal", 10, MoveBaseGoalCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 10, InitialPoseCallback);
    ros::Subscriber plan_sub = nh.subscribe("/clicked_point", 10, PlanCallback);

    //Position localizer

    path_pub = nh.advertise<nav_msgs::Path>("/path", 10, true);

    ros::Rate loop_rate(10); // Loop rate in Hz
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 