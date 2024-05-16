#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "Planner.h"

Planner *planner;
ros::Publisher path_pub;
tf::TransformListener *tf_listener = nullptr;
tf::TransformBroadcaster *tf_broadcaster = nullptr;
bool goal_provided = false;

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    planner->Set(msg->data, msg->info.width, msg->info.height, 100, msg->info.resolution);
    ROS_INFO("Received map message:");
    ROS_INFO("Map metadata:");
    ROS_INFO("Resolution: %.3f", msg->info.resolution);
    ROS_INFO("Width: %d, Height: %d", msg->info.width, msg->info.height);
}

void MoveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    planner->AddEnd(Eigen::Vector2i(msg->pose.position.x, msg->pose.position.y));
    goal_provided = true;
    ROS_INFO("Received move_base_simple_goal message:");
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    planner->AddStart(Eigen::Vector2i(msg->pose.pose.position.x, msg->pose.pose.position.y));
    ROS_INFO("Received initialpose message:");
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void PlanCallback()
{
    ROS_INFO("Planning path...");
    std::vector<Eigen::Vector2f> path = planner->PlanPath();

    // Check if path is empty
    if (path.empty())
    {
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
        ROS_INFO("X: %.2f Y: %.2f", pos.x(), pos.y()); // Correct format specifier
        pose.pose.position.z = 0;      // Z coordinate
        pose.pose.orientation.w = 1;   // Default orientation
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map"; // Adjust frame_id according to your map frame
        path_msg.poses.push_back(pose);
    }


    // Publish the path message
    path_pub.publish(path_msg);

    ROS_INFO("Published path to RViz.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    planner = new Planner();
    tf_listener = new tf::TransformListener();
    tf_broadcaster = new tf::TransformBroadcaster();

    ros::Subscriber map_sub = nh.subscribe("/map", 10, MapCallback);
    ros::Subscriber move_base_sub = nh.subscribe("/move_base_simple/goal", 10, MoveBaseGoalCallback);
    // ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 10, InitialPoseCallback);
    // ros::Subscriber plan_sub = nh.subscribe("/clicked_point", 10, PlanCallback);

    path_pub = nh.advertise<nav_msgs::Path>("/path", 10, true);

    // Parse command line arguments for transform
    double px = 0.0, py = 0.0, pz = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool transform_provided = false;

    if (argc == 8)
    {
        px = atof(argv[1]);
        py = atof(argv[2]);
        pz = atof(argv[3]);
        qx = atof(argv[4]);
        qy = atof(argv[5]);
        qz = atof(argv[6]);
        qw = atof(argv[7]);
        transform_provided = true;
    }
    else
    {
        ROS_INFO("No transform arguments provided. Waiting in loop.");
    }

    ros::Rate loop_rate(10); // Loop rate in Hz
    while (ros::ok())
    {
        ros::spinOnce();

        if (transform_provided)
        {
            // Broadcast the transform
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(px, py, pz));
            tf::Quaternion q(qx, qy, qz, qw);
            transform.setRotation(q);

            tf::StampedTransform stamped_transform(transform, ros::Time::now(), "map", "base_link");
            tf_broadcaster->sendTransform(stamped_transform);

            // Get the transform from map to base_link
            tf::StampedTransform transformStamped;
            try
            {
                tf_listener->lookupTransform("map", "base_link", ros::Time(0), transformStamped);
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            // Extract the position from the transform and set it as the initial pose
            geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
            initial_pose_msg.pose.pose.position.x = transformStamped.getOrigin().x();
            initial_pose_msg.pose.pose.position.y = transformStamped.getOrigin().y();
            initial_pose_msg.pose.pose.position.z = transformStamped.getOrigin().z();
            initial_pose_msg.pose.pose.orientation.x = transformStamped.getRotation().x();
            initial_pose_msg.pose.pose.orientation.y = transformStamped.getRotation().y();
            initial_pose_msg.pose.pose.orientation.z = transformStamped.getRotation().z();
            initial_pose_msg.pose.pose.orientation.w = transformStamped.getRotation().w();
            InitialPoseCallback(boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_msg));
            transform_provided = false; // Clear the flag after setting initial pose
        }

        if (goal_provided)
        {
            PlanCallback();  // Plan path once goal is provided
            goal_provided = false; // Clear the flag after planning
        }

        loop_rate.sleep();
    }

    delete planner;
    delete tf_listener;
    delete tf_broadcaster;

    return 0;
}
