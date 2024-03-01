#ifndef WAYPOINT_RECORDER_HPP
#define WAYPOINT_RECORDER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>

void waypoint_recorder(std::string sub="/ndt_pose", std::string file_name="./waypoints.txt", double interval=0.25);

#endif // WAYPOINT_RECORDER_HPP
