#include "waypoint_recorder.hpp"


double last_point[7] = {0, 0, 0, 0, 0, 0, 0};
std::string waypoint_filename;
double waypoint_interval;
int idx = 0;
ros::Publisher marker_pub;

void callback(const nav_msgs::Odometry& msg) {
    double distance = sqrt(pow(last_point[0]-msg.pose.pose.position.x, 2) +
                           pow(last_point[1]-msg.pose.pose.position.y, 2) +
                           pow(last_point[2]-msg.pose.pose.position.z, 2));

    if (distance >= waypoint_interval) {
        last_point[0] = msg.pose.pose.position.x;
        last_point[1] = msg.pose.pose.position.y;
        last_point[2] = msg.pose.pose.position.z;
        last_point[3] = msg.pose.pose.orientation.x;
        last_point[4] = msg.pose.pose.orientation.y;
        last_point[5] = msg.pose.pose.orientation.z;
        last_point[6] = msg.pose.pose.orientation.w;

        // Record the waypoint
        std::ofstream record_file(waypoint_filename.c_str(), std::ios::app);
        std::stringstream wp;
        for (int i = 0; i < 7; i++) {
            wp << last_point[i];
            if (i != 6) {
                wp << ",";
            }
        }
        record_file << wp.str() << std::endl;
        record_file.close();

        // Marker generation
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = idx;
        idx++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg.pose.pose.position.x;
        marker.pose.position.y = msg.pose.pose.position.y;
        marker.pose.position.z = msg.pose.pose.position.z;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker_pub.publish(marker);
    }
}

void waypoint_recorder(std::string sub, std::string file_name, double interval) {
    last_point[0] = 0;
    last_point[1] = 0;
    last_point[2] = 0;
    last_point[3] = 0;
    last_point[4] = 0;
    last_point[5] = 0;
    last_point[6] = 0;
    waypoint_filename = file_name;
    waypoint_interval = interval;

    ros::NodeHandle nh;
    ROS_INFO("Ready to record the waypoints......");

    marker_pub = nh.advertise<visualization_msgs::Marker>("/waypoint_marker", 10);
    ros::Subscriber sub_pose = nh.subscribe(sub, 10, callback);

    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_recorder", ros::init_options::AnonymousName);
    std::string waypoint_path;
    ros::NodeHandle nh;

    if (!nh.getParam("waypoint_path", waypoint_path))
    {
        ROS_ERROR("No waypoint_path param");
        return -1;
    }

    waypoint_recorder("/odom", waypoint_path, 0.25);
    return 0;
}
