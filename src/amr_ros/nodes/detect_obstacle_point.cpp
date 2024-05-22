#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/distances.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

#include <cmath>

double OBST_HIGHT_MIN_Z = -0.1; //default: -0.1
double OBST_HIGHT_MAX_Z = 1.0;  //default: 1.0

ros::Publisher pub_close_points;
//ros::Publisher pub_slice_points;
ros::Publisher pub_cmd_vel;

double linear_velocity = 0;
double turning_radius = 0;
double goal_distance = 0;
double offset_dist = 0.1;
bool is_near_goal = false;
int32_t obstacle_points_num = 0;
int32_t obstacle_lim = 0;

//sonar
double sonar_dist_left = 4.0;
double sonar_dist_center = 4.0;
double sonar_dist_right = 4.0;
#define SONAR_MIN_DIST 0.30 //30cm

pcl::PointCloud<pcl::PointXYZ>::Ptr unslice(pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(points->size());
    for (int i = 0; i < points->size(); i++)
    {
        cloud->at(i).getVector3fMap().head<2>() = points->at(i).getVector3fMap().head<2>();
        cloud->at(i).z = 0.0f;
    }

    return cloud;
}

double calculateSpeedReduction()
{
    double speed_rate = 0;

    if ((obstacle_points_num > obstacle_lim) || (sonar_dist_center < SONAR_MIN_DIST))
    {
        speed_rate = 0;
    }
    else
    {
        speed_rate = 1.0;
    }

    return speed_rate;
}

void sonarLeftCallback(const sensor_msgs::RangeConstPtr& msg)
{
    sonar_dist_left = msg->range;
}

void sonarCenterCallback(const sensor_msgs::RangeConstPtr& msg)
{
    sonar_dist_center = msg->range;
    ROS_INFO("detect_obst: sonar_center:%.3f",sonar_dist_center);
}

void sonarRightCallback(const sensor_msgs::RangeConstPtr& msg)
{
    sonar_dist_right = msg->range;
}


void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs::PointCloud2 data to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input,*cloud_input);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_input);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_input);

    // Create the PassThrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    // Set the axis to filter along
    pass.setFilterFieldName("z");
    // Set the range of acceptable values
    pass.setFilterLimits(OBST_HIGHT_MIN_Z, OBST_HIGHT_MAX_Z);
    // Set the input cloud
    pass.setInputCloud(cloud_input);
    pass.filter(*cloud_input);

    const double width = 0.52;
    const double ratio = 2.0; //3.0

    double max_filter_dist = 0;

    // Create a new point cloud for the filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //If robot turn, consider to ring area
    if (turning_radius != 0)
    {
        double center_angle;
        center_angle = (linear_velocity * ratio) / turning_radius;

        pcl::CropBox<pcl::PointXYZ> boxFilter;

        //TODO: min is too small z,floor is not removed
        if (turning_radius > 0) {
            // Set min and max for positive turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, -width/2, OBST_HIGHT_MIN_Z, 1.0));
            max_filter_dist = is_near_goal ? goal_distance : turning_radius * std::sin(center_angle);
            boxFilter.setMax(Eigen::Vector4f(max_filter_dist, turning_radius, OBST_HIGHT_MAX_Z, 1.0));

        }
        else if (turning_radius < 0){
            // Set min and max for negative turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, turning_radius, OBST_HIGHT_MIN_Z, 1.0));
            max_filter_dist = is_near_goal ? goal_distance : turning_radius * std::sin(center_angle);
            boxFilter.setMax(Eigen::Vector4f(max_filter_dist, width/2, OBST_HIGHT_MAX_Z, 1.0));

        }
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*cloud_input);

        double inner_radius = abs(turning_radius) - width/2; // Set your desired distance
        double outer_radius = abs(turning_radius) + width/2; // Set your desired distance


        // Iterate over the points in the point cloud
        for (const auto& point : cloud_input->points)
        {
            // Compute the Euclidean distance from the point to the turning radius center
            double point_distance = std::sqrt(point.x * point.x + (point.y - turning_radius) * (point.y - turning_radius));

            // If the distance is less than or equal to the desired distance, add the point to the filtered cloud
            if (point_distance <= outer_radius && point_distance >= inner_radius)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }
    //If robot go straight, consider to box area
    else
    {
        // Create the filtering object
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(0.0, -width/2, OBST_HIGHT_MIN_Z, 1.0));
        max_filter_dist = is_near_goal ? goal_distance : linear_velocity*ratio;
        boxFilter.setMax(Eigen::Vector4f(max_filter_dist, width/2, OBST_HIGHT_MAX_Z, 1.0));
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*filtered_cloud);

    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    obstacle_points_num = filtered_cloud->points.size();

    //Publish point cloud which is removed background
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg (*filtered_cloud, output);

    output.header.frame_id = "livox_frame";
    pub_close_points.publish (output);

}

void cmdrawCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(msg->angular.z != 0)
    {
        turning_radius = msg->linear.x / msg->angular.z;
        linear_velocity = msg->linear.x;
    }
    else
    {
        turning_radius = 0;
        linear_velocity = msg->linear.x;
    }

    //update speed cmd
    geometry_msgs::Twist cmd_vel;
    double reduction_ratio = calculateSpeedReduction();
    cmd_vel.linear.x = msg->linear.x * reduction_ratio;
    cmd_vel.angular.z = msg->angular.z * reduction_ratio;
    pub_cmd_vel.publish(cmd_vel);

}

void goalDistCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    is_near_goal = ((uint)msg->data[0] > 0) ? true : false;
    goal_distance = msg->data[1] + offset_dist;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_obst_detector");
    ros::NodeHandle nh("~");
    nh.getParam("OBST_HIGH_MIN_Z", OBST_HIGHT_MIN_Z);
    nh.getParam("OBST_HIGH_MAX_Z", OBST_HIGHT_MAX_Z);
    if (!nh.getParam("obstacle_lim", obstacle_lim))
    {
        obstacle_lim = 10;
        ROS_WARN("Failed to get param 'obstacle_lim'");
    }

    pub_close_points = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 10);

    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);


    ros::Subscriber sub_points = nh.subscribe ("/livox/lidar", 10, cloudCB);
    ros::Subscriber sub_cmdraw = nh.subscribe("/cmd_raw", 10, cmdrawCallback);
    ros::Subscriber sub_goal_dist = nh.subscribe("/goal_dist",10,goalDistCallback);
    //detect obstacle by sonar
    ros::Subscriber sub_sonar_left = nh.subscribe("/scan_sonar_left",10, sonarLeftCallback);
    ros::Subscriber sub_sonar_center = nh.subscribe("/scan_sonar_center",10, sonarCenterCallback);
    ros::Subscriber sub_sonar_right = nh.subscribe("/scan_sonar_right",10, sonarRightCallback);

    ros::spin ();

    return 0;
}
