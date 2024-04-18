#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/distances.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

#include <cmath>

double OBST_HIGHT_MIN_Z = -0.1; //default: -0.1
double OBST_HIGHT_MAX_Z = 2.0;  //default: 1.0
double OBST_WIDTH_MAX = 20;
double OBST_LENGTH_MAX = 10;

ros::Publisher obstacle_filter_pub;

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


    // Create a new point cloud for the filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //If robot go straight, consider to box area
    {
        // Create the filtering object
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(0.0, -OBST_WIDTH_MAX/2, OBST_HIGHT_MIN_Z, 1.0));
        boxFilter.setMax(Eigen::Vector4f(OBST_LENGTH_MAX, OBST_WIDTH_MAX/2, OBST_HIGHT_MAX_Z, 1.0));
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*filtered_cloud);
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    //Publish point cloud which is removed background
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg (*filtered_cloud, output);

    output.header.frame_id = "livox_frame";
    obstacle_filter_pub.publish (output);

}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "local_obstacle_filter");
    ros::NodeHandle nh("~");
    nh.getParam("OBST_HIGH_MIN_Z", OBST_HIGHT_MIN_Z);
    nh.getParam("OBST_HIGH_MAX_Z", OBST_HIGHT_MAX_Z);

    obstacle_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_scan", 10);

    ros::Subscriber sub_points = nh.subscribe ("/livox/lidar", 10, cloudCB);

    ros::spin ();

    return 0;
}
