#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/distances.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

#include <cmath>

double OBST_HIGH_MIN_Z = -0.1; //default: -0.1
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (0.5f);

ros::Publisher pub_diff_points;
ros::Publisher pub_course_points;
ros::Publisher pub_course_points_num;

double linear_velocity = 0;
double turning_radius = 0;

Eigen::Affine3f transform;

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{

    // Convert the sensor_msgs::PointCloud2 data to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input,*cloud_input);

    // Create the PassThrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    // Set the axis to filter along
    pass.setFilterFieldName("z");
    // Set the range of acceptable values
    pass.setFilterLimits(OBST_HIGH_MIN_Z, 1.0);
    // Set the input cloud
    pass.setInputCloud(cloud_input);
    pass.filter(*cloud_input);


    octree.setInputCloud (cloud_map); // Is this right code?
    octree.addPointsFromInputCloud ();
    octree.switchBuffers ();  // Switch to cloud_input
    octree.setInputCloud (cloud_input);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);
    octree.switchBuffers ();  // Switch to cloud_???

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < newPointIdxVector.size (); ++i)
        cloud_diff->points.push_back(cloud_input->points[newPointIdxVector[i]]);

    cloud_diff->width = cloud_diff->points.size();
    cloud_diff->height = 1;
    cloud_diff->is_dense = true;

    //Publish point cloud which is removed background
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_diff, output);
    output.header.frame_id = "map";
    pub_diff_points.publish (output);


    // Apply the transformation to the point cloud,map->base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_diff, *transformed_cloud, transform);


    const double width = 0.52;
    const double ratio = 8.0;

    // Create a new point cloud for the filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //If robot turn, consider to ring area
    //TODO Replace sin crop box to angle crop box
    if (turning_radius != 0)
    {
        double center_angle;
        center_angle = (linear_velocity * ratio) / turning_radius;

        pcl::CropBox<pcl::PointXYZ> boxFilter;

        if (turning_radius > 0) {
            // Set min and max for positive turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, -width/2, OBST_HIGH_MIN_Z, 1.0));
            boxFilter.setMax(Eigen::Vector4f(turning_radius * std::sin(center_angle), turning_radius, 1.0, 1.0));
        }
        else if (turning_radius < 0){
            // Set min and max for negative turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, turning_radius, OBST_HIGH_MIN_Z, 1.0));
            boxFilter.setMax(Eigen::Vector4f(turning_radius * std::sin(center_angle), width/2, 1.0, 1.0));
        }
        boxFilter.setInputCloud(transformed_cloud);
        boxFilter.filter(*transformed_cloud);

        double inner_radius = abs(turning_radius) - width/2; // Set your desired distance
        double outer_radius = abs(turning_radius) + width/2; // Set your desired distance


        // Iterate over the points in the point cloud
        for (const auto& point : transformed_cloud->points)
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
        boxFilter.setMin(Eigen::Vector4f(0.0, -width/2, OBST_HIGH_MIN_Z, 1.0));
        boxFilter.setMax(Eigen::Vector4f(linear_velocity*ratio, width/2, 1.0, 1.0));
        boxFilter.setInputCloud(transformed_cloud);
        boxFilter.filter(*filtered_cloud);

    }

    pcl::toROSMsg (*filtered_cloud, output);
    output.header.frame_id = "livox_frame";
    pub_course_points.publish (output);

    size_t num_points = filtered_cloud->points.size();
    std_msgs::Float32 msg;
    msg.data = num_points;
    pub_course_points_num.publish(msg);
}

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Now you can use turning_radius and linear_velocity
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
}

void timerCallback(const ros::TimerEvent&)
{
    // Create a TransformListener
    tf::TransformListener listener;

    // Wait for the transform to be available
    ros::Duration(0.5).sleep();

    // Get the transformation
    tf::StampedTransform transform_tf;
    try
    {
        listener.lookupTransform("livox_frame", "map", ros::Time(0), transform_tf);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    // Convert the tf::Transform to an Eigen::Affine3d
    Eigen::Affine3d transform_d;
    tf::transformTFToEigen(transform_tf, transform_d);

    // Convert the Eigen::Affine3d to an Eigen::Affine3f
    transform = transform_d.cast<float>();
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_change_detector");
    ros::NodeHandle nh;

    nh.getParam("OBST_HIGH_MIN_Z", OBST_HIGH_MIN_Z);

    std::string pcd_file_path;
    if (!ros::param::get("pcd_file_path", pcd_file_path)) {
        ROS_ERROR("Failed to get param pcd_file_path");
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_path, *cloud_map) == -1)
    {
        PCL_ERROR ("Couldn't read the file cloud_map.pcd \n");
        return (-1);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_map);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_map);


    pub_diff_points = nh.advertise<sensor_msgs::PointCloud2>("diff_points", 10);
    pub_course_points = nh.advertise<sensor_msgs::PointCloud2>("course_points", 10);
    pub_course_points_num = nh.advertise<std_msgs::Float32>("course_points_num", 10);

    ros::Subscriber sub_points = nh.subscribe ("/aligned_points", 10, cloudCB);
    ros::Subscriber sub = nh.subscribe("cmd_raw", 10, callback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
    ros::spin ();

    return 0;
}
