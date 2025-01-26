#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

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

#include <QString>
#include <QList>
#include <QRegularExpression>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>

double OBST_HIGHT_MIN_Z = -0.1; //default: -0.1
double OBST_HIGHT_MAX_Z = 1.0;  //default: 1.0

ros::Publisher pub_close_points;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_obst_points_num;
ros::Publisher pub_camera_ctrl_status;

double linear_velocity = 0;
double turning_radius = 0;
double goal_distance = 0;
double offset_dist = 0.1;
bool is_near_goal = false;
int32_t obstacle_points_num = 0;
int32_t obstacle_lim = 0;

double robot_width_size;
double robot_width_tolerance;

double set_robot_stop;
double obst_speed_rate = 0; //0.0~1.0

//Control speed by the distance of person behind of cart
struct Speed_Ctrl
{
    bool guide_enable;
    double idealDistance;
    double stopDistance;
    double kp;
    double threshHold;
    double startThreshHold;
    double guideless_distance;
    double guideSpeedRate;  // 当前实际速度(或速度比例)
    bool runStatus;
};
Speed_Ctrl speed_ctrl;
double currentDistance;
uint person_num;
uint person_num_curr;
uint person_num_last;
int detect_cnt = -1;
const uint DETECT_MAX_TIMES = 5;

QString log_fileName;

//control by user voice command
struct Voice_Ctrl
{
    bool voice_ctrl_enable;
    bool start_cart;

};
Voice_Ctrl voice_ctrl;

struct Odom_Limit_Status
{
    double odom_distance;
    double total_distance;
    bool limit_on;
    double max_vel;
    double obst_tolerance;
    bool is_front_careless;
    bool is_end_careless;
    void reset()
    {
        odom_distance = 0;
        total_distance = 0;
        limit_on = false;
        max_vel = 0;
        obst_tolerance = 0;
        is_front_careless  = true;
        is_end_careless = true;

    }
};
Odom_Limit_Status odom_limit_status;

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
    if (obstacle_points_num > obstacle_lim)
    {
        obst_speed_rate -= 0.25;
        obst_speed_rate = std::max(obst_speed_rate,0.0);
    }
    else
    {
        obst_speed_rate = 1.0;
    }

    return obst_speed_rate;
}

void voiceCtrlCallback(const std_msgs::String::ConstPtr& cmd)
{
    qDebug() << "detect obstacle > robot voice cmd:" << QString::fromStdString(cmd->data) << "\n";

    if(cmd->data == "start")
    {
        voice_ctrl.start_cart = true;
    }
    else if(cmd->data == "stop")
    {
        voice_ctrl.start_cart = false;
    }
    else
    {
        //do nothing
    }
}

void personInfoCallback(const std_msgs::String::ConstPtr& msg)
{
    QString person_msg = QString::fromStdString(msg->data);
    if(person_msg.isEmpty()) return;

    double nearDistance = qInf();
    QStringList persons = person_msg.trimmed().split(QRegularExpression("[!;]+"));
    person_num_curr = persons.takeFirst().split(" ").at(1).toUInt();
    if((person_num_curr == 0 && person_num_last > 0) || (person_num_curr > 0 && person_num_last == 0))
    {
        detect_cnt = 0;
    }
    else
    {
        if(detect_cnt >= 0)
            detect_cnt++;
    }

    if(detect_cnt > DETECT_MAX_TIMES)
    {
        person_num = person_num_curr;
        // if(persons.isEmpty()) return;
        if(person_num > 0)
        {
            //find the nearest person
            for(const QString& record : persons)
            {
                QStringList str_tmp = record.trimmed().split(QRegularExpression("[:= ]+"));

                nearDistance = qMin(str_tmp.at(str_tmp.indexOf("Distance") + 1).toDouble(), nearDistance);

            }

            currentDistance = nearDistance;
        }

    }

    person_num_last = person_num_curr;

#if 1 //debug_ryu
    QFile log_file(log_fileName);
    if (!log_file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "无法打开文件:" << log_fileName;
        return;
    }

    QTextStream out(&log_file);
    out << QDateTime::currentDateTime().toString("hh:mm:ss");
    out << " Person Info Msg:" << person_msg << "\n";
    out << "detect_cnt:" << detect_cnt << " person_num:" << person_num << " currentDistance:" << currentDistance << "\n";
    log_file.close();

#endif

}

void odomLimit_Callback(const std_msgs::String::ConstPtr& msg)
{
    QStringList odom_limit = QString::fromStdString(msg->data).split(";");

    if(odom_limit.size() < 7)
        return;

    odom_limit_status.odom_distance = odom_limit.at(0).split(":").at(1).toDouble();
    odom_limit_status.total_distance = odom_limit.at(1).split(":").at(1).toDouble();
    odom_limit_status.limit_on = (odom_limit.at(2).split(":").at(1) == "1");
    odom_limit_status.max_vel = odom_limit.at(3).split(":").at(1).toDouble();
    odom_limit_status.obst_tolerance = odom_limit.at(4).split(":").at(1).toDouble();
    odom_limit_status.is_front_careless = (odom_limit.at(5).split(":").at(1) == "1");
    odom_limit_status.is_end_careless = (odom_limit.at(6).split(":").at(1) == "1");

    ROS_INFO("odom_limit msg: %s",msg->data.c_str());

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
    double obstal_width = robot_width_size + (odom_limit_status.limit_on ? odom_limit_status.obst_tolerance : robot_width_tolerance) * 2;

    const double ratio = (odom_limit_status.limit_on && odom_limit_status.obst_tolerance < 0.03) ? 3.0 : 2.0;

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
            boxFilter.setMin(Eigen::Vector4f(0, -obstal_width/2, OBST_HIGHT_MIN_Z, 1.0));
            max_filter_dist = is_near_goal ? goal_distance : turning_radius * std::sin(center_angle);
            boxFilter.setMax(Eigen::Vector4f(max_filter_dist, turning_radius, OBST_HIGHT_MAX_Z, 1.0));

        }
        else if (turning_radius < 0){
            // Set min and max for negative turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, turning_radius, OBST_HIGHT_MIN_Z, 1.0));
            max_filter_dist = is_near_goal ? goal_distance : turning_radius * std::sin(center_angle);
            boxFilter.setMax(Eigen::Vector4f(max_filter_dist, obstal_width/2, OBST_HIGHT_MAX_Z, 1.0));

        }
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*cloud_input);

        double inner_radius = abs(turning_radius) - obstal_width/2; // Set your desired distance
        double outer_radius = abs(turning_radius) + obstal_width/2; // Set your desired distance


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
        boxFilter.setMin(Eigen::Vector4f(0.0, -obstal_width/2, OBST_HIGHT_MIN_Z, 1.0));
        max_filter_dist = is_near_goal ? goal_distance : linear_velocity*ratio;
        boxFilter.setMax(Eigen::Vector4f(max_filter_dist, obstal_width/2, OBST_HIGHT_MAX_Z, 1.0));
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*filtered_cloud);

    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    obstacle_points_num = filtered_cloud->points.size();
    //publish detected obstacle points total number
    std_msgs::Int32 points_num;
    points_num.data = obstacle_points_num;
    pub_obst_points_num.publish(points_num);

    //Publish point cloud which is removed background
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg (*filtered_cloud, output);

    output.header.frame_id = "livox_frame";
    pub_close_points.publish (output);

}

void cmdrawCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    static int stopDelayCounter = 0;         // 停止延时计数器
    const int STOP_DELAY = 15;               // 假设主循环 10Hz，延时 1 秒

    //update speed cmd by
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z;
    if(odom_limit_status.limit_on)
    {
       cmd_vel.linear.x = std::min(msg->linear.x, odom_limit_status.max_vel);
    }

    if(cmd_vel.angular.z != 0)
    {
        turning_radius = cmd_vel.linear.x / cmd_vel.angular.z;
        linear_velocity = cmd_vel.linear.x;
    }
    else
    {
        turning_radius = 0;
        linear_velocity = cmd_vel.linear.x;
    }

    // double guide_speed_rate = 1.0;
    // 取出/使用上次的速度值(保存在 speed_ctrl 里)
    double guide_speed_rate = speed_ctrl.guideSpeedRate;
    bool   run_status       = speed_ctrl.runStatus;


    if(speed_ctrl.guide_enable)
    {
        // bool run_status = true;

        if( (odom_limit_status.is_front_careless && (odom_limit_status.odom_distance < speed_ctrl.guideless_distance)) //when start to run
         || (odom_limit_status.is_end_careless && (odom_limit_status.odom_distance > (odom_limit_status.total_distance - speed_ctrl.guideless_distance)))) //when near target
        {
            //do nothing
            run_status = true;
        }
        else
        {
#if 1

            if (person_num > 0)
            {
                // 如果还在“停止延时计数”当中，就必须强制保持停止，不允许重启
                if (stopDelayCounter > 0)
                {
                    // 这里进入延时滤波阶段：不计算新的 rate，不启动
                    stopDelayCounter--;

                    guide_speed_rate = 0.0;  // 强制速度为 0
                    run_status = false;

                }
                else
                {
                    // 未处于延时停止阶段，则正常计算 rate
                    double dist_error = (speed_ctrl.stopDistance - speed_ctrl.startThreshHold) - currentDistance;
                    double rate = dist_error / (speed_ctrl.stopDistance - speed_ctrl.idealDistance);

                     // rate < 0：需要停止并启动延时滤波
                    if (rate <= 0.0)
                    {
                        // 需要停止，但这次我们不立即设速度=0，而是“逐周期减速”
                        // 如果当前速度还>0.0，就缓慢减 0.1
                        if (guide_speed_rate > 0.0)
                        {
                            guide_speed_rate -= 0.1; // 每次减 0.1
                            if (guide_speed_rate < 0.0)
                            {
                                guide_speed_rate = 0.0; // 最终不能为负
                            }
                            run_status = true;
                        }
                        else
                        {
                            // 速度已经为 0，则进入延时停滞阶段
                            guide_speed_rate = 0.0;
                            run_status       = false;

                            // 重置 startThreshHold
                            speed_ctrl.startThreshHold = speed_ctrl.threshHold;

                            // 启动延时计数
                            stopDelayCounter = STOP_DELAY;
                        }
                    }
                    else //rate > 0
                    {
                        // 限幅到 [0,1]
                        if (rate > 1.0)
                        {
                            rate = 1.0;
                        }

                        guide_speed_rate = speed_ctrl.kp * rate;
                        // rate >= 0：正常跑
                        speed_ctrl.startThreshHold = 0.0;
                        run_status = true;
                    }
                }
            }
            else // person_num == 0
            {
                // (C) person_num == 0, 无人 => 也要减速到 0
                if (guide_speed_rate > 0.0)
                {
                    guide_speed_rate -= 0.1;
                    if (guide_speed_rate < 0.0) {
                        guide_speed_rate = 0.0;
                    }
                    run_status = true;
                }
                else
                {
                    // 一旦减为 0，进入延时
                    guide_speed_rate = 0.0;
                    run_status       = false;
                    stopDelayCounter = STOP_DELAY;

                }

            }

            // 将本次结果写回 speed_ctrl
            speed_ctrl.guideSpeedRate = guide_speed_rate;
            speed_ctrl.runStatus      = run_status;

#else
            if(person_num > 0)
            {
                double dist_error = (speed_ctrl.stopDistance - speed_ctrl.startThreshHold) - currentDistance;
                double rate = dist_error/(speed_ctrl.stopDistance - speed_ctrl.idealDistance);
                if(rate < 0)
                {
                    rate = 0;
                    speed_ctrl.startThreshHold = speed_ctrl.threshHold;
                    run_status = false;
                }
                else //rate > 0
                {
                    speed_ctrl.startThreshHold = 0.0;
                    run_status = true;
                }

                if(rate > 1)
                {
                    rate = 1;
                }
                guide_speed_rate = speed_ctrl.kp * rate;
                last_rate = guide_speed_rate;

                qDebug() << "currentDistance:" << currentDistance << "dist_error:" << dist_error << "rate:" << guide_speed_rate << "\n";
            }
            else //person_num == 0, stop
            {
                guide_speed_rate = 0.0;
                // run_status = false;

            }
#endif
        }

        std_msgs::String camera_ctrl_msg;
        camera_ctrl_msg.data = "person_num:" + QString::number(person_num).toStdString() + ";"
                            + "run_status:" + QString::number(run_status).toStdString();
        pub_camera_ctrl_status.publish(camera_ctrl_msg);

    }

    double reduction_ratio = calculateSpeedReduction();
    cmd_vel.linear.x = cmd_vel.linear.x * reduction_ratio;
    cmd_vel.angular.z = cmd_vel.angular.z * reduction_ratio;

    //update by people's distance behind cart
    if(speed_ctrl.guide_enable)
    {
        //when stop or turn round, not control by behind person
        if(std::abs(cmd_vel.linear.x) > 0)
        {
            cmd_vel.linear.x = guide_speed_rate * cmd_vel.linear.x;
            cmd_vel.angular.z = guide_speed_rate * cmd_vel.angular.z;
        }
    }

    //update by voice control comamnd
    if(voice_ctrl.voice_ctrl_enable)
    {
        if( (odom_limit_status.is_front_careless && (odom_limit_status.odom_distance < speed_ctrl.guideless_distance)) //when start to run
         || (odom_limit_status.is_end_careless && (odom_limit_status.odom_distance > (odom_limit_status.total_distance - speed_ctrl.guideless_distance)))) //when near target
        {
            //do nothing
        }
        else
        {
            if(voice_ctrl.start_cart == false)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
        }

    }

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
    speed_ctrl.guide_enable = false;
    speed_ctrl.startThreshHold = 0;
    speed_ctrl.guideSpeedRate = 1.0;
    speed_ctrl.runStatus = true;
    nh.param("guide_enable", speed_ctrl.guide_enable, false);
    nh.param("idealDistance", speed_ctrl.idealDistance, 2.0);
    nh.param("stopDistance", speed_ctrl.stopDistance, 4.0);
    nh.param("kp", speed_ctrl.kp, 1.0);
    nh.param("threshHold",speed_ctrl.threshHold,0.5);
    voice_ctrl.voice_ctrl_enable = false;
    nh.param("voice_ctrl_enable", voice_ctrl.voice_ctrl_enable, false);
    nh.param("robot_width_size", robot_width_size, 0.52);
    nh.param("robot_width_tolerance", robot_width_tolerance, 0.1);
    nh.param("guideless_distance", speed_ctrl.guideless_distance, 4.0);

    pub_close_points = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 10);

    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_obst_points_num = nh.advertise<std_msgs::Int32>("/obstacle_points_num",10);
    pub_camera_ctrl_status = nh.advertise<std_msgs::String>("/camera_ctrl_status",10);

    odom_limit_status.odom_distance = 0.0;
    odom_limit_status.total_distance = 0.0;
    odom_limit_status.limit_on = false;
    odom_limit_status.max_vel = 0.0;
    odom_limit_status.obst_tolerance = 0.0;

#if 1 //debug_ryu
    log_fileName = "/home/mikuni/catkin_ws/logs/detect_" + QDateTime::currentDateTime().toString("yyyyMMdd_hh_mm") + ".txt";
#endif

    ros::Subscriber sub_points = nh.subscribe ("/livox/lidar", 10, cloudCB);
    ros::Subscriber sub_cmdraw = nh.subscribe("/cmd_raw", 10, cmdrawCallback);
    ros::Subscriber sub_goal_dist = nh.subscribe("/goal_dist",10,goalDistCallback);
    ros::Subscriber sub_odom_limit = nh.subscribe("/odom_limit",10, odomLimit_Callback);
    //detect person and distance
    ros::Subscriber sub_back_person_info = nh.subscribe("/person_detection_info",10, personInfoCallback);
    //wait voice control command
    ros::Subscriber sub_voice_ctrl = nh.subscribe("/voice_ctrl_cmd",10, voiceCtrlCallback);
    voice_ctrl.start_cart = true;

    ros::spin ();

    return 0;
}
