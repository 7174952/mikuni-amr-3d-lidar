#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
//#include <tf/tf.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <QFile>
#include <QTextStream>
#include <QString>
#include <QList>
#include <QRegularExpression>
#include <QThread>
#include <QDebug>

struct Navi_Waypoint_info
{
    QString waypoint_name;
    int wait_time;
    int delt_angle;
};
QList<Navi_Waypoint_info> navi_route_list;

bool cycle_en = false;
bool is_arrived = false;
enum Navi_State
{
    NAVI_STATE_INIT_WAIT = 0,
    NAVI_START_INIT_ROUTE,
    NAVI_STATE_WAIT_ARRIVED_WAYPOINT,
    NAVI_STATE_STOP_AND_WAIT,
    NAVI_STATE_ARRIVED_GOAL,
};

Navi_State navi_state = NAVI_STATE_INIT_WAIT;

int loadRouteScript(const QString& script_name)
{
    QFile file_script(script_name);
    QTextStream line_script_in(&file_script);

    if(!file_script.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        ROS_INFO("Load Navi script file: %s failed!", script_name.toStdString().c_str());
        return -1;
    }

    QStringList str_list;
    //Load cycle value
    if(!line_script_in.atEnd())
    {
        if(line_script_in.readLine().contains("true",Qt::CaseInsensitive))
        {
            cycle_en = true;
        }
        else
        {
            cycle_en = false;
        }
    }

    //load all route waypoint configuration
    while(!line_script_in.atEnd())
    {
        str_list = line_script_in.readLine().split(QRegularExpression("[:, ]+"));
        //at(0) - waypoint name, at(1) - wait time(sec), at(2) - change angle(deg)
        Navi_Waypoint_info item_tmp = {str_list.at(0), str_list.at(1).toInt(), str_list.at(2).toInt()};
        navi_route_list.push_back(item_tmp);
    }

    if(navi_route_list.size() < 2)
    {
        ROS_INFO("Load Navi waypoints num=%d not enough!",navi_route_list.size());
        return -1;
    }

    return 0;
}


nav_msgs::Path loadWaypoints(const QList<Navi_Waypoint_info>& sub_route, QString dir)
{
    nav_msgs::Path waypoints_list;
    waypoints_list.header.frame_id = "map";
    waypoints_list.header.stamp = ros::Time::now();


    QFile file_path;

    for(uint i = 0; i < sub_route.size() - 1; i++)
    {
        QString route_name = dir + "/base_" + sub_route.at(i).waypoint_name + "_" + sub_route.at(i + 1).waypoint_name + ".txt";
        file_path.setFileName(route_name);
        if(!file_path.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            ROS_ERROR("load route: %s", route_name.toStdString().c_str());
            ros::shutdown();
            return waypoints_list;
        }
        //Load route data
        QTextStream line_out(&file_path);
        while(!line_out.atEnd())
        {
            QStringList str_tmp = line_out.readLine().split(",");

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = str_tmp[0].toDouble();
            pose.pose.position.y = str_tmp[1].toDouble();
            pose.pose.position.z = str_tmp[2].toDouble();
            pose.pose.orientation.x = str_tmp[3].toDouble();
            pose.pose.orientation.y = str_tmp[4].toDouble();
            pose.pose.orientation.z = str_tmp[5].toDouble();
            pose.pose.orientation.w = str_tmp[6].toDouble();
            waypoints_list.poses.push_back(pose);
        }
        file_path.close();
    }

    if(sub_route.last().delt_angle != 0)
    {
        tf::Quaternion quat_original(waypoints_list.poses.back().pose.orientation.x,
                                     waypoints_list.poses.back().pose.orientation.y,
                                     waypoints_list.poses.back().pose.orientation.z,
                                     waypoints_list.poses.back().pose.orientation.w);
        tf::Quaternion quat_delt;
        quat_delt.setRPY(0,0,(sub_route.last().delt_angle / 180.0) *M_PI);
        tf::Quaternion quat_new = quat_original * quat_delt;
        //append last waypoint
        geometry_msgs::PoseStamped pose_last;
        pose_last.header.frame_id = "map";
        pose_last.pose.position.x = waypoints_list.poses.back().pose.position.x;
        pose_last.pose.position.y = waypoints_list.poses.back().pose.position.y;
        pose_last.pose.position.z = waypoints_list.poses.back().pose.position.z;
        pose_last.pose.orientation.x = quat_new.getX();
        pose_last.pose.orientation.y = quat_new.getY();
        pose_last.pose.orientation.z = quat_new.getZ();
        pose_last.pose.orientation.w = quat_new.getW();
        waypoints_list.poses.push_back(pose_last);

        qDebug() << "z rotate:" << sub_route.last().delt_angle << "degree";
        qDebug() << "Append waypoint: x=" << waypoints_list.poses.back().pose.position.x
                 << "y=" << waypoints_list.poses.back().pose.position.y
                 << "z=" << waypoints_list.poses.back().pose.position.z
                 << "ori_x=" << waypoints_list.poses.back().pose.orientation.x
                 << "ori_y=" << waypoints_list.poses.back().pose.orientation.y
                 << "ori_z=" << waypoints_list.poses.back().pose.orientation.z
                 << "ori_w=" << waypoints_list.poses.back().pose.orientation.w;
    }

    ROS_INFO("Fetched %lu waypoints", waypoints_list.poses.size());

    if (waypoints_list.poses.empty())
    {
        ROS_WARN("No waypoint to draw... Shutdown");
        ros::shutdown();
    }

    return waypoints_list;
}

void state_callback(const std_msgs::Bool::ConstPtr& msg)
{
    is_arrived = msg->data;
    qDebug() << "callback is_arrived=" << is_arrived;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoints_loader");
    ros::NodeHandle nh;

    std::string waypoint_path;
    nh.param<std::string>("load_waypoint/waypoint_path", waypoint_path, "");

    if (waypoint_path.empty())
    {
        ROS_ERROR("Waypoint route script is missing.");
        return -1;
    }

    if(loadRouteScript(QString::fromStdString(waypoint_path)) != 0)
    {
        ROS_ERROR("Load Navi route script failed!");
        return -1;
    }

    nav_msgs::Path waypoints_info;
    QString sub_route_dir = QString::fromStdString(waypoint_path).replace("navi_route","base_route");
    sub_route_dir = sub_route_dir.left(sub_route_dir.lastIndexOf("/"));


    ros::Publisher waypoint_pub = nh.advertise<nav_msgs::Path>("/waypoints", 1);
    ros::ServiceClient reset_path_client = nh.serviceClient<std_srvs::Empty>("reset_path");
    std_srvs::Empty srv;
    ros::Subscriber reached_goal_pub = nh.subscribe("/reached_goal",10, state_callback);

    ros::Rate rate(1);
    QList<Navi_Waypoint_info> sub_route;
    uint wp_index = 0;
    ros::Time last_time = ros::Time::now();
    ros::Time curr_time;

    QThread::sleep(1);

    navi_state = (navi_route_list.at(wp_index).wait_time > 0) ? NAVI_STATE_INIT_WAIT : NAVI_START_INIT_ROUTE;

    while (ros::ok())
    {
        switch(navi_state)
        {
        case NAVI_STATE_INIT_WAIT:
            curr_time = ros::Time::now();
            if((curr_time.toSec() - last_time.toSec()) > navi_route_list.at(wp_index).wait_time)
            {
                navi_state = NAVI_START_INIT_ROUTE;
            }
            break;
        case NAVI_START_INIT_ROUTE:
            sub_route.clear();
            sub_route.push_back(navi_route_list.at(wp_index));
            wp_index++;
            //seach not stop waypoints
            while((wp_index < navi_route_list.size() - 1) && (navi_route_list.at(wp_index).wait_time == 0))
            {
                sub_route.push_back(navi_route_list.at(wp_index));
                wp_index++;
            }
            sub_route.push_back(navi_route_list.at(wp_index));
            //get sub path route
            //debug
            qDebug() << "Current Route Path:" << sub_route.first().waypoint_name + "->" + sub_route.last().waypoint_name;

            waypoints_info = loadWaypoints(sub_route, sub_route_dir);
            //debug
            qDebug() << "waypoints_info size:" << waypoints_info.poses.size();

            waypoint_pub.publish(waypoints_info);

            // request:pure_pursuit to reset path
            if(reset_path_client.call(srv))
            {
                ROS_INFO("load_waypoint: Reset path succeeded.");
            }
            else
            {
                ROS_WARN("load_waypoint: Reset path failed.");
            }

            navi_state = NAVI_STATE_WAIT_ARRIVED_WAYPOINT;
            break;
        case NAVI_STATE_WAIT_ARRIVED_WAYPOINT:
            //get next sub route
            if(is_arrived)
            {
                is_arrived = false;
                //check final waypoint
                if(navi_route_list.at(wp_index).wait_time < 0) //-1: final stopped
                {
                    navi_state = NAVI_STATE_ARRIVED_GOAL;
                }
                else
                {
                    //Stop and wait
                    last_time = ros::Time::now();
                    navi_state = NAVI_STATE_STOP_AND_WAIT;
                }
            }
            break;
        case NAVI_STATE_STOP_AND_WAIT:
            curr_time = ros::Time::now();
            if((curr_time.toSec() - last_time.toSec()) > navi_route_list.at(wp_index).wait_time)
            {
                //prepare for next sub route data
                sub_route.clear();
                sub_route.push_back(navi_route_list.at(wp_index));
                wp_index++;
                //seach not stop waypoints
                while((wp_index < navi_route_list.size() - 1) && (navi_route_list.at(wp_index).wait_time == 0))
                {
                    sub_route.push_back(navi_route_list.at(wp_index));
                    wp_index++;
                }
                sub_route.push_back(navi_route_list.at(wp_index));
                //get next sub path route

                //debug
                qDebug() << "Current Route Path:" << sub_route.first().waypoint_name + "->" + sub_route.last().waypoint_name;

                waypoints_info = loadWaypoints(sub_route, sub_route_dir);
                waypoint_pub.publish(waypoints_info);
                // request:pure_pursuit to reset path
                if(reset_path_client.call(srv))
                {
                    ROS_INFO("load_waypoint: Reset path succeeded.");
                }
                else
                {
                    ROS_WARN("load_waypoint: Reset path failed.");
                }

                navi_state = NAVI_STATE_WAIT_ARRIVED_WAYPOINT;
            }
            break;
        case NAVI_STATE_ARRIVED_GOAL:
        default:
            if(cycle_en)
            {
                last_time = ros::Time::now();
                wp_index = 0;

                navi_state = NAVI_STATE_INIT_WAIT;
            }
            else
            {
                ROS_INFO("Reached Goal!\n Navigation is Over!");
                ros::shutdown();
            }
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
