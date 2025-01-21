#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

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

struct Navi_Route_Status
{
    QString sub_route;         //A-B, From A, To B,A->B
    QString robot_state;       //start,stop, running
    QString current_location;  //A or B, or AB
    QString route_finished;    //true or false
    QString is_auto_pass;      //true or false at waypoint
    QString is_front_careless; //true or false
    QString is_end_careless;   //true or false
    void clear()
    {
        sub_route = "";
        robot_state = "";
        current_location = "";
        route_finished = "";
        is_auto_pass = "";
        is_front_careless = "";
        is_end_careless = "";
    }
};
Navi_Route_Status navi_route_status;

bool auto_next_target = false;
bool srv_req_next_target = false;
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
            auto_next_target = true;
        }
        else
        {
            auto_next_target = false;
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
        waypoints_list.poses.back().pose.orientation.x = quat_new.getX();
        waypoints_list.poses.back().pose.orientation.y = quat_new.getY();
        waypoints_list.poses.back().pose.orientation.z = quat_new.getZ();
        waypoints_list.poses.back().pose.orientation.w = quat_new.getW();
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

bool next_target_Request(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    srv_req_next_target = true;
    return true;
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
    ros::ServiceServer req_next_target_srv = nh.advertiseService("/req_next_target",next_target_Request);
    std_srvs::Empty srv;
    ros::Subscriber reached_goal_pub = nh.subscribe("/reached_goal",10, state_callback);
    ros::Publisher navi_status_pub = nh.advertise<std_msgs::String>("/navi_status",1);
    std_msgs::String msg_navi_route_status;
    navi_route_status = {"","stop",navi_route_list.front().waypoint_name,"false"};

    ros::Rate rate(1);
    QList<Navi_Waypoint_info> sub_route;
    uint wp_index = 0;
    ros::Time last_time = ros::Time::now();
    ros::Time curr_time;

    QThread::sleep(1);

    navi_state = NAVI_STATE_INIT_WAIT;

    while (ros::ok())
    {
        switch(navi_state)
        {
        case NAVI_STATE_INIT_WAIT:
            curr_time = ros::Time::now();
            if(    ((auto_next_target == true) && ((curr_time.toSec() - last_time.toSec()) > navi_route_list.at(wp_index).wait_time + 1))
                || ((auto_next_target == false) && (srv_req_next_target == true)))
            {
                srv_req_next_target = false;

                navi_state = NAVI_START_INIT_ROUTE;
            }
            navi_route_status.robot_state = "stop";
            navi_route_status.current_location = navi_route_list.front().waypoint_name;
            navi_route_status.route_finished = "false";
            navi_route_status.is_auto_pass = "false";
            navi_route_status.is_front_careless = "true";
            navi_route_status.is_end_careless = "true";

            break;
        case NAVI_START_INIT_ROUTE:
            sub_route.clear();
            sub_route.push_back(navi_route_list.at(wp_index));
            wp_index++;
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

            //set navigation status
            navi_route_status.clear();
            //for(const Navi_Waypoint_info info : sub_route)
            for(uint i = 0; i < sub_route.size(); i++)
            {
                navi_route_status.sub_route.append(sub_route.at(i).waypoint_name);
                if(i < sub_route.size() - 1)
                {
                    navi_route_status.sub_route.append("->");
                }
            }
            navi_route_status.robot_state = "start";
            navi_route_status.current_location = sub_route.front().waypoint_name;
            navi_route_status.route_finished = "false";
            navi_route_status.is_auto_pass = "false";
            navi_route_status.is_front_careless = "true";
            navi_route_status.is_end_careless = (sub_route.last().wait_time != 0) ? "true" : "false";

            navi_state = NAVI_STATE_WAIT_ARRIVED_WAYPOINT;
            break;
        case NAVI_STATE_WAIT_ARRIVED_WAYPOINT:
            //get next sub route
            if(is_arrived)
            {
                //update navigation status
                navi_route_status.robot_state = "stop";
                navi_route_status.current_location = sub_route.back().waypoint_name;
                navi_route_status.is_auto_pass = "false";

                //check final waypoint
                if(navi_route_list.at(wp_index).wait_time < 0) //-1: final stopped
                {
                    navi_route_status.route_finished = "true";
                    navi_state = NAVI_STATE_ARRIVED_GOAL;
                }
                else if(navi_route_list.at(wp_index).wait_time == 0)
                {
                    //auto continue to run ton next target
                    navi_route_status.robot_state = "running";
                    navi_route_status.route_finished = "false";
                    navi_route_status.is_auto_pass = "true";
                    navi_state = NAVI_STATE_STOP_AND_WAIT;
                }
                else
                {
                    navi_route_status.route_finished = "false";
                    //Stop and wait
                    last_time = ros::Time::now();
                    navi_state = NAVI_STATE_STOP_AND_WAIT;
                }

                is_arrived = false;
            }
            else
            {
                //update navigation status
                navi_route_status.robot_state = "running";
                navi_route_status.current_location = navi_route_status.sub_route;
                navi_route_status.route_finished = "false";
            }

            break;
        case NAVI_STATE_STOP_AND_WAIT:
            curr_time = ros::Time::now();
            if(    ((auto_next_target == true) && ((curr_time.toSec() - last_time.toSec()) > navi_route_list.at(wp_index).wait_time))
                || ((auto_next_target == false) && (srv_req_next_target == true))
                || (navi_route_status.is_auto_pass == "true"))
            {
                //prepare for next sub route data
                sub_route.clear();
                sub_route.push_back(navi_route_list.at(wp_index));
                wp_index++;
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

                //set navigation status
                navi_route_status.clear();
                for(uint i = 0; i < sub_route.size(); i++)
                {
                    navi_route_status.sub_route.append(sub_route.at(i).waypoint_name);
                    if(i < sub_route.size() - 1)
                    {
                        navi_route_status.sub_route.append("->");
                    }
                }
                navi_route_status.robot_state = "start";
                navi_route_status.current_location = sub_route.front().waypoint_name;
                navi_route_status.route_finished = "false";
                navi_route_status.is_auto_pass = "false";

                navi_route_status.is_front_careless = (sub_route.first().wait_time != 0) ? "true" : "false";
                navi_route_status.is_end_careless = (sub_route.last().wait_time != 0) ? "true" : "false";


                srv_req_next_target = false;
                navi_state = NAVI_STATE_WAIT_ARRIVED_WAYPOINT;
            }
            break;
        case NAVI_STATE_ARRIVED_GOAL:
        default:
            ROS_INFO("Reached Goal!\n Navigation is Over!");
            ros::shutdown();
        }

        msg_navi_route_status.data =  navi_route_status.sub_route.toStdString() + ";"
                                   +  navi_route_status.robot_state.toStdString() + ";"
                                   +  navi_route_status.current_location.toStdString() + ";"
                                   +  navi_route_status.route_finished.toStdString() + ";"
                                   +  navi_route_status.is_auto_pass.toStdString() + ";"
                                   +  navi_route_status.is_front_careless.toStdString() + ";"
                                   +  navi_route_status.is_end_careless.toStdString();

        navi_status_pub.publish(msg_navi_route_status);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
