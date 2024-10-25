#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "om_cart/om_cart_state.h"
#include "om_cart.h"
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>

class DataSaver
{
public:
    DataSaver()
    {
        std::time_t t = std::time(nullptr);
        char data_time[100];
        std::strftime(data_time,sizeof(data_time), "%Y%m%d_%H_%M_%S", std::localtime(&t));
        //setup path to save log file
        std::string log_fileName = std::string("cart_log/log_") + data_time + ".txt";
        log_file.open(log_fileName, std::ios::out);

        if(!log_file.is_open())
        {
            ROS_ERROR("Failed to open file to save data");
            ros::shutdown();
        }

        odom_sub = nh_.subscribe("odom",10,&DataSaver::odomCallback,this);
        status_sub = nh_.subscribe("cart_status", 10, &DataSaver::cartStatusCall, this);

        pos_x = 0;
        pos_y = 0;
        pos_z = 0;
        //write log file header
        log_file << "TimeStamp(:m:s.ms)" << ";" << "x(m)" << ";" << "y(m)" << ";" << "z(m)" << ";"
                 << "v_line(m/sec)" << ";" << "v_turn(rad/sec)" << ";" << std::endl;
        ROS_INFO("Log file generated.");

    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
        pos_z = msg->pose.pose.position.z;
    }

    void cartStatusCall(const om_cart::om_cart_state::ConstPtr& msg)
    {
        Cart_Status_Info cart_data;
        uint16_t i;

        if((msg->type == 1) && (msg->size > 2))
        {
            ros::Time curr_time = ros::Time::now();
            time_t sec = curr_time.sec;
            int msec = curr_time.nsec / 1000000;
            struct tm* time_info = localtime(&sec);

            //save location and speed when speed data arrived
            for(i = 0; i < msg->size; i++)
            {
                cart_data.data[i] = msg->data[i];
            }
            log_file << std::fixed << time_info->tm_hour << ":" << time_info->tm_min << ":" << std::setprecision(3)
                     << time_info->tm_sec + (msec / 1000.0) << ";" << pos_x << ";" << pos_y << ";" << pos_z << ";"
                     << cart_data.cart_status.vel_line << ";" << cart_data.cart_status.vel_theta << ";" << std::endl;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Subscriber status_sub;
    std::ofstream log_file;
    float pos_x, pos_y, pos_z;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_amr_log");
    DataSaver data_saver;

    ros::spin();
    return 0;

}
