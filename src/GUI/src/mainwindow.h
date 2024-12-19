#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Qt Set
//#include <QMainWindow>
#include <QProcess>
#include <QDebug>
#include <QMessageBox>
#include <QFile>
#include <QDir>
#include <QFileDialog>
#include <QWidget>
#include <QCloseEvent>
#include <QTextStream>
#include <QRegularExpression>
#include <QList>
#include <QVBoxLayout>
#include <QButtonGroup>
#include <QtMultimedia/QMediaPlayer>
#include <QtMultimedia/QMediaPlaylist>
#include <QThread>
#include <QSettings>
#include <QStandardPaths>
#include <QMap>

#include <sys/stat.h>

//ROS Set
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
//QT_END_NAMESPACE

class MainWindow : public QWidget
{
    Q_OBJECT
    ros::NodeHandle nh_;
    ros::Publisher pub_voice_mode;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_reach_goal_;
    ros::Subscriber sub_voice_order_;
    ros::Subscriber sub_navi_status;
    ros::Subscriber sub_obstacle_detect;
    ros::ServiceClient client_request_next_target;


public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void terminate_process(QProcess*);
    void start_process(QProcess*, QString);
    void closeEvent(QCloseEvent*);
    void Odometry_CallBack(const nav_msgs::Odometry&);
    void obstacle_CallBack(const std_msgs::Int32::ConstPtr&);

    void navi_status_callback(const std_msgs::String::ConstPtr&);

    QString execute_shell_cmd(QString);

    QList<QProcess *> ros_process;
    QProcess* launchManualCtrlProcess;
    QProcess* launchLiosamProcess;
    QProcess* launchChatgptProcess;
    QProcess* launchMakeRouteProcess;
    QProcess* launchNaviProcess;
    QProcess* launchNaviLoadRouteProcess;
    QProcess* launchNaviVoiceCtrlEnvProcess;
    QProcess* launchNaviCameraProcess;

    // QVector<QProcess *> proc_amr;
    QString map_3d;
    QVector<QString> pose_para;
    QString RootPath;

    int process_num =16;
    int person_state = 0;

    QString navi_route_name;

    struct Robot_Pose
    {
        double pos_x;
        double pos_y;
        double pos_z;
        double ori_x;
        double ori_y;
        double ori_z;
        double ori_w;
    };
    struct Waypoint_Data
    {
        QString waypoint_name;
        Robot_Pose pose;
    };
    Waypoint_Data init_waypoint;

    struct Navi_Waypoint_info
    {
        QString waypoint_name;
        int wait_time;
        int delt_angle;
    };
    QList<Navi_Waypoint_info> navi_route_list;

    struct StartStop_Flag
    {
        bool is_joy_used;
        bool is_making_map;
        bool is_making_route;
        bool is_recording_route;
        bool is_navi_Startup;
        bool is_navi_running;
        bool is_essay_playing;
        bool is_obst_playing;
    };
    StartStop_Flag startStop_flag;

    struct Navi_Route_Status
    {
        QString sub_route;         //A-B, From A, To B,A->B
        QString robot_state;       //stop, running
        QString current_location;  //A or B, or AB
        QString route_finished;    //true or false
    };
    Navi_Route_Status navi_route_status;

    Robot_Pose robot_cur_pose;
    Robot_Pose robot_start_pose;

    QList<QPushButton*> obj_buttons_list;
    QList<QString> route_list_record;
    QVector<double> last_point;
    double route_distance;

private slots:
    void on_pushButton_Manual_Control_clicked();
    void on_pushButton_Map_Startup_clicked();
    void on_pushButton_Route_Startup_clicked();
    void on_pushButton_Route_Record_clicked();

    void on_pushButton_Navi_StartUp_clicked();

    void on_pushButton_Waypoint_Add_clicked();

    void on_pushButton_Navi_Run_clicked();

    void on_pushButton_Navi_Save_Script_clicked();

    void on_pushButton_Navi_Load_Script_clicked();

    void on_pushButton_Select_Folder_clicked();

    void on_pushButton_Navi_Script_Clear_clicked();

    void on_comboBox_Sub_MapFolder_currentTextChanged(const QString &arg1);

    void on_pushButton_Next_Target_clicked();

    void on_pushButton_Upload_Location_clicked();

    void on_pushButton_Select_Essay_clicked();

    void on_comboBox_Location_Essay_currentTextChanged(const QString &arg1);

    void on_pushButton_Save_Location_Essay_clicked();

    void on_radioButton_Waypoint_Final_clicked();

    void on_radioButton_Waypoint_First_clicked();

    void on_radioButton_Waypoint_Middle_clicked();

private:
    void writeSettings();
    void readSettings();
    void initStartupLocation();
    void initWaypoints();
    bool getStartupCordinate();

private:
    Ui::MainWindow *ui;
    QMediaPlayer *player_bgm;
    QMediaPlayer *greet_player;
    QMediaPlayer *obst_player;
    QString user_map_path;
    QString last_location;
    QMap<QString, QString> location_essay;
    const uint16_t OBSTACLE_LIM_NUM = 10;
};
#endif // MAINWINDOW_H
