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

#include <sys/stat.h>

//ROS Set
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
//

//QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
//QT_END_NAMESPACE

class MainWindow : public QWidget
{
    Q_OBJECT
    ros::NodeHandle nh_;
    ros::Publisher pub_goal_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_reach_goal_;
    ros::Subscriber sub_voice_order_;
    ros::Subscriber sub_music_ctrl;

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void terminate_process(QProcess*);
    void start_process(QProcess*, QString);
    void closeEvent(QCloseEvent*);
    void Odometry_CallBack(const nav_msgs::Odometry&);

    void Music_Ctrl_CallBack(const std_msgs::String::ConstPtr&);
    void Start_Close_Mapping();
    bool Save_Map();
    void Copy_3D_Data();

    QString execute_shell_cmd(QString);

    QList<QProcess *> ros_process;
    QProcess* launchManualCtrlProcess;
    QProcess* launchLiosamProcess;
    QProcess* launchChatgptProcess;
    QProcess* launchMakeRouteProcess;
    QProcess* launchNaviProcess;
    QProcess* launchNaviLoadRouteProcess;
    QProcess* launchNaviVoiceCtrlEnvProcess;

    // QVector<QProcess *> proc_amr;
    QString map_3d;
    QVector<QString> pose_para;
    QString RootPath;

    int process_num =16;

    int person_state = 0;

    bool is_joy_used = false;
    bool is_route_maker_running = false;
    bool is_nav_sys_running = false;
    bool save_route_en = false;
    bool is_saving_route = false;
    bool is_navi_route_running = false;

    QString navi_route_name;

    enum Map_State
    {
        MAKE_MAP_START = 0,
        MAKE_MAP_SAVE,  //1
        MAKE_MAP_READY, //2
        MAKE_MAP_FINISH //3
    };
    Map_State map_make_state = MAKE_MAP_START;

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

    struct Navi_Waypoint_info
    {
        QString waypoint_name;
        int wait_time;
        int delt_angle;
    };
    QList<Navi_Waypoint_info> navi_route_list;


    Robot_Pose robot_cur_pose;
    Robot_Pose robot_start_pose;
    Robot_Pose robot_goal_pose;

    QList<QPushButton*> obj_buttons_list;
    QList<QString> route_list_record;
    QVector<double> last_point;
    double route_distance;

    QVBoxLayout *groupBoxLayout_navi;


    QList<QPushButton *> btn_list;
    QButtonGroup *buttonGroup_navi;


private slots:
    void on_pushButton_Manual_Controll_clicked();
    void on_pushButton_Map_Startup_clicked();
    void on_pushButton_Route_Startup_clicked();
    void on_pushButton_Route_SetWaypoint_clicked();
    void on_pushButton_Route_Record_clicked();

    void on_lineEdit_MapName_textChanged(const QString &arg1);

    void on_pushButton_Navi_StartUp_clicked();

    void on_pushButton_Spin_Run_clicked();

    void on_pushButton_Waypoint_Add_clicked();

    void on_pushButton_Navi_Run_clicked();

    void on_pushButton_Navi_Save_Script_clicked();

    void on_pushButton_Navi_Load_Script_clicked();

private:
    Ui::MainWindow *ui;
    void Startup_Joy(bool);
    QMediaPlayer *player;
};
#endif // MAINWINDOW_H
