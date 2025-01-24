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
#include <QTimer>

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

class AudioWorker : public QObject
{
    Q_OBJECT
public:
    // 播放模式：单次播放 或 循环播放
    enum PlayMode
    {
        SinglePlay,
        LoopPlay
    };
    Q_ENUM(PlayMode)

    explicit AudioWorker(QObject *parent = nullptr);

public slots:
    // 设置播放模式

    void setPlayMode(bool is_LoopPlay);


    // 设置新的音频文件(本地文件路径)
    void setAudioFile(const QString &filePath);

    // 开始播放
    void startPlaying();

    // 停止播放
    void stopPlaying();

signals:
    /**
     * @brief 当播放真正停止后（包括自然结束或外部请求停止），发出该信号
     */
    void playbackStopped();

private slots:
    // 监听 QMediaPlayer 媒体状态变化
    void onMediaStatusChanged(QMediaPlayer::MediaStatus status);

private:
    QMediaPlayer *m_player;
    PlayMode      m_playMode;

    /**
     * @brief 标志位：是否已收到“停止”的请求（要求本次播放结束后停止）
     */
    bool m_stopRequested;
    bool m_startRequested;
};

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
    ros::Publisher pub_odom_limit;
    ros::Subscriber sub_camera_ctrl_status;


public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void terminate_process(QProcess*);
    void start_process(QProcess*, QString);
    void closeEvent(QCloseEvent*);
    void Odometry_CallBack(const nav_msgs::Odometry&);
    void obstacle_CallBack(const std_msgs::Int32::ConstPtr&);
    void cameraCtrl_CallBack(const std_msgs::String::ConstPtr&);

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

    const QMap<QString, QString> user_language = {
        {"English","en"},
        {"日本語","ja"},
        {"中文","zh"}
    };

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
        bool is_navi_limit_on;
        bool is_navi_next_target_on;
        bool is_camera_ctrl_playing;
    };
    StartStop_Flag startStop_flag;

    struct Navi_Route_Status
    {
        QString sub_route;         //A-B, From A, To B,A->B
        QString robot_state;       //stop, running
        QString current_location;  //A or B, or AB
        QString route_finished;    //true or false
        QString is_auto_pass;      //true or false
        QString is_front_careless; //true or false
        QString is_end_careless;   //true or false

    };
    Navi_Route_Status navi_route_status;

    struct LimitKey
    {
        int id;
        double limit_from;
        double limit_to;
        bool operator<(const LimitKey &other) const
        {
            return id < other.id;
        }
    };
    struct LimitValue
    {
        double max_vel;
        double obst_tolerance;
    };

    struct Base_Route_Navi_Limit
    {
        double total_distance;
        QMap<LimitKey, LimitValue> limit_list;
    };

    Base_Route_Navi_Limit base_route_navi_limit;
    QMap<QString, Base_Route_Navi_Limit> base_route_info;
    LimitKey limit_key;

    struct Odom_Limit_Status
    {
        double odom_distance;
        double total_distance;
        bool limit_on;
        double max_vel;
        double obst_tolerance;

        void clear()
        {
            odom_distance = 0;
            total_distance = 0;
            limit_on = false;
            max_vel = 0;
            obst_tolerance = 0;
        }
    };

    struct Camera_Ctrl_Status
    {
        uint person_num;
        bool run_status;
    };
    Camera_Ctrl_Status camera_ctrl_status;

    Robot_Pose robot_cur_pose;
    Robot_Pose robot_start_pose;

    QList<QPushButton*> obj_buttons_list;
    QList<QString> route_list_record;
    QVector<double> last_point;
    double route_distance;
    double odom_distance;
    bool is_open_door;
    bool is_start_init_pose;

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

    void on_pushButton_Init_Robot_Pose_clicked();

    void on_pushButton_Limit_Vel_clicked();
    // 在 Widget 的头文件中声明一个槽函数
    void onPlaybackStopped();

    void on_comboBox_1_currentTextChanged(const QString &arg1);

    void on_comboBox_2_currentTextChanged(const QString &arg1);

private:
    void writeSettings();
    void readSettings();
    void initStartupLocation();
    void initWaypoints();
    bool getStartupCordinate();
    void audio_cleanup();
    void initRobotCoorinate();


private:
    Ui::MainWindow *ui;
    QThread     *m_thread;
    AudioWorker *m_worker;
    QString audio_name;
    bool is_waiting_camera_play;
    bool is_waiting_obst_play;
    bool start_guide;

    QString user_map_path;
    QString last_location;
    QMap<QString, QString> location_essay;
    const uint16_t OBSTACLE_LIM_NUM = 10;
};
#endif // MAINWINDOW_H
