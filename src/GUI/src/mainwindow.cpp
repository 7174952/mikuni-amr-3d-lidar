#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
    , nh_()
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    execute_shell_cmd("bash ~/catkin_ws/src/amr_ros/scripts/amr_start.sh");
    // execute_shell_cmd("python3 ~/catkin_ws/src/amr_ros/scripts/set_voice_device.py");

    //restore last configuration
    readSettings();

    sub_odom_        = nh_.subscribe("/odom", 1, &MainWindow::Odometry_CallBack, this);
    pub_voice_mode   = nh_.advertise<std_msgs::String>("/voice_set_mode",10);
    sub_navi_status  = nh_.subscribe("/navi_status",1, &MainWindow::navi_status_callback, this);
    client_request_next_target = nh_.serviceClient<std_srvs::Empty>("req_next_target");
    sub_obstacle_detect = nh_.subscribe("/obstacle_points_num",1, &MainWindow::obstacle_CallBack,this);

    //Set 3D data file name
    RootPath = QDir::homePath();
    map_3d = "/catkin_ws/src/LIO-SAM-MID360/LOAM/GlobalMap.pcd";

    pose_para.resize(7);

    //find sub map folder
    if(!ui->lineEdit_Folder_Path->text().isEmpty())
    {
        //Search all maps sub folder
        QDir dir(ui->lineEdit_Folder_Path->text());
        QString currText = ui->comboBox_Sub_MapFolder->currentText();

        if(dir.exists())
        {
            ui->comboBox_Sub_MapFolder->clear();
            QFileInfoList list = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);
            for(const QFileInfo &subDirInfo : list)
            {
                if(subDirInfo.isDir())
                {
                    ui->comboBox_Sub_MapFolder->addItem(subDirInfo.fileName());
                }
            }
        }
        ui->comboBox_Sub_MapFolder->setCurrentText(currText);

    }
    //Init waypoints from waypoints_info.txt
    initWaypoints();
    initStartupLocation();

    user_map_path = ui->lineEdit_Folder_Path->text() + "/" + ui->comboBox_Sub_MapFolder->currentText();

    startStop_flag = {false,false,false,false,false,false,false,false};
    init_waypoint = {"origin", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    navi_route_status = {"","stop","","true"};

    //registered ros process
    launchManualCtrlProcess = new QProcess(this);
    launchLiosamProcess = new QProcess(this);
    launchChatgptProcess = new QProcess(this);
    launchNaviProcess = new QProcess(this);
    launchMakeRouteProcess = new QProcess(this);
    launchNaviLoadRouteProcess = new QProcess(this);
    launchNaviVoiceCtrlEnvProcess = new QProcess(this);
    launchNaviCameraProcess = new QProcess(this);

    ros_process.push_back(launchManualCtrlProcess);
    ros_process.push_back(launchLiosamProcess);
    ros_process.push_back(launchChatgptProcess);
    ros_process.push_back(launchMakeRouteProcess);
    ros_process.push_back(launchNaviProcess);
    ros_process.push_back(launchNaviLoadRouteProcess);
    ros_process.push_back(launchNaviVoiceCtrlEnvProcess);
    ros_process.push_back(launchNaviCameraProcess);

    ui->pushButton_Manual_Control->setEnabled(true);
    ui->pushButton_Map_Startup->setEnabled(true);
    ui->pushButton_Route_Startup->setEnabled(true);
    ui->pushButton_Navi_StartUp->setEnabled(true);
    ui->pushButton_Route_Record->setEnabled(false);
    ui->pushButton_Waypoint_Add->setEnabled(false);
    ui->pushButton_Navi_Load_Script->setEnabled(false);
    ui->pushButton_Navi_Save_Script->setEnabled(false);
    ui->pushButton_Navi_Script_Clear->setEnabled(false);
    ui->pushButton_Navi_Run->setEnabled(false);
    ui->pushButton_Next_Target->setEnabled(false);
    if(ui->radioButton_Waypoint_First->isChecked())
    {
        ui->spinBox_Waypoint_Wait->setValue(0);
        ui->spinBox_Waypoint_Wait->setEnabled(false);
        ui->spinBox_Waypoint_Angle->setValue(0);
        ui->spinBox_Waypoint_Angle->setEnabled(false);
    }

    //Regist navi object
    robot_cur_pose = {0,0,0,0,0,0,1};
    route_distance = 0.0;

    last_point.resize(7);
    last_point.fill(0.0);

    //Play music
    player_bgm = new QMediaPlayer(this);
    // 创建 QMediaPlaylist 播放列表对象
    QMediaPlaylist *playlist = new QMediaPlaylist(this);

    playlist->addMedia(QUrl::fromLocalFile(RootPath + "/catkin_ws/src/amr_ros/resource/robot_navi_bgm.mp3"));
    // 设置播放模式为循环播放
    playlist->setPlaybackMode(QMediaPlaylist::Loop);
    // 将播放列表设置为播放器的播放源
    player_bgm->setPlaylist(playlist);
    player_bgm->setVolume(15);

    greet_player = new QMediaPlayer(this);
    obst_player = new QMediaPlayer(this);

}

MainWindow::~MainWindow()
{
    delete ui;
}

//CloseEvent
void MainWindow::closeEvent(QCloseEvent *event)
{
    writeSettings();

    //kill self GUI process
    ros::shutdown();

    //terminate all ros node
    if(ros_process.size() > 0)
    {
        for(QProcess *proc : ros_process)
        {
            terminate_process(proc);
            qDebug() << "Closed ROS process!";
        }
    }

    execute_shell_cmd("pkill -f rosmaster");

    event->accept();
}

void MainWindow::writeSettings()
{
     QSettings settings("mikuni", "GuideRobot");

     //Save configuration
     settings.setValue("windowPosition", pos());
     settings.setValue("MainMapFolder", ui->lineEdit_Folder_Path->text());
     settings.setValue("userMapName", ui->comboBox_Sub_MapFolder->currentText());
     settings.setValue("StartupLocation", ui->comboBox_Startup_Location->currentText());
     settings.setValue("WithCamera", ui->checkBox_With_Camera->isChecked());
     settings.setValue("GuideEnable", ui->checkBox_Guide_Enable->isChecked());
     settings.setValue("WithMic", ui->checkBox_With_Mic->isChecked());
     settings.setValue("ControlEnable", ui->checkBox_Voice_Control_En->isChecked());
     settings.setValue("userLanguage", ui->comboBox_language->currentText());

}

void MainWindow::readSettings()
{
    QSettings settings("mikuni", "GuideRobot");

    //Save configuration
    QPoint pos = settings.value("windowPosition", QPoint(200,200)).toPoint();
    move(pos);

    QString folderName =  settings.value("MainMapFolder", "").toString();
    ui->lineEdit_Folder_Path->setText(folderName);
    ui->comboBox_Sub_MapFolder->setCurrentText(settings.value("userMapName", "").toString());
    last_location = settings.value("StartupLocation", "").toString();
    ui->checkBox_With_Camera->setChecked(settings.value("WithCamera",false).toBool());
    ui->checkBox_Guide_Enable->setChecked(settings.value("GuideEnable",false).toBool());
    ui->checkBox_With_Mic->setChecked(settings.value("WithMic",false).toBool());
    ui->checkBox_Voice_Control_En->setChecked(settings.value("ControlEnable",false).toBool());

    for(const QString str : user_language.keys())
        ui->comboBox_language->addItem(str);
    ui->comboBox_language->setCurrentText(settings.value("userLanguage","English").toString());

}
void MainWindow::initWaypoints()
{
    QString fileName = ui->lineEdit_Folder_Path->text() + "/" + ui->comboBox_Sub_MapFolder->currentText() + "/waypoints_info.txt";
    QFile file_in(fileName);

    ui->comboBox_Startup_Location->clear();
    ui->comboBox_Route_From->clear();
    ui->comboBox_Route_To->clear();
    ui->comboBox_waypoint_list->clear();

    QString location_name;
    if(file_in.exists())
    {
        //find waypoint from file.
        if(file_in.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream data_in(&file_in);

            while(!data_in.atEnd())
            {
                location_name = data_in.readLine().trimmed().split(":").at(0);
                if(!location_name.isEmpty())
                {
                    ui->comboBox_Startup_Location->addItem(location_name);
                    ui->comboBox_Route_From->addItem(location_name);
                    ui->comboBox_Route_To->addItem(location_name);
                    ui->comboBox_waypoint_list->addItem(location_name);
                }
            }
            file_in.close();
        }
        return;
    }

    location_name = "origin";
    if(ui->comboBox_Startup_Location->findText(location_name) < 0)
    {
        ui->comboBox_Startup_Location->addItem(location_name);
    }
    if(ui->comboBox_Route_From->findText(location_name) < 0)
    {
        ui->comboBox_Route_From->addItem(location_name);
    }
    if(ui->comboBox_Route_To->findText(location_name) < 0)
    {
        ui->comboBox_Route_To->addItem(location_name);
    }
    if(ui->comboBox_waypoint_list->findText(location_name) < 0)
    {
        ui->comboBox_waypoint_list->addItem(location_name);
    }
}

void MainWindow::initStartupLocation()
{

    if(ui->comboBox_Startup_Location->findText(last_location) < 0)
    {
        ui->comboBox_Startup_Location->setCurrentIndex(0);
    }
    else
    {
        ui->comboBox_Startup_Location->setCurrentText(last_location);
    }

}

void MainWindow::obstacle_CallBack(const std_msgs::Int32::ConstPtr& msg)
{
    if((msg->data > OBSTACLE_LIM_NUM) && (navi_route_status.robot_state == "running"))
    {
        if(startStop_flag.is_obst_playing == false)
        {
            if(ui->checkBox_With_Mic->isChecked() == false || ui->checkBox_Voice_Control_En->isChecked() == false)
            {
                player_bgm->stop();
            }
            //greet for finished
            QMediaPlaylist *playlist = new QMediaPlaylist(this);
            playlist->addMedia(QUrl::fromLocalFile(RootPath + "/catkin_ws/src/amr_ros/resource/obstacle_alert_" + user_language[ui->comboBox_language->currentText()] + ".mp3"));
            // 设置播放模式为循环播放
            playlist->setPlaybackMode(QMediaPlaylist::Loop);
            // 将播放列表设置为播放器的播放源
            obst_player->setPlaylist(playlist);
            obst_player->play();
            startStop_flag.is_obst_playing = true;
        }

    }
    else
    {
        if(startStop_flag.is_obst_playing == true)
        {
            obst_player->stop();
        }
        startStop_flag.is_obst_playing = false;

        //replay bgm
        if(    (startStop_flag.is_navi_Startup == true)
            && (startStop_flag.is_navi_running == true)
            && (navi_route_status.robot_state == "running"))
        {
            if(ui->checkBox_With_Mic->isChecked() == false || ui->checkBox_Voice_Control_En->isChecked() == false)
            {
                player_bgm->play();
            }
        }

    }
}

void MainWindow::navi_status_callback(const std_msgs::String::ConstPtr& msg)
{
    QStringList status_list = QString::fromStdString(msg->data).split(";");
    navi_route_status.sub_route = status_list.at(0);        // A-B-C
    navi_route_status.robot_state = status_list.at(1);      // start/stop/running
    navi_route_status.current_location = status_list.at(2); // A/B/C/A-B-C
    navi_route_status.route_finished = status_list.at(3);   // "false", "true"

    std_msgs::String voice_mode;
    if(navi_route_status.robot_state == "start")
    {
        if(ui->checkBox_With_Mic->isChecked() && ui->checkBox_Voice_Control_En->isChecked())
        {
            voice_mode.data = "control";
        }
        else
        {
            voice_mode.data = "off";
            player_bgm->play();
        }
        pub_voice_mode.publish(voice_mode);
        ui->pushButton_Next_Target->setEnabled(false);
        startStop_flag.is_essay_playing = false;
    }
    else if(navi_route_status.robot_state == "stop")
    {
        player_bgm->stop();

        if(startStop_flag.is_essay_playing == false && location_essay.contains(navi_route_status.current_location))
        {
            startStop_flag.is_essay_playing = true;
            //essay for target location arrived
            //greet for finished
            greet_player->setMedia(QUrl::fromLocalFile(location_essay.value(navi_route_status.current_location) + "/"
                                                     + location_essay.value(navi_route_status.current_location).split("/").last() + "_"
                                                     + user_language[ui->comboBox_language->currentText()] + ".mp3"));
            greet_player->play();
            // 连接信号到槽函数
            connect(greet_player, &QMediaPlayer::mediaStatusChanged, this, [=](QMediaPlayer::MediaStatus status)
            {
                if (status == QMediaPlayer::EndOfMedia)
                {
                    std_msgs::String voice_mode;
                    voice_mode.data = "chat";
                    pub_voice_mode.publish(voice_mode);
                }
            });
        }
        else
        {
            voice_mode.data = "chat";
            pub_voice_mode.publish(voice_mode);
        }

        if((ui->checkBox_Route_Auto_Next->isChecked() == false) && (navi_route_status.route_finished == "false"))
        {
            ui->pushButton_Next_Target->setEnabled(true);
        }

        //Auto completed all process
        if(navi_route_status.route_finished == "true")
        {
            terminate_process(launchNaviLoadRouteProcess);
            startStop_flag.is_navi_running = false;
            ui->pushButton_Navi_Run->setText("RUN");

            ui->pushButton_Waypoint_Add->setEnabled(true);
            ui->pushButton_Navi_Load_Script->setEnabled(true);
            ui->pushButton_Navi_Save_Script->setEnabled(true);
            ui->pushButton_Navi_Script_Clear->setEnabled(true);
            ui->checkBox_Route_Auto_Next->setEnabled(true);

        }
    }
    else //state=="running"
    {
        //Do nothing
    }

}

void MainWindow::Odometry_CallBack(const nav_msgs::Odometry& odom)
{
    //update current position and orient
    robot_cur_pose.pos_x = odom.pose.pose.position.x;
    robot_cur_pose.pos_y = odom.pose.pose.position.y;
    robot_cur_pose.pos_z = odom.pose.pose.position.z;
    robot_cur_pose.ori_x = odom.pose.pose.orientation.x;
    robot_cur_pose.ori_y = odom.pose.pose.orientation.y;
    robot_cur_pose.ori_z = odom.pose.pose.orientation.z;
    robot_cur_pose.ori_w = odom.pose.pose.orientation.w;

    //save current pose to route file
    if(startStop_flag.is_recording_route == true)
    {
        double distance = std::sqrt(  std::pow(last_point[0] - robot_cur_pose.pos_x, 2)
                                    + std::pow(last_point[1] - robot_cur_pose.pos_y, 2)
                                    + std::pow(last_point[2] - robot_cur_pose.pos_z, 2));
        const double waypoint_interval = 0.25;

        if (distance >= waypoint_interval)
        {
            last_point[0] = robot_cur_pose.pos_x;
            last_point[1] = robot_cur_pose.pos_y;
            last_point[2] = robot_cur_pose.pos_z;
            last_point[3] = robot_cur_pose.ori_x;
            last_point[4] = robot_cur_pose.ori_y;
            last_point[5] = robot_cur_pose.ori_z;
            last_point[6] = robot_cur_pose.ori_w;

            QString record_tmp;

            for(uint i = 0; i < last_point.size(); i++)
            {
                record_tmp.append(QString::number(last_point.at(i)));
                if(i < last_point.size() - 1)
                {
                    record_tmp.append(",");
                }
            }
            //save record to buffer
            route_list_record.push_back(record_tmp);
            route_distance += distance;
        }
    }
}

QString MainWindow::execute_shell_cmd(QString Cmd)
{
    QProcess proc;
    proc.start("bash", QStringList() << "-c" << Cmd);
    if(!proc.waitForFinished(10000))
    {
        return "error";
    }
    // proc.waitForFinished();
    QString Result = proc.readAllStandardOutput();
    return Result;
}

void MainWindow::start_process(QProcess* rosProcess, QString cmd)
{
    rosProcess->start(cmd);
    rosProcess->waitForStarted();
}

void MainWindow::terminate_process(QProcess* rosProcess)
{
    if(rosProcess && (rosProcess->state() != QProcess::NotRunning))
    {
        rosProcess->terminate();
        if(!rosProcess->waitForFinished(3000))
        {
            rosProcess->kill();
        }
    }
}

//JoyStick
void MainWindow::on_pushButton_Manual_Control_clicked()
{
    if(startStop_flag.is_joy_used == false)
    {
        ui->pushButton_Map_Startup->setEnabled(false);
        ui->pushButton_Route_Startup->setEnabled(false);
        ui->pushButton_Navi_StartUp->setEnabled(false);
        //startup cart and joy
        start_process(launchManualCtrlProcess, "roslaunch amr_ros om_manual.launch");
        ui->pushButton_Manual_Control->setText("Stop Manual");
        startStop_flag.is_joy_used = true;
    }
    else
    {
        //end of cart and joy
        terminate_process(launchManualCtrlProcess);
        ui->pushButton_Manual_Control->setText("Manual Control");
        startStop_flag.is_joy_used = false;

        ui->pushButton_Map_Startup->setEnabled(true);
        ui->pushButton_Route_Startup->setEnabled(true);
        ui->pushButton_Navi_StartUp->setEnabled(true);
    }
}

//LIO-SAM
void MainWindow::on_pushButton_Map_Startup_clicked()
{
    QString buf_string = user_map_path + "/" + ui->comboBox_Sub_MapFolder->currentText() + ".yaml";
    ui->pushButton_Map_Startup->setEnabled(false);

    if(startStop_flag.is_making_map == false)
    {
        if(ui->comboBox_Sub_MapFolder->currentText() == "")
        {
            QMessageBox::warning(this,"Warning","Map Name is blank.Write the Map Name.");
            ui->label_MapMaker->setText("[Info] Map Name is blank.Write the Map Name.");
            ui->pushButton_Map_Startup->setEnabled(true);
            return;
        }
        else if(QFile::exists(buf_string))
        {
            QMessageBox::StandardButton reply = QMessageBox::question(
                        this,
                        "Make Sure",
                        "Same Map exist! Do You want to remake new map again?",
                        QMessageBox::Yes | QMessageBox::No);
            if(reply == QMessageBox::Yes)
            {
                QDir dir(user_map_path);
                if(dir.removeRecursively())
                {
                    mkdir(user_map_path.toStdString().c_str(), 0777);
                    initWaypoints();
                }
                else //delete map failed
                {
                    ui->pushButton_Map_Startup->setEnabled(true);
                    return;
                }
            }
            else
            {
                ui->pushButton_Map_Startup->setEnabled(true);
                return;
            }
        }
        else //not exist,to create map folder
        {
            mkdir(user_map_path.toStdString().c_str(), 0777);
        }

        ui->pushButton_Map_Startup->setText("Finish and Save Map");
        ui->label_MapMaker->setText("[Info] System is generating new map.");
        //startup joy and cart
        if(launchManualCtrlProcess->state() == QProcess::NotRunning)
        {
            start_process(launchManualCtrlProcess, "roslaunch amr_ros om_manual.launch");
        }
        //start to generate 2D/3D map
        if(launchLiosamProcess->state() == QProcess::NotRunning)
        {
            start_process(launchLiosamProcess, "roslaunch amr_ros om_map_liosam.launch");
        }

        startStop_flag.is_making_map = true;

        ui->pushButton_Manual_Control->setEnabled(false);
        ui->pushButton_Route_Startup->setEnabled(false);
        ui->pushButton_Navi_StartUp->setEnabled(false);
    }
    else //finish and save map
    {
        //save 2d map
        ui->label_MapMaker->setText("[Info] Save 2D map.");

        QString saveMap_cmd = "rosrun map_server map_saver -f  " + user_map_path + "/" + ui->comboBox_Sub_MapFolder->currentText();
        if(execute_shell_cmd(saveMap_cmd) == "error")
        {
            QMessageBox::warning(this,"Warning","Map data not exist!");
        }

        ui->label_MapMaker->setText("[Info] Shutdown mapping Nodes.");
        //Shutdown mapping process
        terminate_process(launchManualCtrlProcess);
        terminate_process(launchLiosamProcess);

        //save 3D pcd map
        ui->label_MapMaker->setText("[Info] Save 3d map.");
        ros::Time last_time = ros::Time::now();
        bool is_pcd_exist = false;
        while((ros::Time::now().toSec() - last_time.toSec()) < 5.0)
        {
            if(QFile::exists(RootPath + map_3d))
            {
                execute_shell_cmd("mv " + RootPath + map_3d + " " + user_map_path + "/");
                is_pcd_exist = true;
                break;
            }

        }

        if(is_pcd_exist == false)
        {
            QMessageBox::warning(this,"Warning","pcd file not exist!.");

        }
        ui->label_MapMaker->setText("[Info] Making Map Finished!");
        startStop_flag.is_making_map = false;

        execute_shell_cmd("rosnode kill /map_saver");
        execute_shell_cmd("pkill livox_ros_drive");
        ui->pushButton_Map_Startup->setText("Start to Make Map");

        ui->pushButton_Manual_Control->setEnabled(true);
        ui->pushButton_Route_Startup->setEnabled(true);
        ui->pushButton_Navi_StartUp->setEnabled(true);
    }

    ui->pushButton_Map_Startup->setEnabled(true);

}

bool MainWindow::getStartupCordinate()
{
    QString file_name = user_map_path + "/waypoints_info.txt";

    QFile file_in(file_name);
    if(file_in.exists())
    {
        if(file_in.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream data_in(&file_in);
            while(!data_in.atEnd())
            {
                QString line = data_in.readLine().trimmed();
                QString location_name = line.split(":").at(0);
                if(location_name == ui->comboBox_Startup_Location->currentText())
                {
                    QStringList val_list = line.split(":").at(1).split(",");
                    init_waypoint.waypoint_name = location_name;
                    init_waypoint.pose.pos_x = val_list.at(0).toDouble();
                    init_waypoint.pose.pos_y = val_list.at(1).toDouble();
                    init_waypoint.pose.pos_z = val_list.at(2).toDouble();
                    init_waypoint.pose.ori_x = val_list.at(3).toDouble();
                    init_waypoint.pose.ori_y = val_list.at(4).toDouble();
                    init_waypoint.pose.ori_z = val_list.at(5).toDouble();
                    init_waypoint.pose.ori_w = val_list.at(6).toDouble();
                    break;
                }
            }
            file_in.close();
            return true;
        }
    }
    return false;
}

//Make Route
void MainWindow::on_pushButton_Route_Startup_clicked()
{
    //startup make route process
    if(startStop_flag.is_making_route == false)
    {
        QString file_name = user_map_path + "/" + ui->comboBox_Sub_MapFolder->currentText() + ".yaml";

        //check 2D map
        if(!QFile::exists(file_name))
        {
            QMessageBox::warning(this,"Warning","Map File is not exist.Check the Map folder.");
            return;
        }

        ui->pushButton_Route_Startup->setEnabled(false);
        if(getStartupCordinate() == false)
        {
            init_waypoint = {"origin", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
            //to do :write to file
            file_name = user_map_path + "/" + "waypoints_info.txt";
            QFile file_out(file_name);
            if(file_out.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                QTextStream data_out(&file_out);
                data_out << init_waypoint.waypoint_name << ":"
                         << init_waypoint.pose.pos_x << ","
                         << init_waypoint.pose.pos_y << ","
                         << init_waypoint.pose.pos_z << ","
                         << init_waypoint.pose.ori_x << ","
                         << init_waypoint.pose.ori_y << ","
                         << init_waypoint.pose.ori_z << ","
                         << init_waypoint.pose.ori_w << "\n";
                file_out.close();

                ui->comboBox_Startup_Location->addItem(init_waypoint.waypoint_name);
                ui->comboBox_Route_From->addItem(init_waypoint.waypoint_name);
                ui->comboBox_Route_To->addItem(init_waypoint.waypoint_name);
                ui->comboBox_waypoint_list->addItem(init_waypoint.waypoint_name);
            }
            else
            {
                QMessageBox::warning(this,"Warning","Make waypoints_info.txt failed!");
                ui->pushButton_Route_Startup->setEnabled(true);
                return;
            }

        }

        ui->pushButton_Route_Startup->setText("Stop Route Maker");
        ui->label_RouteMaker->setText("[Info] Make route application started!");
        //startup joy to run by manual
        if(launchManualCtrlProcess->state() == QProcess::NotRunning)
        {
            start_process(launchManualCtrlProcess, "roslaunch amr_ros om_manual.launch");
        }

        start_process(launchMakeRouteProcess, "roslaunch amr_ros om_navi_make_route_gui.launch static_map:=" + ui->comboBox_Sub_MapFolder->currentText()
                                            + " p_x:=" + QString::number(init_waypoint.pose.pos_x)
                                            + " p_y:=" + QString::number(init_waypoint.pose.pos_y)
                                            + " p_z:=" + QString::number(init_waypoint.pose.pos_z)
                                            + " o_x:=" + QString::number(init_waypoint.pose.ori_x)
                                            + " o_y:=" + QString::number(init_waypoint.pose.ori_y)
                                            + " o_z:=" + QString::number(init_waypoint.pose.ori_z)
                                            + " o_w:=" + QString::number(init_waypoint.pose.ori_w));

        //Create folder /base_route, /route
        QString path_name = user_map_path + "/" + "base_route";
        QDir dir;

        if(!dir.exists(path_name))
        {
            mkdir(path_name.toStdString().c_str(), 0777);
        }

        path_name = user_map_path + "/" + "navi_route";
        if (!dir.exists(path_name))
        {
            mkdir(path_name.toStdString().c_str(), 0777);
        }

        startStop_flag.is_making_route = true;

        ui->pushButton_Manual_Control->setEnabled(false);
        ui->pushButton_Map_Startup->setEnabled(false);
        ui->pushButton_Navi_StartUp->setEnabled(false);
        ui->pushButton_Route_Record->setEnabled(true);

    }
    else //stop make route
    {
        ui->pushButton_Route_Startup->setText("Startup Route Maker");
        ui->label_RouteMaker->setText("[Info] Make route application finished!");

        terminate_process(launchManualCtrlProcess);
        terminate_process(launchMakeRouteProcess);
        startStop_flag.is_making_route = false;

        ui->pushButton_Manual_Control->setEnabled(true);
        ui->pushButton_Navi_StartUp->setEnabled(true);
        ui->pushButton_Map_Startup->setEnabled(true);
        ui->pushButton_Route_Record->setEnabled(false);

    }

    ui->pushButton_Route_Startup->setEnabled(true);

}

void MainWindow::on_pushButton_Route_Record_clicked()
{
    if(startStop_flag.is_recording_route == false)
    {
        //Check if FROM not equal TO.
        if(ui->comboBox_Route_From->currentText() == ui->comboBox_Route_To->currentText())
        {
            QMessageBox::warning(this,"Warnig","Cannot make route from [" + ui->comboBox_Route_From->currentText()
                                        + "] to [" + ui->comboBox_Route_To->currentText() + "]");
            return;
        }

        //check base route if exist
        QString file_name = user_map_path + "/base_route/" + "base_" + ui->comboBox_Route_From->currentText() + "_" + ui->comboBox_Route_To->currentText() + ".txt";
        if(QFile::exists(file_name))
        {
            QMessageBox::StandardButton reply = QMessageBox::question(
                        this,
                        "Make Sure",
                        "Same Route exist! Do You want to make new route again?",
                        QMessageBox::Yes | QMessageBox::No);
            if(reply == QMessageBox::No)
            {
                return;
            }
        }

        route_list_record.clear();
        route_distance = 0.0;
        startStop_flag.is_recording_route = true;

        ui->pushButton_Route_Record->setText("Finish Record");
    }
    else //startStop_flag.is_recording_route == true
    {
        startStop_flag.is_recording_route = false;

        //update waypoints_info.txt
        QString file_name = user_map_path + "/" + "waypoints_info.txt";
        QFile file_inout(file_name);
        if(!file_inout.open(QIODevice::ReadWrite | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warnig","Update waypoints_info.txt failed!");
            return;
        }

        bool is_FROM_exist = false;
        bool is_TO_exist = false;
        QTextStream data_inout(&file_inout);
        while(!data_inout.atEnd())
        {
            QString line = data_inout.readLine().trimmed().split(":").at(0);
            if(line == ui->comboBox_Route_From->currentText())
            {
                is_FROM_exist = true;
            }
            if(line == ui->comboBox_Route_To->currentText())
            {
                is_TO_exist = true;
            }
        }
        //add new waypoint
        if(is_FROM_exist == false)
        {
            data_inout << ui->comboBox_Route_From->currentText() << ":" << route_list_record.first() << "\n";
            ui->comboBox_Startup_Location->addItem(ui->comboBox_Route_From->currentText());
            ui->comboBox_Route_From->addItem(ui->comboBox_Route_From->currentText());
            ui->comboBox_Route_To->addItem(ui->comboBox_Route_From->currentText());
            ui->comboBox_waypoint_list->addItem(ui->comboBox_Route_From->currentText());
        }
        if(is_TO_exist == false)
        {
            data_inout << ui->comboBox_Route_To->currentText() << ":" << route_list_record.last() << "\n";
            ui->comboBox_Startup_Location->addItem(ui->comboBox_Route_To->currentText());
            ui->comboBox_Route_From->addItem(ui->comboBox_Route_To->currentText());
            ui->comboBox_Route_To->addItem(ui->comboBox_Route_To->currentText());
            ui->comboBox_waypoint_list->addItem(ui->comboBox_Route_To->currentText());
        }
        file_inout.close();

        //update base_route_info.txt
        file_name = user_map_path + "/" + "base_route_info.txt";
        QString route_name1 = ui->comboBox_Route_From->currentText() + "->" + ui->comboBox_Route_To->currentText();
        QString route_name2 = ui->comboBox_Route_To->currentText() + "->" + ui->comboBox_Route_From->currentText();

        QFile file_route(file_name);
        if(file_route.exists())
        {
            //check route, if not exist then addin
            if(!file_route.open(QIODevice::ReadWrite | QIODevice::Text))
            {
                QMessageBox::warning(this,"Warnig","Update base_route_info.txt failed!");
                return;
            }
            QTextStream data_inout(&file_route);
            bool is_route_exist = false;
            while(!data_inout.atEnd())
            {
                QString line = data_inout.readLine().trimmed().split(":").at(0);
                if((line == route_name1) || (line == route_name2))
                {
                    is_route_exist = true;
                }
            }
            //add new route
            if(is_route_exist == false)
            {
                data_inout << route_name1 << ":" << (uint)route_distance << "\n";
                data_inout << route_name2 << ":" << (uint)route_distance << "\n";
            }
            file_route.close();
        }
        else //create file
        {
            if(!file_route.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                QMessageBox::warning(this,"Warnig","Create base_route_info.txt failed!");
                return;
            }
            QTextStream data_out(&file_route);
            data_out << route_name1 << ":" << (uint)route_distance << "\n";
            data_out << route_name2 << ":" << (uint)route_distance << "\n";
            file_route.close();
        }

        //Save route record: From => To (A=>B)
        QString route_filename = user_map_path
                                + "/base_route/"
                                + "base_" + ui->comboBox_Route_From->currentText() + "_" + ui->comboBox_Route_To->currentText() + ".txt";

        QFile route_file(route_filename);
        if(!route_file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warnig","Save " + route_filename + "failed!");
            return;
        }
        //at last, save From waypoint pose to route file
        QTextStream data_out(&route_file);
        for(const QString& str_tmp : route_list_record)
        {
            data_out << str_tmp << "\n";
        }
        route_file.close();

        //Save reversed route record: To => From (B=>A), orient * 180deg
        QString file_name_reverse = user_map_path
                                    + "/base_route/"
                                    + "base_" + ui->comboBox_Route_To->currentText() + "_" + ui->comboBox_Route_From->currentText() + ".txt";
        QFile file_reverse(file_name_reverse);
        if(!file_reverse.open(QIODevice::WriteOnly | QIODevice::Text))
        {
             QMessageBox::warning(this,"Warnig","Save route file: " + file_name_reverse + "failed!");
             return;
        }
        QTextStream record_out(&file_reverse);
        while(!route_list_record.isEmpty())
        {
            QStringList val_list = route_list_record.takeLast().split(",");

            tf::Quaternion quat_A(val_list.at(3).toDouble(), //orig_x
                                         val_list.at(4).toDouble(), //orig_y
                                         val_list.at(5).toDouble(), //orig_z
                                         val_list.at(6).toDouble()  //orig_w
                                        );
            tf::Quaternion quat_delt;
            quat_delt.setRPY(0, 0, M_PI);
            tf::Quaternion quat_B = quat_A * quat_delt;

            record_out << val_list.at(0) << ","
                       << val_list.at(1) << ","
                       << val_list.at(2) << ","
                       << QString::number(quat_B.getX())  << ","
                       << QString::number(quat_B.getY())  << ","
                       << QString::number(quat_B.getZ())  << ","
                       << QString::number(quat_B.getW())  << "\n";
        }
        file_reverse.close();

        ui->pushButton_Route_Record->setText("Start Record");
    }
}

//Pursuite Navigation Continuous
void MainWindow::on_pushButton_Navi_StartUp_clicked()
{
    //check 2D map
    QString file_name = user_map_path + "/" + ui->comboBox_Sub_MapFolder->currentText() + ".yaml";
    if(!QFile::exists(file_name))
    {
        QMessageBox::warning(this,"Warning","Map File is not exist.");
        return;
    }

    ui->pushButton_Navi_StartUp->setEnabled(false);
    //startup navigation node
    if(startStop_flag.is_navi_Startup == false)
    {
        //find robot current location
        if(getStartupCordinate() == false)
        {
            QMessageBox::warning(this,"Warning", "waypoints_info.txt file is not exist!");
            ui->pushButton_Navi_StartUp->setEnabled(true);
            return;
        }

        //startup navigation nodes with initial position and orientation
        if(launchNaviProcess->state() == QProcess::NotRunning)
        {
            QString guide_en = "false";
            if(ui->checkBox_With_Camera->isChecked() && ui->checkBox_Guide_Enable->isChecked())
            {
                guide_en = "true";
            }

            QString voice_control_en = "false";
            if(ui->checkBox_With_Mic->isChecked() && ui->checkBox_Voice_Control_En->isChecked())
            {
                voice_control_en = "true";
            }

            start_process(launchNaviProcess, "roslaunch amr_ros om_navi_fixed_route_gui.launch static_map:="
                    + ui->comboBox_Sub_MapFolder->currentText()
                    + " camera_guide_en:=" + guide_en
                    + " voice_control_en:=" + voice_control_en
                    + " obstacle_lim_num:=" + OBSTACLE_LIM_NUM
                    + " p_x:=" + QString::number(init_waypoint.pose.pos_x)
                    + " p_y:=" + QString::number(init_waypoint.pose.pos_y)
                    + " p_z:=" + QString::number(init_waypoint.pose.pos_z)
                    + " o_x:=" + QString::number(init_waypoint.pose.ori_x)
                    + " o_y:=" + QString::number(init_waypoint.pose.ori_y)
                    + " o_z:=" + QString::number(init_waypoint.pose.ori_z)
                    + " o_w:=" + QString::number(init_waypoint.pose.ori_w));
        }
        //startup camera module
        if(ui->checkBox_With_Camera->isChecked())
        {
            if(launchNaviCameraProcess->state() == QProcess::NotRunning)
            {
                start_process(launchNaviCameraProcess, "roslaunch amr_ros om_navi_depth_camera.launch");
            }
        }
        //startup voice control module
        if(ui->checkBox_With_Mic->isChecked())
        {
#if 0 //debug_ryu
            if(launchNaviVoiceCtrlEnvProcess->state() == QProcess::NotRunning)
            {
                QString command = QString("bash -c \"source ~/myenv3.9/bin/activate && roslaunch amr_ros om_navi_voice_control.launch voice_lang:=%1\"")
                                          .arg(user_language[ui->comboBox_language->currentText()]);

                start_process(launchNaviVoiceCtrlEnvProcess, command);
            }
#endif
        }
        ui->pushButton_Navi_StartUp->setText("Finish Navigation");

        startStop_flag.is_navi_Startup = true;

        ui->pushButton_Manual_Control->setEnabled(false);
        ui->pushButton_Map_Startup->setEnabled(false);
        ui->pushButton_Route_Startup->setEnabled(false);
        ui->pushButton_Navi_Load_Script->setEnabled(true);
        ui->pushButton_Navi_Save_Script->setEnabled(true);
        ui->pushButton_Navi_Script_Clear->setEnabled(true);
        ui->pushButton_Waypoint_Add->setEnabled(true);
        ui->pushButton_Navi_Run->setEnabled(true);

        //load essay_voice_setup.txt
        location_essay.clear();
        QString fileName = user_map_path + "/essay_voice_setup.txt";
        QFile file(fileName);
        if(file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream data_essay_in(&file);
            while(!data_essay_in.atEnd())
            {
                QStringList line_in = data_essay_in.readLine().trimmed().split(":");
                location_essay[line_in.at(0)] = line_in.at(1);
            }
            file.close();
        }
        else
        {
            QMessageBox::warning(this,"Warning","Open essay_voice_setup.txt failed! But continue to run!");
        }

        //greet for startup
        greet_player->setMedia(QUrl::fromLocalFile(RootPath + "/catkin_ws/src/amr_ros/resource/greet_startup_navi_" + user_language[ui->comboBox_language->currentText()] + ".mp3"));
        greet_player->play();

    }
    else
    {
        player_bgm->stop();
        obst_player->stop();
        //greet for finished
        greet_player->setMedia(QUrl::fromLocalFile(RootPath + "/catkin_ws/src/amr_ros/resource/greet_finish_navi_"+user_language[ui->comboBox_language->currentText()]+".mp3"));
        greet_player->play();


        //terminate camera process
        if(ui->checkBox_With_Camera->isChecked())
        {
            terminate_process(launchNaviCameraProcess);
        }
        //terminate voice/mic process
        if(ui->checkBox_With_Mic->isChecked())
        {
#if 0 //debug_ryu
            terminate_process(launchNaviVoiceCtrlEnvProcess);
#endif
        }
        terminate_process(launchNaviProcess);

        ui->pushButton_Navi_StartUp->setText("Startup Navigation");
        startStop_flag.is_navi_Startup = false;

        ui->pushButton_Manual_Control->setEnabled(true);
        ui->pushButton_Map_Startup->setEnabled(true);
        ui->pushButton_Route_Startup->setEnabled(true);
        ui->pushButton_Navi_Load_Script->setEnabled(false);
        ui->pushButton_Navi_Save_Script->setEnabled(false);
        ui->pushButton_Navi_Script_Clear->setEnabled(false);
        ui->pushButton_Waypoint_Add->setEnabled(false);
        ui->pushButton_Navi_Run->setEnabled(false);

    }

    ui->pushButton_Navi_StartUp->setEnabled(true);

}

void MainWindow::on_pushButton_Waypoint_Add_clicked()
{
    QString script_str = ui->lineEdit_Navi_Script->text();

    if(ui->comboBox_waypoint_list->currentText() != "")
    {
        if(ui->radioButton_Waypoint_First->isChecked()
            || ui->radioButton_Waypoint_Middle->isChecked()
            || (ui->radioButton_Waypoint_Final->isChecked() && script_str.isEmpty()))
        {
            script_str.append(ui->comboBox_waypoint_list->currentText() + "->");
        }
        else
        {
            script_str.append(ui->comboBox_waypoint_list->currentText());
        }
        ui->lineEdit_Navi_Script->setText(script_str);

        //save selected waypoint infomation
        Navi_Waypoint_info info;
        info.waypoint_name = ui->comboBox_waypoint_list->currentText();
        info.delt_angle = ui->spinBox_Waypoint_Angle->value();
        info.wait_time = ui->spinBox_Waypoint_Wait->value();
        navi_route_list.push_back(info);
    }
    else
    {
        QMessageBox::warning(this,"Warning","Not select waypoint!");
        return;
    }
}


void MainWindow::on_pushButton_Navi_Run_clicked()
{
    if(startStop_flag.is_navi_running == false)
    {
        //Startup load waypoints nodes
        if(launchNaviLoadRouteProcess->state() == QProcess::NotRunning)
        {
            start_process(launchNaviLoadRouteProcess, "roslaunch amr_ros om_navi_load_waypoints.launch route_script_name:=" + navi_route_name);
        }
        startStop_flag.is_navi_running = true;
        ui->pushButton_Navi_Run->setText("Push To Stop");

        ui->pushButton_Waypoint_Add->setEnabled(false);
        ui->pushButton_Navi_Load_Script->setEnabled(false);
        ui->pushButton_Navi_Save_Script->setEnabled(false);
        ui->pushButton_Navi_Script_Clear->setEnabled(false);
        ui->checkBox_Route_Auto_Next->setEnabled(false);

    }
    else
    {
        terminate_process(launchNaviLoadRouteProcess);
        startStop_flag.is_navi_running = false;
        ui->pushButton_Navi_Run->setText("RUN");

        ui->pushButton_Waypoint_Add->setEnabled(true);
        ui->pushButton_Navi_Load_Script->setEnabled(true);
        ui->pushButton_Navi_Save_Script->setEnabled(true);
        ui->pushButton_Navi_Script_Clear->setEnabled(true);
        ui->checkBox_Route_Auto_Next->setEnabled(true);
        obst_player->stop();
        player_bgm->stop();
    }

}


void MainWindow::on_pushButton_Navi_Save_Script_clicked()
{
    if(navi_route_list.isEmpty())
    {
        QMessageBox::warning(this,"Warning","No waypoint was selected!");
        return;
    }
    if(navi_route_list.size() == 1)
    {
        QMessageBox::warning(this,"Warning","Only one waypoint. Not include final waypoint!");
        return;
    }
    if((navi_route_list.size() == 2) && (navi_route_list.at(0).waypoint_name == navi_route_list.at(1).waypoint_name))
    {
        QMessageBox::warning(this,"Warning","Start waypoint is same to Final waypoint!");
        return;
    }

    QString script_default_name = user_map_path + "/navi_route" + "/navi_script_";
    for(const Navi_Waypoint_info wp: navi_route_list)
    {
        script_default_name.append(wp.waypoint_name + "_");
    }
    if(ui->checkBox_Route_Auto_Next->isChecked())
    {
        script_default_name.append("auto.txt");
    }
    else
    {
        script_default_name.append("manual.txt");
    }

    QString fileName = QFileDialog::getSaveFileName(this,"Save file", script_default_name, "Text(*.txt);;All(*.*)");

    if(fileName.isEmpty()) //cancel
    {
        return;
    }
    //make route file: navi_script_<start>_<final>.txt
    QFile file_navi_script(fileName);
    if(!file_navi_script.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,"Warning","Create Navi route file failed!");
        return;
    }
    QTextStream navi_data_out(&file_navi_script);
    //save selected waypoint to file
    if(ui->checkBox_Route_Auto_Next->isChecked())
    {
        navi_data_out << "AutoNextTarget:true\n";
    }
    else
    {
        navi_data_out << "AutoNextTarget:false\n";
    }
    for(uint i = 0; i < navi_route_list.size(); i++)
    {
        navi_data_out << navi_route_list.at(i).waypoint_name << ":"
                      << navi_route_list.at(i).wait_time     << ","
                      << navi_route_list.at(i).delt_angle    << "\n";
    }
    file_navi_script.close();
    navi_route_list.clear();

    navi_route_name = fileName;
}


void MainWindow::on_pushButton_Navi_Load_Script_clicked()
{
    QString script_default_path = user_map_path + "/navi_route/";

    QString fileName = QFileDialog::getOpenFileName(this, "Open File", script_default_path, "Text(*.txt);;All (*.*)");
    if(fileName.isEmpty()) //cancel
    {
        return;
    }

    QFile file_script(fileName);

    if(!file_script.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,"Warning","Open file: " + fileName + "failed!");
        return;
    }
    navi_route_list.clear();

    QTextStream line_in(&file_script);
    if(!line_in.atEnd())
    {
        QString is_auto_next_target = line_in.readLine().split(":").at(1);
        if(is_auto_next_target.toLower().contains("true"))
        {
            ui->checkBox_Route_Auto_Next->setChecked(true);
        }
        else
        {
            ui->checkBox_Route_Auto_Next->setChecked(false);
        }
    }
    while(!line_in.atEnd())
    {
        QStringList str_list = line_in.readLine().split(QRegularExpression("[:, ]+"));
        Navi_Waypoint_info info_tmp = {str_list.at(0), str_list.at(1).toInt(), str_list.at(2).toInt()};
        navi_route_list.push_back(info_tmp);
    }

    //show on panel
    QString route_str;
    for(uint i = 0; i < navi_route_list.size() - 1; i++)
    {
        route_str.append(navi_route_list.at(i).waypoint_name + "->");
    }
    route_str.append(navi_route_list.back().waypoint_name);
    ui->lineEdit_Navi_Script->setText(route_str);
    navi_route_name = fileName;

}


void MainWindow::on_pushButton_Select_Folder_clicked()
{
    QString work_folder_path = QFileDialog::getExistingDirectory(
                this,
                "Select Folder",
                QDir::homePath(),
                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
            );

    if(!work_folder_path.isEmpty())
    {
        ui->lineEdit_Folder_Path->setText(work_folder_path);
        ui->comboBox_Sub_MapFolder->clear();
        //Search all maps sub folder
        QDir dir(work_folder_path);
        QFileInfoList list = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);
        for(const QFileInfo &subDirInfo : list)
        {
            if(subDirInfo.isDir())
            {
                ui->comboBox_Sub_MapFolder->addItem(subDirInfo.fileName());
            }
        }

    }

    user_map_path = ui->lineEdit_Folder_Path->text() + "/" + ui->comboBox_Sub_MapFolder->currentText();

}

void MainWindow::on_pushButton_Navi_Script_Clear_clicked()
{
    navi_route_list.clear();
    ui->lineEdit_Navi_Script->setText("");
    navi_route_name = "";
}


void MainWindow::on_comboBox_Sub_MapFolder_currentTextChanged(const QString &arg1)
{
    initWaypoints();
    user_map_path = ui->lineEdit_Folder_Path->text() + "/" + ui->comboBox_Sub_MapFolder->currentText();

}

void MainWindow::on_pushButton_Next_Target_clicked()
{
    // request:pure_pursuit to reset path
    std_srvs::Empty srv;
    if(client_request_next_target.call(srv))
    {
        ROS_INFO("load_waypoint: Reset path succeeded.");
        qDebug() << "Request to next target is successful.";
    }
    else
    {
        QMessageBox::warning(this,"Warning","Request to next target is failed!");
    }

}

void MainWindow::on_pushButton_Upload_Location_clicked()
{
    location_essay.clear();
    //load locations (waypoints)
    QString fileName = user_map_path + "/waypoints_info.txt";
    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,"Warning","Open file: " + fileName + "failed!");
        return;
    }
    QTextStream data_in(&file);
    while(!data_in.atEnd())
    {
        QString line = data_in.readLine().trimmed().split(":").at(0);
        ui->comboBox_Location_Essay->addItem(line);
        location_essay.insert(line, "");
    }
    file.close();

    //load essay_voice_setup.txt
    fileName = user_map_path + "/essay_voice_setup.txt";
    file.setFileName(fileName);
    if(!file.exists())
        return;
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream data_essay_in(&file);
    while(!data_essay_in.atEnd())
    {
        QStringList line_in = data_essay_in.readLine().trimmed().split(":");
        if(location_essay.contains(line_in.at(0)))
        {
            location_essay[line_in.at(0)] = line_in.at(1);
            ui->comboBox_Location_Essay->setItemData(ui->comboBox_Location_Essay->findText(line_in.at(0)) ,QBrush(Qt::green),Qt::ForegroundRole);
        }
    }
    file.close();

}


void MainWindow::on_pushButton_Select_Essay_clicked()
{
    QString default_path = RootPath + "/catkin_ws/src/amr_ros/resource/";
    QString introduce_audio_path = QFileDialog::getExistingDirectory(
                this,
                "Select Folder",
                default_path,
                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
            );


    if(introduce_audio_path.isEmpty()) //cancel
    {
        return;
    }
    ui->lineEdit_Essay_Path->setText(introduce_audio_path);
    if(ui->comboBox_Location_Essay->currentText().isEmpty())
    {
        QMessageBox::warning(this,"Warning","Please Select location first!");
        return;
    }
    location_essay[ui->comboBox_Location_Essay->currentText()] = introduce_audio_path;
    ui->comboBox_Location_Essay->setItemData(ui->comboBox_Location_Essay->currentIndex() ,QBrush(Qt::green),Qt::ForegroundRole);

}


void MainWindow::on_comboBox_Location_Essay_currentTextChanged(const QString &arg1)
{
    ui->lineEdit_Essay_Path->clear();
    ui->lineEdit_Essay_Path->setText(location_essay[ui->comboBox_Location_Essay->currentText()]);
}


void MainWindow::on_pushButton_Save_Location_Essay_clicked()
{
    QMessageBox::StandardButton reply = QMessageBox::question(
                this,
                "Make Sure",
                "Do you want to update essay to locations?",
                QMessageBox::Yes | QMessageBox::No);
    if(reply == QMessageBox::No)
    {
        return;
    }

    QString fileName = user_map_path + "/essay_voice_setup.txt";
    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,"Warning","Update essay voice for locations failed!");
        return;
    }

    QTextStream data_out(&file);
    for(auto it = location_essay.cbegin(); it != location_essay.cend(); ++it)
    {
        data_out << it.key() << ":" << it.value() << "\n";
    }
    file.close();

}


void MainWindow::on_radioButton_Waypoint_Final_clicked()
{
    ui->spinBox_Waypoint_Wait->setValue(-1);
    ui->spinBox_Waypoint_Wait->setEnabled(false);
    ui->spinBox_Waypoint_Angle->setEnabled(true);
}


void MainWindow::on_radioButton_Waypoint_First_clicked()
{
    ui->spinBox_Waypoint_Wait->setValue(0);
    ui->spinBox_Waypoint_Wait->setEnabled(false);
    ui->spinBox_Waypoint_Angle->setValue(0);
    ui->spinBox_Waypoint_Angle->setEnabled(false);
}


void MainWindow::on_radioButton_Waypoint_Middle_clicked()
{
    ui->spinBox_Waypoint_Wait->setValue(0);
    ui->spinBox_Waypoint_Wait->setEnabled(true);
    ui->spinBox_Waypoint_Angle->setEnabled(true);
}


void MainWindow::on_pushButton_Init_Robot_Pose_clicked()
{
    init_waypoint = {"origin", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    getStartupCordinate();

    QProcess* init_robot_pose_process = new QProcess(this);

    start_process(init_robot_pose_process, QString("roslaunch amr_ros om_init_robot_pose.launch ")
            + " init_pos_x:=" + QString::number(init_waypoint.pose.pos_x)
            + " init_pos_y:=" + QString::number(init_waypoint.pose.pos_y)
            + " init_pos_z:=" + QString::number(init_waypoint.pose.pos_z)
            + " init_ori_x:=" + QString::number(init_waypoint.pose.ori_x)
            + " init_ori_y:=" + QString::number(init_waypoint.pose.ori_y)
            + " init_ori_z:=" + QString::number(init_waypoint.pose.ori_z)
            + " init_ori_w:=" + QString::number(init_waypoint.pose.ori_w));

    init_robot_pose_process->waitForFinished(10000);


}

