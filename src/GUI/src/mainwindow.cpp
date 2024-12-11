#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
    , nh_()
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    execute_shell_cmd("bash ~/catkin_ws/src/amr_ros/scripts/amr_start.sh");

    //restore last configuration
    readSettings();

    sub_odom_        = nh_.subscribe("/odom", 1, &MainWindow::Odometry_CallBack, this);
    pub_goal_        = nh_.advertise<std_msgs::Float32>("goal_cmd",10);
    sub_music_ctrl   = nh_.subscribe("/music_cmd",1, &MainWindow::Music_Ctrl_CallBack, this);

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

    startStop_flag = {false,false,false,false,false,false};
    init_waypoint = {"", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


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

    //Regist navi object
    robot_cur_pose = {0,0,0,0,0,0,1};
    route_distance = 0.0;


    last_point.resize(7);
    last_point.fill(0.0);

    //Play music
    player = new QMediaPlayer(this);
    // 创建 QMediaPlaylist 播放列表对象
    QMediaPlaylist *playlist = new QMediaPlaylist(this);
    playlist->addMedia(QUrl::fromLocalFile(RootPath + "/catkin_ws/src/amr_ros/resource/running_back_music.mp3"));
    // 设置播放模式为循环播放
    playlist->setPlaybackMode(QMediaPlaylist::Loop);
    // 将播放列表设置为播放器的播放源
    player->setPlaylist(playlist);
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

void MainWindow::Music_Ctrl_CallBack(const std_msgs::String::ConstPtr& msg)
{
    qDebug() << "gui music command:" << QString::fromStdString(msg->data) << "\n";

    if(msg->data == "play")
    {
        player->play();
    }
    else if(msg->data == "stop")
    {
        player->stop();
    }
    else if(msg->data == "pause")
    {
        player->pause();
    }
    else
    {
        //do nothing
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
                if(location_name.contains(ui->comboBox_Startup_Location->currentText()))
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
            init_waypoint = {"origin", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
                                            + " o_z:=" + QString::number(init_waypoint.pose.pos_z)
                                            + " o_w:=1");

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
        if(ui->comboBox_Route_From->currentText().contains(ui->comboBox_Route_To->currentText()))
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
            start_process(launchNaviProcess, "roslaunch amr_ros om_navi_fixed_route_gui.launch static_map:="
                    + ui->comboBox_Sub_MapFolder->currentText()
                    + " camera_guide_en:=" + guide_en
                    + " p_x:=" + QString::number(init_waypoint.pose.pos_x)
                    + " p_y:=" + QString::number(init_waypoint.pose.pos_y)
                    + " o_z:=" + QString::number(init_waypoint.pose.pos_z)
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
            if(launchNaviVoiceCtrlEnvProcess->state() == QProcess::NotRunning)
            {
                start_process(launchNaviVoiceCtrlEnvProcess, "bash -c \"source ~/myenv3.9/bin/activate && roslaunch amr_ros om_navi_voice_control.launch\"");
            }
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
    }
    else
    {
        //terminate camera process
        if(ui->checkBox_With_Camera->isChecked())
        {
            terminate_process(launchNaviCameraProcess);
        }
        //terminate voice/mic process
        if(ui->checkBox_With_Mic->isChecked())
        {
            terminate_process(launchNaviVoiceCtrlEnvProcess);
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

    //Reverse waypoint and append to route list
    if(ui->checkBox_Go_Back->isChecked())
    {
        QList<Navi_Waypoint_info> navi_route_list_tmp(navi_route_list);
        navi_route_list_tmp.pop_back();
        while(navi_route_list_tmp.empty())
        {
            navi_route_list.push_back(navi_route_list_tmp.takeLast());
        }
    }

    QString script_default_name = user_map_path + "/navi_route/" + "navi_script_"
                        + navi_route_list.first().waypoint_name + "_" + navi_route_list.last().waypoint_name + ".txt";
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
    if(ui->checkBox_Route_Cycle->isChecked())
    {
        navi_data_out << "cycle:true\n";
    }
    else
    {
        navi_data_out << "cycle:false\n";
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
        QString is_cycle = line_in.readLine().split(":").at(1);
        if(is_cycle.toLower().contains("true"))
        {
            ui->checkBox_Route_Cycle->setChecked(true);
        }
        else
        {
            ui->checkBox_Route_Cycle->setChecked(false);
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

