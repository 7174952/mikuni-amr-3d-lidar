#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
    , nh_()
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    execute_shell_cmd("bash ~/Desktop/amr_start.sh");

    sub_odom_        = nh_.subscribe("/odom", 1, &MainWindow::Odometry_CallBack, this);
    sub_count_       = nh_.subscribe("/count_up", 1, &MainWindow::Count_CallBack, this);
    sub_person_      = nh_.subscribe("/person", 1, &MainWindow::Person_CallBack, this);
    pub_goal_        = nh_.advertise<std_msgs::Float32>("goal_cmd",10);

    //Set 3D data file name
    Data_3D.resize(5);
    RootPath   = "/home/mikuni/";
    Data_3D[0] = "catkin_ws/src/LIO-SAM-MID360/LOAM/CornerMap.pcd";
    Data_3D[1] = "catkin_ws/src/LIO-SAM-MID360/LOAM/GlobalMap.pcd";
    Data_3D[2] = "catkin_ws/src/LIO-SAM-MID360/LOAM/SurfMap.pcd";
    Data_3D[3] = "catkin_ws/src/LIO-SAM-MID360/LOAM/trajectory.pcd";
    Data_3D[4] = "catkin_ws/src/LIO-SAM-MID360/LOAM/transformations.pcd";

    pose_para.resize(7);

    //registered ros process
    launchManualCtrlProcess = new QProcess(this);
    launchLiosamProcess = new QProcess(this);
    launchChatgptProcess = new QProcess(this);
    launchNaviProcess = new QProcess(this);
    launchMakeRouteProcess = new QProcess(this);
    launchNaviLoadRouteProcess = new QProcess(this);
    ros_process.push_back(launchManualCtrlProcess);
    ros_process.push_back(launchLiosamProcess);
    ros_process.push_back(launchChatgptProcess);
    ros_process.push_back(launchMakeRouteProcess);
    ros_process.push_back(launchNaviProcess);
    ros_process.push_back(launchNaviLoadRouteProcess);

    groupBoxLayout_navi = new QVBoxLayout(ui->groupBox_Navi_Waypoints);

    buttonGroup_navi = new QButtonGroup(this);

    //startup chatgpt
    start_process(launchChatgptProcess, "cd ~/catkin_ws/src/amr_ros/scripts && python3 GPT_talker_ros.py");

    //Regist navi object
    ui->pushButton_Navi_StartUp->setDisabled(false);
    ui->lineEdit_MapName_MapMaker->setText(ui->lineEdit_MapName->text());
    ui->lineEdit_MapName_RouteMaker->setText(ui->lineEdit_MapName->text());
    robot_cur_pose = {0,0,0,0,0,0,1};
    route_distance = 0.0;


    last_point.resize(7);
    last_point.fill(0.0);

}

MainWindow::~MainWindow()
{
    //terminate all ros node
    if(ros_process.size() > 0)
    {
        for(QProcess *proc : ros_process)
        {
            terminate_process(proc);
        }
    }

    delete ui;
}

//CloseEvent
void MainWindow::closeEvent(QCloseEvent *event)
{
    //terminate all ros node
    if(ros_process.size() > 0)
    {
        for(QProcess *proc : ros_process)
        {
            terminate_process(proc);
            ROS_INFO("closeEvent!");
        }
    }
}

void MainWindow::Person_CallBack(const std_msgs::String& msg)
{
    if((person_state == 0) && ((msg.data == "RUN") || (msg.data == "STOP")))
    {
        qDebug() << "Check:Perosn";
        person_state = 1;
    }
}

void MainWindow::Odometry_CallBack(const nav_msgs::Odometry& odom)
{
    ui->lineEdit_X_set->setText(QString::number(odom.pose.pose.position.x,'f',3));
    ui->lineEdit_Y_set->setText(QString::number(odom.pose.pose.position.y,'f',3));
    double y_sign = (odom.pose.pose.orientation.z >= 0)? std::sqrt(1 - pow(odom.pose.pose.orientation.w, 2))
                                                       : -std::sqrt(1 - pow(odom.pose.pose.orientation.w, 2));

    ui->lineEdit_theta_set->setText(QString::number(2 * 45 / atan(1) * atan2(y_sign, odom.pose.pose.orientation.w),'f',2));

    //update current position and orient
    robot_cur_pose.pos_x = odom.pose.pose.position.x;
    robot_cur_pose.pos_y = odom.pose.pose.position.y;
    robot_cur_pose.pos_z = odom.pose.pose.position.z;
    robot_cur_pose.ori_x = odom.pose.pose.orientation.x;
    robot_cur_pose.ori_y = odom.pose.pose.orientation.y;
    robot_cur_pose.ori_z = odom.pose.pose.orientation.z;
    robot_cur_pose.ori_w = odom.pose.pose.orientation.w;

    //save current pose to route file
    if(save_route_en == true)
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

void MainWindow::Count_CallBack(const std_msgs::Int32& count_up)
{
#if 0
    if(count_up.data >= 5)
    {
        cmd_exe("vlc ~/Music/doite.wav",proc_amr[12]);
    }
    else if(count_up.data == -10)
    {
        kill_process(12);
    }
#endif
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
void MainWindow::on_pushButton_Manual_Controll_clicked()
{
    if(is_joy_used == false)
    {
        //startup cart and joy
        start_process(launchManualCtrlProcess, "roslaunch amr_ros om_manual.launch");
        ui->pushButton_Manual_Controll->setText("Stop Manual");
        is_joy_used = true;
    }
    else
    {
        terminate_process(launchManualCtrlProcess);
        ui->pushButton_Manual_Controll->setText("Manual Controll");
        is_joy_used = false;
    }
}

void MainWindow::Startup_Joy(bool force_to_start)
{
    if(force_to_start) //startup joy
    {
        start_process(launchManualCtrlProcess, "roslaunch amr_ros om_manual.launch");
        ui->pushButton_Manual_Controll->setText("Stop Manual");
        is_joy_used = true;

    }
    else //stop joy
    {
        terminate_process(launchManualCtrlProcess);
        ui->pushButton_Manual_Controll->setText("Manual Controll");
        is_joy_used = false;
    }
}

//LIO-SAM
void MainWindow::on_pushButton_Map_Startup_clicked()
{
    QString buf_string = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_MapMaker->text() + "/" + ui->lineEdit_MapName_MapMaker->text() + ".yaml";
    switch (map_make_state)
    {
    case MAKE_MAP_START://Start
        if(ui->lineEdit_MapName_MapMaker->text() == "")
        {
            QMessageBox::warning(this,"Warning","Map Name is blank.Write the Map Name.");
            ui->label_MapMaker->setText("[Info] Map Name is blank.Write the Map Name.");
        }
        else if(QFile::exists(buf_string))
        {
            ui->label_MapMaker->setText("[Info] Map File is already exist. Change the name of Map Name.");
            QMessageBox::warning(this,"Warning","Map File is already exist. Change the name of Map Name.");
        }
        else
        {
            ui->pushButton_Map_Startup->setText("Save");
            ui->pushButton_Route_Startup->setDisabled(true);
            ui->label_MapMaker->setText("[Info] 2nd Step:When finish make map,then push the Save button.");
            ui->lineEdit_MapName_MapMaker->setDisabled(true);
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

            map_make_state = MAKE_MAP_SAVE;
        }
        break;
    case MAKE_MAP_SAVE://Save
        ui->pushButton_Map_Startup->setText("Ready");
        ui->label_MapMaker->setText("[Info] 3rd Step:Ready to bring 3D data,push the Ready button.");
        Save_Map();
        map_make_state = MAKE_MAP_READY;
        break;
    case MAKE_MAP_READY://Ready
        if(QFile::exists(buf_string))
        {
            ui->pushButton_Map_Startup->setText("Finish");
            ui->label_MapMaker->setText("[Info] Last Step:Due to close Map Maker,push the Finish button.");
            //Shutdown mapping process
            terminate_process(launchManualCtrlProcess);
            terminate_process(launchLiosamProcess);

            //on_pushButton_Mapping_clicked();
            map_make_state = MAKE_MAP_FINISH;
        }
        else
        {
            ui->pushButton_Map_Startup->setText("Save");
            QMessageBox::warning(this,"Warning","Failed reading Map file.Try Save again.");
            ui->label_MapMaker->setText("[Info] Failed reading Map file.Try Save again.");
            map_make_state = MAKE_MAP_START; //return to generate map again
        }
        break;
    case MAKE_MAP_FINISH://Finish
        if(    QFile::exists(RootPath + Data_3D[0])
            && QFile::exists(RootPath + Data_3D[1])
            && QFile::exists(RootPath + Data_3D[2])
            && QFile::exists(RootPath + Data_3D[3])
            && QFile::exists(RootPath + Data_3D[4]))
        {
            ui->pushButton_Map_Startup->setText("Start");
            ui->pushButton_Route_Startup->setDisabled(false);
            ui->label_MapMaker->setText("[Info] 1st Step:Input Map Name,then push the Start button.");
            ui->lineEdit_MapName_MapMaker->setDisabled(false);
            Copy_3D_Data();

            map_make_state = MAKE_MAP_START;
        }
        else
        {
            ui->label_MapMaker->setText("[Info] Do not bring the 3D data,push the Finish button again.");
            QMessageBox::warning(this,"Warning","Do not bring the 3D data,push the Finish button again.");
        }
        break;
    default:
        break;
    }
}

bool MainWindow::Save_Map()
{
    QDir dir;
    QString buf_string = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_MapMaker->text();
    dir.mkdir(buf_string);

    buf_string = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_MapMaker->text() + "/" + ui->lineEdit_MapName_MapMaker->text() + ".yaml";

    if(ui->lineEdit_MapName_MapMaker->text() != "")
    {
        if(QFile::exists(buf_string))
        {
            QMessageBox::warning(this,"Warning","Map File is already exist. Change the name of MapName.");
        }
        else
        {
            buf_string = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_MapMaker->text() + "/" + ui->lineEdit_MapName_MapMaker->text();
            ui->statusbar->showMessage("Saving MapData as:" + buf_string + ".yaml & .pgm", 10000);
            if(execute_shell_cmd("rosrun map_server map_saver -f  " + buf_string) != "error")
            {
                return true;
            }
            else
            {
                QMessageBox::warning(this,"Warning","No map data exist!");
            }
        }
    }
    else
    {
        QMessageBox::warning(this,"WARNIG","MapName is blank.Please write the MapName");
    }
    return false;
}

void MainWindow::Copy_3D_Data()
{
    QString buf_string = ui->lineEdit_MapName_MapMaker->text();
    execute_shell_cmd("mv ~/catkin_ws/src/LIO-SAM-MID360/LOAM/*pcd ~/catkin_ws/src/amr_ros/maps/" + buf_string + "/.");
}

//Make Route
void MainWindow::on_pushButton_Route_Startup_clicked()
{
    QString file_name;

    //check waypoints file
#if 0
    if(ui->lineEdit_SetStart_RouteMaker->text() != "")
    {
        file_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text() + "/waypoints_info.txt";
        if(!QFile::exists(file_name))
        {
            QMessageBox::warning(this,"Warning","waypoints_info File is not exist. Make the waypoints_info file");
            return;
        }
    }
#endif
    //check 2D map
    file_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text() + "/" + ui->lineEdit_MapName_RouteMaker->text() + ".yaml";
    if(!QFile::exists(file_name))
    {
        QMessageBox::warning(this,"Warning","Map File is not exist.Check the Map Name.");
        return;
    }

    //startup make route process
    if(is_route_maker_running == false)
    {
        ui->lineEdit_MapName_RouteMaker->setDisabled(true);
        ui->lineEdit_MapRoute_RouteMaker->setDisabled(true);
        ui->pushButton_Route_Startup->setText("Close System");
        ui->label_RouteMaker->setText("[Info] Last Step:When finish make route,push the Close System button.");
        //startup joy to run by manual
        if(launchManualCtrlProcess->state() == QProcess::NotRunning)
        {
            start_process(launchManualCtrlProcess, "roslaunch amr_ros om_manual.launch");
        }

        //setup robot's initial postion and pose
#if 0
        if(ui->lineEdit_MapRoute_RouteMaker->text() != "")
        {
            QString file_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text() + "/waypoints_info.txt";

            QFile file_in(file_name);
            if(!file_in.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                //show error
                QMessageBox::warning(this,"Warning","Read *_SetStart.txt Failed.");
                return;
            }

            //get position and origent from file
            QTextStream data_in(&file_in);
            bool is_waypoint_ready = false;
            while(!data_in.atEnd())
            {
                QString line = data_in.readLine();
                if(line.trimmed().startsWith(ui->lineEdit_MapRoute_RouteMaker->text()))
                {
                    QStringList list = line.trimmed().split(QRegularExpression("[:, ]+"));
                    for(uint i = 0; i < list.count() - 1; i++)
                    {
                        pose_para[i] = list.at(i + 1);
                    }
                    is_waypoint_ready = true;
                    break;
                }
            }
            if(!is_waypoint_ready)
            {
                QMessageBox::warning(this,"Warning", ui->lineEdit_MapRoute_RouteMaker->text() + " waypoint is not exist!");
                return;
            }

        }
        else
        {
            //set default pose value
            pose_para.fill("0");
            pose_para[6] = "1";
        }
        start_process(launchMakeRouteProcess, "roslaunch amr_ros om_navi_make_route_gui.launch static_map:=" + ui->lineEdit_MapName_RouteMaker->text()
                                            + " p_x:=" + pose_para[0]
                                            + " p_y:=" + pose_para[1]
                                            + " o_z:=" + pose_para[5]
                                            + " o_w:=" + pose_para[6]);
#endif

        start_process(launchMakeRouteProcess, "roslaunch amr_ros om_navi_make_route_gui.launch static_map:=" + ui->lineEdit_MapName_RouteMaker->text()
                                            + " p_x:=0"
                                            + " p_y:=0"
                                            + " o_z:=0"
                                            + " o_w:=1");
        is_route_maker_running = true;
    }
    else //stop make route
    {
        ui->lineEdit_MapName_RouteMaker->setDisabled(false);
        ui->lineEdit_MapRoute_RouteMaker->setDisabled(false);
        ui->pushButton_Route_Startup->setText("System Start");
        ui->label_RouteMaker->setText("[Info] 1st Step:Input Map Name & Map Route,then push the Start button.");

        terminate_process(launchManualCtrlProcess);
        terminate_process(launchMakeRouteProcess);
        is_route_maker_running = false;
    }

    //Create folder /waypoints, /route
    QString path_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_MapMaker->text() + "/waypoints";
    QDir dir;

    if(!dir.exists(path_name))
    {
        mkdir(path_name.toStdString().c_str(), 0777);
    }

    path_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_MapMaker->text() + "/navi_route";
    if (!dir.exists(path_name))
    {
        mkdir(path_name.toStdString().c_str(), 0777);
    }

    if(is_route_maker_running == true)
    {
        ui->pushButton_Map_Startup->setDisabled(true);
    }
    else
    {
        ui->pushButton_Map_Startup->setDisabled(false);
    }
}

void MainWindow::on_pushButton_Route_SetWaypoint_clicked()
{
    //read waypoints from waypoints_info.txt
    QString file_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text() + "/waypoints_info.txt";
    QFile file_in(file_name);

    if(file_in.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        //find waypoint from file.
        QTextStream data_in(&file_in);
        QString waypoint_list = data_in.readAll();
        file_in.close();
        if(waypoint_list.contains(ui->lineEdit_SetStart_RouteMaker->text()))
        {
            QMessageBox::warning(this,"Warning","Waypoint exist! Set by another waypoint name.");
            return;
        }
    }

    //Save new waypoint to file
    if(!file_in.open(QIODevice::Append | QIODevice::Text))
    {
        QMessageBox::warning(this, "Warning", "Open file " + file_name + " failed!");
        return;
    }
    QTextStream data_out(&file_in);
    data_out << ui->lineEdit_SetStart_RouteMaker->text() << ":"
             << robot_cur_pose.pos_x << ","
             << robot_cur_pose.pos_y << ","
             << robot_cur_pose.pos_z << ","
             << robot_cur_pose.ori_x << ","
             << robot_cur_pose.ori_y << ","
             << robot_cur_pose.ori_z << ","
             << robot_cur_pose.ori_w << "\n";
    file_in.close();
}

void MainWindow::on_pushButton_Route_Record_clicked()
{
    if(is_saving_route == false)
    {
        //Check if Start not equal Goal.
        if(ui->lineEdit_StartID->text().contains(ui->lineEdit_GoalID->text()))
        {
            QMessageBox::warning(this,"Warnig","Error!! -> Start_ID and Goal_ID is same.");
            return;
        }
        //check goalID,startID in waypoints file
        QString file_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text() + "/waypoints_info.txt";
        QFile file_waypoint(file_name);
        if(!file_waypoint.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warnig","Open " + file_name + "failed!");
            return;
        }

        QTextStream line_in(&file_waypoint);
        QString line_tmp;
        bool is_goalID_exist = false;
        bool is_startID_exist = false;
        while(!line_in.atEnd())
        {
            line_tmp = line_in.readLine();
            if(line_tmp.contains(ui->lineEdit_StartID->text()))
            {
                QStringList pose_tmp = line_tmp.split(QRegularExpression("[:, ]+"));
                robot_start_pose.pos_x = pose_tmp.at(1).toDouble();
                robot_start_pose.pos_y = pose_tmp.at(2).toDouble();
                robot_start_pose.pos_z = pose_tmp.at(3).toDouble();
                robot_start_pose.ori_x = pose_tmp.at(4).toDouble();
                robot_start_pose.ori_y = pose_tmp.at(5).toDouble();
                robot_start_pose.ori_z = pose_tmp.at(6).toDouble();
                robot_start_pose.ori_w = pose_tmp.at(7).toDouble();
                is_startID_exist = true;
            }
            if(line_tmp.contains(ui->lineEdit_GoalID->text()))
            {
                QStringList pose_tmp = line_tmp.split(QRegularExpression("[:, ]+"));
                robot_goal_pose.pos_x = pose_tmp.at(1).toDouble();
                robot_goal_pose.pos_y = pose_tmp.at(2).toDouble();
                robot_goal_pose.pos_z = pose_tmp.at(3).toDouble();
                robot_goal_pose.ori_x = pose_tmp.at(4).toDouble();
                robot_goal_pose.ori_y = pose_tmp.at(5).toDouble();
                robot_goal_pose.ori_z = pose_tmp.at(6).toDouble();
                robot_goal_pose.ori_w = pose_tmp.at(7).toDouble();
                is_goalID_exist = true;
            }

            if(is_goalID_exist && is_startID_exist)
            {
                break;
            }
        }
        file_waypoint.close();

        if(is_startID_exist == false)
        {
            QMessageBox::warning(this,"Warnig","Start ID: " + ui->lineEdit_StartID->text() + " not exist!");
            return;
        }
        if(is_goalID_exist == false)
        {
            QMessageBox::warning(this,"Warnig","Goal ID: " + ui->lineEdit_GoalID->text() + " not exist!");
            return;
        }

        route_list_record.clear();
        QString record_tmp =  QString::number(robot_start_pose.pos_x) + ","
                            + QString::number(robot_start_pose.pos_y) + ","
                            + QString::number(robot_start_pose.pos_z) + ","
                            + QString::number(robot_start_pose.ori_x) + ","
                            + QString::number(robot_start_pose.ori_y) + ","
                            + QString::number(robot_start_pose.ori_z) + ","
                            + QString::number(robot_start_pose.ori_w) ;
        route_list_record.push_back(record_tmp);

        route_distance = 0.0;
        save_route_en = true;
        is_saving_route = true;

        ui->pushButton_Route_Record->setText("Finish Record");
    }
    else //is_saving_route == true
    {
        save_route_en = false;
        is_saving_route = false;

        QString record_tmp =  QString::number(robot_goal_pose.pos_x) + ","
                            + QString::number(robot_goal_pose.pos_y) + ","
                            + QString::number(robot_goal_pose.pos_z) + ","
                            + QString::number(robot_goal_pose.ori_x) + ","
                            + QString::number(robot_goal_pose.ori_y) + ","
                            + QString::number(robot_goal_pose.ori_z) + ","
                            + QString::number(robot_goal_pose.ori_w);
        route_list_record.push_back(record_tmp);

        //auto generate route file name
        //Save route record: Start => Goal (A=>B)
        QString route_filename = ui->lineEdit_Directory->text()+ui->lineEdit_MapName_RouteMaker->text()
                                + "/waypoints/"
                                + "base_" + ui->lineEdit_StartID->text() + "_" + ui->lineEdit_GoalID->text() + ".txt";

        QFile route_file(route_filename);
        if(!route_file.open(QIODevice::Append | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warnig","Save " + route_filename + "failed!");
            return;
        }
        //at last, save goal waypoint pose to route file
        QTextStream data_out(&route_file);
        for(const QString& str_tmp : route_list_record)
        {
            data_out << str_tmp << "\n";
        }
        route_file.close();

        //auto generate route file name
        //Save reversed route record: Goal => Start (B=>A)
        QString file_name_reverse = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text()
                                    + "/waypoints/"
                                    + "base_" + ui->lineEdit_GoalID->text() + "_" + ui->lineEdit_StartID->text() + ".txt";
        QFile file_reverse(file_name_reverse);
        if(!file_reverse.open(QIODevice::WriteOnly | QIODevice::Text))
        {
             QMessageBox::warning(this,"Warnig","Save reversed route file: " + file_name_reverse + "failed!");
             return;
        }
        QTextStream record_out(&file_reverse);
        while(!route_list_record.isEmpty())
        {
            record_out << route_list_record.takeLast() << "\n";
        }
        file_reverse.close();

        //Save route info: base_route_info.txt
        QString file_route_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName_RouteMaker->text() + "/base_route_info.txt";
        QFile file_route_info(file_route_name);
        if(!file_route_info.open(QIODevice::Append | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warnig","Open base_route_info.txt failed!");
            return;
        }
        QTextStream str_out(&file_route_info);
        str_out << ui->lineEdit_StartID->text() << "->" << ui->lineEdit_GoalID->text() << ":" << (uint)route_distance << "\n";
        str_out << ui->lineEdit_GoalID->text() << "->" << ui->lineEdit_StartID->text() << ":" << (uint)route_distance << "\n";
        file_route_info.close();

        ui->pushButton_Route_Record->setText("Start Record");
    }

}

//Pursuite Navigation Continuous
void MainWindow::on_pushButton_Navi_StartUp_clicked()
{
    //check 2D map
    QString file_name = ui->lineEdit_Directory->text()+ui->lineEdit_MapName->text()+"/"+ui->lineEdit_MapName->text()+".yaml";
    if(!QFile::exists(file_name))
    {
        QMessageBox::warning(this,"Warning","Map File is not exist.Check the 2D Map Name.");
        return;
    }

    //startup navigation node
    if(is_nav_sys_running == false)
    {
        //Read all waypoints from file
        QString file_waypoint_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName->text() + "/waypoints_info.txt";
        QFile file_waypoint(file_waypoint_name);

        if(!file_waypoint.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warning","Open " + file_waypoint_name + " failed!");
            return;
        }

        QTextStream waypoint_in(&file_waypoint);
        QStringList waypoint_list;
        //load all waypoints
        while(!waypoint_in.atEnd())
        {
            waypoint_list.push_back(waypoint_in.readLine());
        }
        file_waypoint.close();

        if(waypoint_list.isEmpty()) //no navi waypoint
        {
            QMessageBox::warning(this,"Warning", "No waypoints was loaded!");
            return;
        }

        //Delete old button
        while(btn_list.size() > 0)
        {
            QPushButton* btn_tmp = btn_list.takeLast();
            buttonGroup_navi->removeButton(btn_tmp);
            groupBoxLayout_navi->removeWidget(btn_tmp);
            delete btn_tmp;
            btn_tmp = nullptr;
        }

        //create waypoint button and show on panel
        int button_idx = 0;
        //buttonGroup_waypoint->setExclusive(false);
        for(const QString name_str : waypoint_list)
        {
            QPushButton *button = new QPushButton(name_str.split(":").first());
            btn_list.push_back(button);
            groupBoxLayout_navi->addWidget(button);
            buttonGroup_navi->addButton(button, button_idx++);
        }

        QObject::connect(buttonGroup_navi, QOverload<int>::of(&QButtonGroup::buttonClicked), [=](int id)
        {
             ui->lineEdit_Waypoint_Name->setText(btn_list.at(id)->text());
        });

        //find robot current location
        file_name = ui->lineEdit_Directory->text()+ui->lineEdit_MapName->text()+"/waypoints_info.txt";
        QFile file_in(file_name);
        if(!file_in.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warning","Open " + file_name + " failed!");
            return;
        }

        bool is_waypoint_exist = false;
        QTextStream line_in(&file_in);
        while(!line_in.atEnd())
        {
            QString line_temp = line_in.readLine();
            if(line_temp.contains(ui->lineEdit_SetStart->text()))
            {
                is_waypoint_exist = true;
                QStringList str_list = line_temp.split(QRegularExpression("[:, ]+"));
                //load initial pose data
                for(int i = 0; i < str_list.size() - 1; i++)
                {
                    pose_para[i] = str_list.at(i+1);
                }
                break;
            }
        }

        if(is_waypoint_exist == false)
        {
            QMessageBox::warning(this,"Warning", "waypoint: " + ui->lineEdit_SetStart->text() + "is not exist!");
            return;
        }

        //startup navigation nodes with initial position and orientation
        if(launchNaviProcess->state() == QProcess::NotRunning)
        {
            start_process(launchNaviProcess, "roslaunch amr_ros om_navi_fixed_route_gui.launch static_map:="
                    + ui->lineEdit_MapName->text()
                    + " p_x:=" + pose_para[0]
                    + " p_y:=" + pose_para[1]
                    + " o_z:=" + pose_para[5]
                    + " o_w:=" + pose_para[6]);
        }

        ui->pushButton_Navi_StartUp->setText("Stop Navigation");
        ui->pushButton_Map_Startup->setDisabled(true);
        ui->pushButton_Route_Startup->setDisabled(true);
        ui->pushButton_Manual_Controll->setDisabled(true);

        is_nav_sys_running = true;
    }
    else
    {
        terminate_process(launchNaviProcess);

        ui->pushButton_Map_Startup->setDisabled(false);
        ui->pushButton_Route_Startup->setDisabled(false);
        ui->pushButton_Manual_Controll->setDisabled(false);
        ui->pushButton_Navi_StartUp->setText("Startup Navigation");
        is_nav_sys_running = false;
    }

}

void MainWindow::on_lineEdit_MapName_textChanged(const QString &arg1)
{
    ui->lineEdit_MapName_MapMaker->setText(arg1);
    ui->lineEdit_MapName_RouteMaker->setText(arg1);
}


void MainWindow::on_pushButton_Spin_Run_clicked()
{
    std_msgs::Float32 simple_goal;
    simple_goal.data = (ui->lineEdit_theta_set_run->text()).toDouble();
    pub_goal_.publish(simple_goal);

}

void MainWindow::on_pushButton_Waypoint_Add_clicked()
{
    QString script_str = ui->lineEdit_Navi_Script->text();

    if(ui->lineEdit_Waypoint_Name->text() != "")
    {
        if(ui->radioButton_Waypoint_First->isChecked()
            || ui->radioButton_Waypoint_Middle->isChecked()
            || (ui->radioButton_Waypoint_Final->isChecked() && script_str.isEmpty()))
        {
            script_str.append(ui->lineEdit_Waypoint_Name->text() + "->");
        }
        else
        {
            script_str.append(ui->lineEdit_Waypoint_Name->text());
        }
        ui->lineEdit_Navi_Script->setText(script_str);

        //save selected waypoint infomation
        Navi_Waypoint_info info;
        info.waypoint_name = ui->lineEdit_Waypoint_Name->text();
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
    if(is_navi_route_running == false)
    {
        //Startup load waypoints nodes
        if(launchNaviLoadRouteProcess->state() == QProcess::NotRunning)
        {
            start_process(launchNaviLoadRouteProcess, "roslaunch amr_ros om_navi_load_waypoints.launch route_script_name:=" + navi_route_name);
        }
        is_navi_route_running = true;
        ui->pushButton_Navi_Run->setText("Push To Stop");
    }
    else
    {
        terminate_process(launchNaviLoadRouteProcess);
        is_navi_route_running = false;
        ui->pushButton_Navi_Run->setText("RUN");
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

    QString script_default_name = ui->lineEdit_Directory->text() + ui->lineEdit_MapName->text() + "/navi_route/" + "navi_script_"
                        + navi_route_list.first().waypoint_name + "_" + navi_route_list.last().waypoint_name + ".txt";
    QString fileName = QFileDialog::getSaveFileName(this,"Save file", script_default_name, "Text(*.txt);;All(*.*)");

    if(fileName.isEmpty())
    {
        QMessageBox::warning(this,"Warning","Save Failed! Script name is NULL!");
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
    QString script_default_path = ui->lineEdit_Directory->text() + ui->lineEdit_MapName->text() + "/navi_route/";

    QString fileName = QFileDialog::getOpenFileName(this, "Open File", script_default_path, "Text(*.txt);;All (*.*)");
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
