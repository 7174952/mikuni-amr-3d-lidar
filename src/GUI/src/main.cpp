#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "AMR_GUI");

    QApplication a(argc, argv);
    a.setApplicationName("GuideRobot");
    a.setApplicationDisplayName("mikuni");

    MainWindow w;
    w.setWindowTitle(QString::fromStdString("Guide Robot"));
    w.show();
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
      ros::spinOnce();
      a.processEvents();
      loop_rate.sleep();
    }
}
