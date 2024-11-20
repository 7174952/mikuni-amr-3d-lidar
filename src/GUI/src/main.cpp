#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "AMR_GUI");

    QApplication a(argc, argv);

    MainWindow w;
    w.setWindowTitle(QString::fromStdString("Guide ROBOT GUI"));
    w.show();
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
      ros::spinOnce();
      a.processEvents();
      loop_rate.sleep();
    }
}
