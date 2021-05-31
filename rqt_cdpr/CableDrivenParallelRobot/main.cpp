#include "cabledrivenparallerobot.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CableDrivenParalleRobot w;
    w.show();

    return a.exec();
}
