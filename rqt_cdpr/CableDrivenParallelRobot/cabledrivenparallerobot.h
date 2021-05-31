#ifndef CABLEDRIVENPARALLEROBOT_H
#define CABLEDRIVENPARALLEROBOT_H

#include <QMainWindow>

namespace Ui {
class CableDrivenParalleRobot;
}

class CableDrivenParalleRobot : public QMainWindow
{
    Q_OBJECT

public:
    explicit CableDrivenParalleRobot(QWidget *parent = 0);
    ~CableDrivenParalleRobot();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_3_clicked(bool checked);

    void on_pushButton_2_clicked();

private:
    Ui::CableDrivenParalleRobot *ui;
};

#endif // CABLEDRIVENPARALLEROBOT_H
