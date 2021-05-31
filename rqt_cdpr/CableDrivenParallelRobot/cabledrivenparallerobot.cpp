#include "cabledrivenparallerobot.h"
#include "ui_cabledrivenparallerobot.h"

CableDrivenParalleRobot::CableDrivenParalleRobot(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CableDrivenParalleRobot)
{
    ui->setupUi(this);

}

CableDrivenParalleRobot::~CableDrivenParalleRobot()
{
    delete ui;
}

void CableDrivenParalleRobot::on_pushButton_2_clicked()
{
//QString uto = ui->lineEdit->text();
  //  ui->label_6->setText(uto);
}
