#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "quadcopters/quadcopter_pid.h"
#include "quadcopters.h"

#include <QtMath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->pitchPlot->legend->setVisible(true);
    ui->rollPlot->legend->setVisible(true);
    ui->yawPlot->legend->setVisible(true);

    ui->xPlot->legend->setVisible(true);
    ui->yPlot->legend->setVisible(true);
    ui->zPlot->legend->setVisible(true);

    ui->rotor1Plot->legend->setVisible(true);
    ui->rotor2Plot->legend->setVisible(true);
    ui->rotor3Plot->legend->setVisible(true);
    ui->rotor4Plot->legend->setVisible(true);

    ui->controlRollPlot->legend->setVisible(true);
    ui->controlPitchPlot->legend->setVisible(true);
    ui->controlYawPlot->legend->setVisible(true);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//void StartSimulation(double dt, std::array<double, 3> startPosition, ) {
//    Quadcopters.StartSimulation(dt, startPosition, startAttitude);
//}

void MainWindow::on_startSimulationButton_clicked()
{
    double endTime  = ui->endTimeSpinBox->value();
    double timeStep = ui->timeStepSpinBox->value();

    double positionX = ui->positionXLineEdit->text().toDouble();
    double positionY = ui->positionYLineEdit->text().toDouble();
    double positionZ = ui->positionZLineEdit->text().toDouble();

    double roll  = qDegreesToRadians(ui->rollLineEdit->text().toDouble());
    double pitch = qDegreesToRadians(ui->pitchLineEdit->text().toDouble());
    double yaw   = qDegreesToRadians(ui->yawLineEdit->text().toDouble());

    double desiredRoll  = qDegreesToRadians(ui->desiredRollLineEdit->text().toDouble());
    double desiredPitch = qDegreesToRadians(ui->desiredPitchLineEdit->text().toDouble());
    double desiredYaw   = qDegreesToRadians(ui->desiredYawLineEdit->text().toDouble());

    std::function<std::array<double, 3> (double)> GenDesiredPosition;
    std::function<std::array<double, 3> (double)> GenDesiredAttitude;


    GenDesiredPosition = [=](double time) -> std::array<double, 3> {
        (void)time;
        return { 0.0, 0.0, 0.0 };
    };

    GenDesiredAttitude = [=](double time) -> std::array<double, 3> {
        (void)time;
        return { desiredRoll, desiredPitch, desiredYaw };
    };

    Quadcopters.StartSimulation(
                timeStep,
                endTime,
                GenDesiredPosition,
                GenDesiredAttitude,
                std::array<double,3> {
                    positionX,
                    positionY,
                    positionZ
                },
                std::array<double,3> {
                    roll,
                    pitch,
                    yaw,
                });

    Quadcopters.PlotRoll(ui->rollPlot);
    Quadcopters.PlotPitch(ui->pitchPlot);
    Quadcopters.PlotYaw(ui->yawPlot);

    Quadcopters.PlotMoror1(ui->rotor1Plot);
    Quadcopters.PlotMoror2(ui->rotor2Plot);
    Quadcopters.PlotMoror3(ui->rotor3Plot);
    Quadcopters.PlotMoror4(ui->rotor4Plot);

    Quadcopters.PlotRollControl(ui->controlRollPlot);
    Quadcopters.PlotPitchControl(ui->controlPitchPlot);
    Quadcopters.PlotYawControl(ui->controlYawPlot);
}

void Plot();
