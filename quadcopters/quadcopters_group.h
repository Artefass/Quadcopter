#ifndef QUADCOPTERS_GROUP_H
#define QUADCOPTERS_GROUP_H

#include <functional>
#include <QVector>
#include <Eigen/Dense>
#include <QColor>
#include <QVector>

#include <iostream>

#include "../qcustomplot/qcustomplot.h"
#include "quadcopter_pid.h"

using namespace Eigen;

struct QuadcoptersGroup {

    QVector<double> DesiredX;
    QVector<double> DesiredY;
    QVector<double> DesiredZ;

    QVector<double> DesiredRoll;
    QVector<double> DesiredPitch;
    QVector<double> DesiredYaw;

    QVector<double> SavedTimeSteps;

    QuadcopterPID quadcopterPID;

    QuadcoptersGroup(){}

    void StartSimulation(
            double dt,
            double timeEnd,
            std::function<std::array<double, 3> (double)> GenDesiredPosition,
            std::function<std::array<double, 3> (double)> GenDesiredAttitude,
            std::array<double, 3> startPosition,
            std::array<double, 3> startAttitude)
    {
        DesiredX.clear();
        DesiredY.clear();
        DesiredZ.clear();

        DesiredRoll.clear();
        DesiredPitch.clear();
        DesiredYaw.clear();

        SavedTimeSteps.clear();

        quadcopterPID.Reset();
        quadcopterPID.InitTimeStepSimulation(dt);
        quadcopterPID.InitPosition(startPosition[0], startPosition[1], startPosition[2]);
        quadcopterPID.InitAttitude(startAttitude[0], startAttitude[1], startAttitude[2]);

        double currentTime = 0;
        while (currentTime < timeEnd) {
            currentTime += dt;

            SavedTimeSteps.push_back(currentTime);

            std::array<double, 3> desiredPosition = GenDesiredPosition(currentTime);
            std::array<double, 3> desiredAttitude = GenDesiredAttitude(currentTime);

            DesiredX.push_back(desiredPosition[0]);
            DesiredY.push_back(desiredPosition[1]);
            DesiredZ.push_back(desiredPosition[2]);

            DesiredRoll.push_back(desiredAttitude[0]);
            DesiredPitch.push_back(desiredAttitude[1]);
            DesiredYaw.push_back(desiredAttitude[2]);

            quadcopterPID.SetDesiredAttitude(desiredAttitude[0], desiredAttitude[1], desiredAttitude[2]);
            quadcopterPID.Step();
        }

    }

    void Plot(
            QCustomPlot *plot,
            QVector<QString> legends,
            QVector<double> x,
            QVector<QVector<double>> y_vectors)
    {
        plot->clearGraphs();

        QVector<QColor> colors = {
            {0, 83, 138, 255},
            {22, 189, 4, 255},
            {255, 0, 0, 255},
            {242, 255, 0, 255},
            {208, 0, 255, 255}
        };

        double minX, maxX;
        double minY, maxY;

        minX = maxX = minY = maxY = 0.0;

        if (x.size() > 0) {
            minX = x[0];
            maxX = x[x.size()-1];
        }

        if (y_vectors.size() > 0 && y_vectors[0].size() > 0) {
            minY = y_vectors[0][0];
            maxY = y_vectors[0][0];
        }

        for (int i = 0; i < y_vectors.size(); i++) {
            for (int j = 0; j < y_vectors[i].size(); j++) {
                if (minY > y_vectors[i][j]) {
                    minY = y_vectors[i][j];
                }
                if (maxY < y_vectors[i][j]) {
                    maxY = y_vectors[i][j];
                }
            }
            plot->addGraph();
            plot->graph(i)->setData(x, y_vectors[i]);
            plot->graph(i)->setPen(colors[i]);
            plot->graph(i)->setName(legends[i]);
        }
        plot->xAxis->setRange(minX, maxX);

        double delta = 0.0;
        if (minY == maxY) {
            delta =  3;
        } else {
            delta = (maxY - minY) / 4;
        }

//        std::cout << "PLot minY maxY = " << minY << "  " << maxY << std::endl;

        plot->yAxis->setRange(minY - delta, maxY + delta);
        plot->replot();
    }

    void PlotRoll(QCustomPlot *plot) {
        QVector<QString> legends = {
            "Desired",
            "PID",
            "PID-v",
            "PID-a"
        };
        QVector<QVector<double>> quadcoptersRolls;
        quadcoptersRolls.push_back(DesiredRoll);
        quadcoptersRolls.push_back(quadcopterPID.blackBox.GetRollInfo());
        quadcoptersRolls.push_back(quadcopterPID.blackBox.GetRollVelocityInfo());
        //quadcoptersRolls.push_back(quadcopterPID.blackBox.GetRollAccelerationInfo());
        Plot(plot, legends, SavedTimeSteps, quadcoptersRolls);
    }

    void PlotPitch(QCustomPlot *plot) {
        QVector<QString> legends = {
            "Desired",
            "PID",
            "PID-v",
            "PID-a"
        };
        QVector<QVector<double>> quadcoptersPitches;
        quadcoptersPitches.push_back(DesiredPitch);
        quadcoptersPitches.push_back(quadcopterPID.blackBox.GetPitchInfo());
        quadcoptersPitches.push_back(quadcopterPID.blackBox.GetPitchVelocityInfo());
        quadcoptersPitches.push_back(quadcopterPID.blackBox.GetPitchAccelerationInfo());
        Plot(plot, legends, SavedTimeSteps, quadcoptersPitches);
    }

    void PlotYaw(QCustomPlot *plot) {
        QVector<QString> legends = {
            "Desired",
            "PID",
            "PID-v",
            "PID-a"
        };
        QVector<QVector<double>> quadcoptersYaws;
        quadcoptersYaws.push_back(DesiredYaw);
        quadcoptersYaws.push_back(quadcopterPID.blackBox.GetYawInfo());
        quadcoptersYaws.push_back(quadcopterPID.blackBox.GetYawVelocityInfo());
        quadcoptersYaws.push_back(quadcopterPID.blackBox.GetYawAccelerationInfo());
        Plot(plot, legends, SavedTimeSteps, quadcoptersYaws);
    }

    void PlotMoror1(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersMotor1s;
        quadcoptersMotor1s.push_back(quadcopterPID.blackBox.GetW1Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersMotor1s);
    }

    void PlotMoror2(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersMotor2s;
        quadcoptersMotor2s.push_back(quadcopterPID.blackBox.GetW2Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersMotor2s);
    }

    void PlotMoror3(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersMotor3s;
        quadcoptersMotor3s.push_back(quadcopterPID.blackBox.GetW3Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersMotor3s);
    }

    void PlotMoror4(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersMotor4s;
        quadcoptersMotor4s.push_back(quadcopterPID.blackBox.GetW4Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersMotor4s);
    }

    void PlotRollControl(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersRollControls;
        quadcoptersRollControls.push_back(quadcopterPID.blackBox.GetU2Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersRollControls);
    }

    void PlotPitchControl(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersPitchControls;
        quadcoptersPitchControls.push_back(quadcopterPID.blackBox.GetU3Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersPitchControls);
    }

    void PlotYawControl(QCustomPlot *plot) {
        QVector<QString> legends = {
            "PID"
        };
        QVector<QVector<double>> quadcoptersYawControls;
        quadcoptersYawControls.push_back(quadcopterPID.blackBox.GetU4Info());
        Plot(plot, legends, SavedTimeSteps, quadcoptersYawControls);
    }
};

#endif // QUADCOPTERS_GROUP_H
