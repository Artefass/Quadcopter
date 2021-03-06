#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include <QString>
#include <QVector>
#include <QtMath>
#include <Eigen/Dense>

#include <iostream>

#include "config.h"
#include "blackbox.h"

using namespace Eigen;

class Quadcopter
{
    //const double const_mass;    // масса дрона (кг)
    //const double const_length;  // длина луча дрона (м)
protected:
    double eps = 1e-6;

    QString filename;

    const double Ix = Config::Ix; // моментыы инерции относительно осей x,y,z
    const double Iy = Config::Iy;
    const double Iz = Config::Iz;

    const double Ir = Config::Ir;         // инерция ротора

    const double g = Config::g;  // ускорение свободного падения
    const double k = Config::k;         // фактор тяги движка
    const double b = Config::b;         // фактор трения

    const double kw = Config::kw;       // коэфициент прироста скорости (используется для эмуляции работы движка)

    const double m = Config::dron_mass;
    const double l = Config::dron_length;

    double dt = 0.025;            // defaulte time simulation

    double w_r;           // сумма вращений винтов, учитывая направление вращения w1-w2+w3-w4

    Vector4d u_desired;   // требуемый контроль
    Vector4d u_current;   // текущий контроль

    Vector3d xyz;         // текущая позиция
    Vector3d xyz_desired; // требуемая позиция
    Vector3d xyz_velocity; // вектор скорости по осям xyz


    Vector3d attitude;                 // угловое положение
    Vector3d attitude_desired;         // требуемое угловое положение
    Vector3d pqr;                      // угловые скорости

    Vector4d w2;         // угловые скорости роторов
    Vector4d w;
    Vector4d w2_desired; // требуемые угловые скорости роторов

    Matrix4d w2_to_u; //матрица перехода от скоростей к управляющим сигналам
    Matrix4d u_to_w2; //матрица перехода от управляющих сигналов к скоростям

    double currentTime = 0.0;

    Vector3d attitude_acceleration; // угловое ускорение

    double howerW = Config::hower_w;

    double minW = Config::min_w;
    double maxW = Config::max_w;

public:
    BlackBox blackBox;  // место, куда сохраняются объекты

    Quadcopter();

    void StepRotors();
    void StepModel();
    void SaveState();
    virtual void Reset();

    virtual void ComputeControl() {
        return;
    }

    void Step() {
        currentTime += dt;

        ComputeControl();
        StepRotors();
        StepModel();
        SaveState();
    }

    Vector4d GetDesiredControl() {
        return u_desired;
    }

    Vector4d GetCurrentControl() {
        return u_current;
    }

    Vector3d GetCurrentAttitude() {
        return attitude;
    }

    Vector3d GetDesiredAttitude() {
        return attitude_desired;
    }

    Vector3d GetCurrentPosition() {
        return xyz;
    }

    Vector3d GetDesiredPosition() {
        return xyz_desired;
    }

    double SimpleDerivative(double previous_value, double current_value, double dt) {
        return (current_value - previous_value) / dt;
    }

    void SetDesiredAttitude(double desired_roll, double desired_pitch, double desired_yaw) {
        attitude_desired(0) = desired_roll;
        attitude_desired(1) = desired_pitch;
        attitude_desired(2) = desired_yaw;
    }

    void InitAttitude(double roll, double pitch, double yaw) {
        attitude(0) = roll;
        attitude(1) = pitch;
        attitude(2) = yaw;

        std::cout << "Initialize attitude: " << attitude.transpose() << std::endl;
    }

    void InitPosition(double x, double y, double z) {
        xyz(0) = x;
        xyz(1) = y;
        xyz(2) = z;

        std::cout << "Initialize position: " << xyz.transpose() << std::endl;
    }

    virtual void InitTimeStepSimulation(double timeStep) {
        dt = timeStep;
    }

    void Init(double timeStep, double startX, double startY, double startZ, double startRoll, double startPitch, double startYaw) {
        Reset();
        InitTimeStepSimulation(timeStep);
        InitPosition(startX,startY, startZ);
        InitAttitude(startRoll, startPitch, startYaw);
        SaveState();
    }
};

#endif // QUADCOPTER_H
