#ifndef QUADCOPTER_PID_H
#define QUADCOPTER_PID_H

#include "quadcopter.h"
#include "controlers/pid.h"

#include <iostream>

class QuadcopterPID: public Quadcopter
{
public:
    PID phiPID;
    PID thetaPID;
    PID psiPID;

    QuadcopterPID() :
        Quadcopter(),
        phiPID(dt, 0.0, 0.0, 0.3, 0.475, 0.11875),
        thetaPID(dt, 0.0, 0.0, 0.3, 0.475, 0.11875),
        psiPID(dt, 0.0, 0.0, 0.3, 0.475, 0.11875)
    {

    }

    void InitTimeStepSimulation(double timeStep) override {
        phiPID.Setdt(timeStep);
        thetaPID.Setdt(timeStep);
        psiPID.Setdt(timeStep);

        Quadcopter::InitTimeStepSimulation(timeStep);
    }

    void ComputeControl() override {
        u_desired(0) = m * g;
        u_desired(1) = phiPID.ComputeControl(attitude_desired(0) - attitude(0));
//        u_desired(2) = thetaPID.ComputeControl(attitude_desired(1) - attitude(1));
//        u_desired(3) = psiPID.ComputeControl(attitude_desired(2) - attitude(2));

        std::cout << "attitude_desired: " << attitude_desired.transpose() << std::endl;
        std::cout << "attitude: " << attitude.transpose() << std::endl;
        std::cout << "u_desired: " << u_desired.transpose() << std::endl;
    }

    void Reset() override {
        std::cout << "ResetPID" << std::endl;
        phiPID.Reset();
        thetaPID.Reset();
        psiPID.Reset();

        Quadcopter::Reset();
    }
};

#endif // QUADCOPTER_PID_H
