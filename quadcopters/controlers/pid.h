#ifndef PID_H
#define PID_H

#include <QtMath>

const double error_epsilon = 1.0;

class PID
{
    double Kp, Ki, Kd;
    double P, I, D;
    double dt;

    double control;
    double maxControl;
    double sumError;
    double maxSumError;
    double previousError;

public:
    PID(double _dt, double _maxControl, double _maxSumError, double _Kp, double _Ki, double _Kd) {
        dt = _dt;
        maxControl = _maxControl;
        maxSumError = _maxSumError;
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
    }

    void SetKpKiKd(double _Kp, double _Ki, double _Kd) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
    }

    void Setdt(double _dt) {
        dt = _dt;
    }

    double GetKp() {
        return Kp;
    }

    double GetKi() {
        return Ki;
    }

    double GetKd() {
        return Kd;
    }

    void Reset() {
        control = 0.0;
        sumError = 0.0;
        previousError = 0.0;
    }

    double ComputeControl(double error) {
        if (error < error_epsilon) {
            sumError = 0.0;
        }
        sumError += error;
        if (maxSumError != 0.0 && qAbs(sumError) > maxSumError) {
            double sign = 1;
            if (sumError < 0.0) {
                sign = -1;
            }
            sumError = maxSumError * sign;
        }

        P = Kp * error;
        I = Ki * sumError;
        D = Kd * (error - previousError) / dt;

        control = P + I + D;

        if (maxControl != 0.0 && qAbs(control) > maxControl) {
            double sign = 1;
            if (control < 0.0) {
                sign = -1;
            }
            control = maxControl * sign;
        }

        previousError = error;
        return control;
    }
};

#endif // PID_H
