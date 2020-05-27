#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <QVector>

class BlackBox
{
public:
    BlackBox();

public:
    void SetTime( double currentTime ){
        time.push_back(currentTime);
    }

    void SetPosition( double _x, double _y, double _z ) {
        x.push_back(_x);
        y.push_back(_y);
        z.push_back(_z);
    }

    void SetAngles( double phi, double theta, double psi ){
        roll.push_back(phi);
        pitch.push_back(theta);
        yaw.push_back(psi);
    }

    void Reset() {
        x.clear();
        y.clear();
        z.clear();

        roll.clear();
        pitch.clear();
        yaw.clear();

        time.clear();

        w1.clear();
        w2.clear();
        w3.clear();
        w4.clear();

        u1.clear();
        u2.clear();
        u3.clear();
        u4.clear();

        phi_v.clear();
        theta_v.clear();
        psi_v.clear();

        phi_a.clear();
        theta_a.clear();
        psi_a.clear();
    }

    void SetW1234( double _w1, double _w2, double _w3, double _w4) {
        w1.push_back(_w1);
        w2.push_back(_w2);
        w3.push_back(_w3);
        w4.push_back(_w4);
    }

    void SetU1234( double _u1, double _u2, double _u3, double _u4) {
        u1.push_back(_u1);
        u2.push_back(_u2);
        u3.push_back(_u3);
        u4.push_back(_u4);
    }

    void SetRollPitchYawVelocities(double rollVelocity, double pitchVelocity, double yawVelocity) {
        phi_v.push_back(rollVelocity);
        theta_v.push_back(pitchVelocity);
        psi_v.push_back(yawVelocity);
    }

    void SetRollPitchYawAccelerations(double rollAcceleration, double pitchAcceleration, double yawAcceleration) {
        phi_a.push_back(rollAcceleration);
        theta_a.push_back(pitchAcceleration);
        psi_a.push_back(yawAcceleration);
    }

    QVector<double> GetCoordinatesXInfo() const {
        return x;
    }

    QVector<double> GetCoordinatesYInfo() const {
        return y;
    }

    QVector<double> GetCoordinatesZInfo() const {
        return z;
    }

    QVector<double>  GetTimesInfo() const {
        return time;
    }

    QVector<double>  GetRollInfo() const {
        return roll;
    }

    QVector<double>  GetPitchInfo() const {
        return pitch;
    }

    QVector<double>  GetYawInfo() const {
        return yaw;
    }

    QVector<double>  GetW1Info() const {
        return w1;
    }

    QVector<double>  GetW2Info() const {
        return w2;
    }

    QVector<double>  GetW3Info() const {
        return w3;
    }

    QVector<double>  GetW4Info() const {
        return w4;
    }

    QVector<double> GetU1Info() const {
        return u1;
    }

    QVector<double> GetU2Info() const {
        return u2;
    }

    QVector<double> GetU3Info() const {
        return u3;
    }

    QVector<double> GetU4Info() const {
        return u4;
    }

    QVector<double> GetRollVelocityInfo() const {
        return phi_v;
    }

    QVector<double> GetPitchVelocityInfo() const {
        return theta_v;
    }

    QVector<double> GetYawVelocityInfo() const {
        return psi_v;
    }

    QVector<double> GetRollAccelerationInfo() const {
        return phi_a;
    }

    QVector<double> GetPitchAccelerationInfo() const {
        return theta_a;
    }

    QVector<double> GetYawAccelerationInfo() const {
        return psi_a;
    }

private:
    QVector<double>  x, y, z;
    QVector<double>  roll, pitch, yaw;
    QVector<double>  time;
    QVector<double>  w1, w2, w3, w4;
    QVector<double>  u1, u2, u3, u4;

    QVector<double>  phi_v, theta_v, psi_v;
    QVector<double>  phi_a, theta_a, psi_a;
};

#endif // BLACKBOX_H
