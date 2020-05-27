#ifndef BLACK_BOX_H
#define BLACK_BOX_H

#include <QVector>

class BlackBox {
public:
    void SetTime( double currentTime ){
        time.push_back(currentTime);
    }

    void SetPosition( double x, double y, double z ) {
        coordinates_x.push_back(x);
        coordinates_y.push_back(y);
        coordinates_z.push_back(z);
    }

    void SetAngles( double theta, double phi, double psi ){
        angles_theta.push_back(theta);
        angles_phi.push_back(phi);
        angles_psi.push_back(psi);
    }

    void Reset() {
        coordinates_x.clear();
        coordinates_y.clear();
        coordinates_z.clear();

        angles_theta.clear();
        angles_phi.clear();
        angles_psi.clear();

        time.clear();

        w1.clear();
        w2.clear();
        w3.clear();
        w4.clear();
    }

    void SetW1234( double _w1, double _w2, double _w3, double _w4) {
        w1.push_back(_w1);
        w2.push_back(_w2);
        w3.push_back(_w3);
        w4.push_back(_w4);
    }

    QVector<double> GetCoordinatesXInfo() const {
        return coordinates_x;
    }

    QVector<double> GetCoordinatesYInfo() const {
        return coordinates_y;
    }

    QVector<double> GetCoordinatesZInfo() const {
        return coordinates_z;
    }

    QVector<double>  GetTimesInfo() const {
        return time;
    }

    QVector<double>  GetAnglesThetaInfo() const {
        return angles_theta;
    }

    QVector<double>  GetAnglesPhiInfo() const {
        return angles_phi;
    }

    QVector<double>  GetAnglesPsiInfo() const {
        return angles_psi;
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

private:
    QVector<double>  coordinates_x, coordinates_y, coordinates_z;
    QVector<double>  angles_theta, angles_phi, angles_psi;
    QVector<double>  time;
    QVector<double>  w1, w2, w3, w4;
};

#endif // BLACK_BOX_H
