#include "quadcopter.h"

#include <iostream>

#include <QtMath>

Quadcopter::Quadcopter()
{
    u_desired << m*g, 0, 0, 0;
    u_current = u_desired;

    xyz << 0, 0, 1;
    xyz_desired = xyz;
    xyz_velocity << 0.0, 0.0, 0.0;

    attitude << 0, 0, 0;
    attitude_desired = attitude;

    pqr << 0.0, 0.0, 0.0;

    w_r = 0;

    w2_to_u <<   k,    k,   k,   k,
                0, -l*k,   0, l*k,
             -l*k,    0, l*k,   0,
               -b,    b,  -b,   b;

    u_to_w2 = w2_to_u.inverse();
    w2_desired = u_to_w2 * u_desired;
    w2 = w2_desired;

    // debug information

//    std::cout << "w2_desired: " << w2_desired.transpose() << std::endl;
//    std::cout << "w2        : " << w2.transpose() << std::endl;
//    std::cout << "u_to_w2   : " << u_to_w2 << std::endl;
//    std::cout << "w2_to_u   : " << w2_to_u << std::endl;
//    std::cout << "w hower   : " << howerW << std::endl;
//    std::cout << "w hower^2 : " << howerW * howerW << std::endl;
//    std::cout << "w min     : " << minW << std::endl;
//    std::cout << "w min^2   : " << minW * minW << std::endl;
//    std::cout << "w max     : " << maxW << std::endl;
//    std::cout << "w max^2   : " << maxW * maxW << std::endl;
}

void Quadcopter::StepRotors() {
    //Teppo Luukkonen <<Modelling and control of quadcopter>>

//    w2_desired(0) = u_desired(0) / (4 * k) - u_desired(2) / (2 * k *l) - u_desired(3) / (4 * b);
//    w2_desired(1) = u_desired(0) / (4 * k) - u_desired(1) / (2 * k *l) + u_desired(3) / (4 * b);
//    w2_desired(2) = u_desired(0) / (4 * k) + u_desired(2) / (2 * k *l) - u_desired(3) / (4 * b);
//    w2_desired(3) = u_desired(0) / (4 * k) + u_desired(1) / (2 * k *l) + u_desired(3) / (4 * b);

//    Vector4d w_desired;
//    w_desired(0) = qSqrt(w2_desired(0));
//    w_desired(1) = qSqrt(w2_desired(1));
//    w_desired(2) = qSqrt(w2_desired(2));
//    w_desired(3) = qSqrt(w2_desired(3));

//    w(0) = rotor1.Update(w_desired(0)).GetW();
//    w(1) = rotor2.Update(w_desired(1)).GetW();
//    w(2) = rotor3.Update(w_desired(2)).GetW();
//    w(3) = rotor4.Update(w_desired(3)).GetW();

//    w2(0) = w(0) * w(0);
//    w2(1) = w(1) * w(1);
//    w2(2) = w(2) * w(2);
//    w2(3) = w(3) * w(3);

//    w_r = w(1) + w(3) - w(0) - w(2);

    w2_desired = u_to_w2 * u_desired;

    // ограничение квадрата скорости по движкам
    for (int i = 0; i < w2_desired.size(); i++) {
        if (w2_desired(i) > maxW * maxW) {
            w2_desired(i) = maxW * maxW;
        }
        if (w2_desired(i) < minW * minW) {
            w2_desired(i) = minW * minW;
        }
    }

    // имитация расрутки двигателя
    Vector4d w2_delta = kw * (w2_desired - w2) * dt;
    w2 += w2_delta;

    w(0) = qSqrt(w2(0));
    w(1) = qSqrt(w2(1));
    w(2) = qSqrt(w2(2));
    w(3) = qSqrt(w2(3));

//    w_r = qSqrt(qAbs(w[1])) + qSqrt(qAbs(w[3])) - qSqrt(qAbs(w[0])) - qSqrt(qAbs(w[2]));

    // debug information

//    std::cout << "StepRotor" << std::endl;
//    std::cout << "\t w2_desired: " << w2_desired.transpose() << std::endl;
////    std::cout << "\t w_desired: " << w_desired.transpose() << std::endl;
//    std::cout << "\t w_delta  : " << w2_delta.transpose() << std::endl;
////    std::cout << "\t sub_w    : " << (w_desired - w).transpose() << std::endl;
//    std::cout << "\t w        : " << w.transpose() << std::endl;
//    std::cout << "\t w2       : " << w2.transpose() << std::endl;
//    std::cout << "\t w_r      : " << w_r << std::endl;
}

void Quadcopter::StepModel() {

//    u_current(0) = k * (w2(0) + w2(1) + w2(2) + w2(3));
//    u_current(1) = l * k * (w2(3) - w2(1));
//    u_current(2) = l * k * (w2(2) - w2(0));
//    u_current(3) = b * (-w2(0) + w2(1) - w2(2) + w2(3));

    u_current = w2_to_u * w2;

    // attitude

    double phi_dot   = pqr(0);
    double theta_dot = pqr(1);
    double psi_dot   = pqr(2);

    // пока избавился от гироскопических сил
    double phi_dot_dot   = (theta_dot * psi_dot * (Iy - Iz) /* - Ir * theta_dot * w_r */ + u_current(1)) / Ix;
    double theta_dot_dot = (phi_dot   * psi_dot * (Iz - Ix) /* + Ir * phi_dot   * w_r */ + u_current(2)) / Iy;
    double psi_dot_dot   = (phi_dot * theta_dot * (Ix - Iy) + u_current(3)) / Iz;

    // сохраняю ускорения для записи в лог
    attitude_acceleration(0) = phi_dot_dot;
    attitude_acceleration(1) = theta_dot_dot;
    attitude_acceleration(2) = psi_dot_dot;

    // position

    double sin_phi = qSin(attitude(0));
    double cos_phi = qCos(attitude(0));
    double sin_theta = qSin(attitude(1));
    double cos_theta = qCos(attitude(1));
    double sin_psi = qSin(attitude(2));
    double cos_psi = qCos(attitude(2));

    // acceleration

    double x_acceleration = (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * u_current(0) / m;
    double y_acceleration = (cos_phi * sin_theta * cos_psi - sin_phi * sin_psi) * u_current(0) / m;
    double z_acceleration = cos_phi * cos_theta * u_current[0] / m - g;

    Vector3d pqr_dot;
    pqr_dot << phi_dot_dot, theta_dot_dot, psi_dot_dot;

    pqr += pqr_dot * dt;

    attitude += pqr * dt;

    Vector3d xyz_acceleration;
    xyz_acceleration << x_acceleration, y_acceleration, z_acceleration;
    xyz_velocity += xyz_acceleration * dt;
    xyz += xyz_velocity*dt;

    // debug info

//    std::cout << "Step:" << std::endl;

//    std::cout << "phi_dot  : " << phi_dot << std::endl;
//    std::cout << "theta_dot: " << theta_dot << std::endl;
//    std::cout << "psi_dot  : " << psi_dot << std::endl;

//    std::cout << "phi_dot_dot  : " << phi_dot_dot << std::endl;
//    std::cout << "theta_dot_dot: " << theta_dot_dot << std::endl;
//    std::cout << "psi_dot_dot  : " << psi_dot_dot << std::endl;

//    std::cout << "\t u_current: " << u_current.transpose() << std::endl;
//    std::cout << "\t pqr_dot  : " << pqr_dot.transpose() << std::endl;
//    std::cout << "\t pqr      : " << pqr.transpose() << std::endl;
//    std::cout << "\t xyz_accel: " << xyz_acceleration.transpose() << std::endl;
//    std::cout << "\t xyz_veloc: " << xyz_velocity.transpose() << std::endl;
}

void Quadcopter::Reset() {
    currentTime = 0;

    xyz             << 0.0, 0.0, 0.0;
    xyz_velocity    << 0.0, 0.0, 0.0;

    attitude    << 0.0, 0.0, 0.0;
    pqr         << 0.0, 0.0, 0.0;

    attitude_acceleration << 0.0, 0.0, 0.0;

    u_desired << m * g, 0.0, 0.0, 0.0;
    u_current = u_desired;

    w2_desired = u_to_w2 * u_desired;

    for (int i = 0; i < w.size(); i++) {
        w(i) = howerW;
        w2(i) = howerW * howerW;
    }

    blackBox.Reset();

      // debug info
//    std::cout << "Reset: " << std::endl;
//    std::cout << "\t xyz         : " << xyz.transpose() << std::endl;
//    std::cout << "\t xyz_velocity: " << xyz_velocity.transpose() << std::endl;
//    std::cout << "\t attitude    : " << attitude.transpose() << std::endl;
//    std::cout << "\t pqr         : " << pqr.transpose() << std::endl;
//    std::cout << "\t w           : " << w.transpose() << std::endl;
//    std::cout << "\t w2          : " << w2.transpose() << std::endl;
//    std::cout << "\t w2_desired  : " << w2_desired.transpose() << std::endl;
//    std::cout << "\t attitude_acc: " << attitude_acceleration << std::endl;
//    std::cout << "\t attitude_acc: " << attitude_acceleration << std::endl;
}

void Quadcopter::SaveState() {
    blackBox.SetAngles(attitude(0), attitude(1), attitude(2));
    blackBox.SetPosition(xyz(0), xyz(1), xyz(2));
    blackBox.SetTime(currentTime);
    blackBox.SetW1234(w(0), w(1), w(2), w(3));
    blackBox.SetU1234(u_current(0), u_current(1), u_current(2), u_current(3));

    blackBox.SetRollPitchYawVelocities(pqr(0), pqr(1), pqr(2));
    blackBox.SetRollPitchYawAccelerations(attitude_acceleration(0), attitude_acceleration(1), attitude_acceleration(2));


    // debug info
//    std::cout << "SaveState: " << std::endl;
//    std::cout << "\t xyz         : " << xyz.transpose() << std::endl;
//    std::cout << "\t xyz_velocity: " << xyz_velocity.transpose() << std::endl;
//    std::cout << "\t attitude    : " << attitude.transpose() << std::endl;
//    std::cout << "\t pqr         : " << pqr.transpose() << std::endl;
//    std::cout << "\t rotors_w    : " << w.transpose() << std::endl;
}



