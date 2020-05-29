#ifndef CONFIG_H
#define CONFIG_H

#include <QtMath>

//namespace Config {
//    const double dron_mass   = 0.65;
//    const double dron_length = 0.23;

//    const double Ix = 7.5e-3;
//    const double Iy = Ix;
//    const double Iz = 1.3e-2;
//    const double Ir = 1.0e-4;

//    const double g = 9.81;

//    const double k = 0.1;
//    const double b = 0.01;

//    const double kw = 2.0/0.18;
//    const double saturation_k = 50.0;
//}

//namespace Config {

//    const double dron_mass   = 0.5;
//    const double dron_length = 0.2;

//    const double Ix = 4.85e-3;
//    const double Iy = Ix;
//    const double Iz = 8.81e-3;
//    const double Ir = 1.0e-4;

//    const double g = 9.81;

//    const double k = 2.92e-6;
//    const double b = 1.12e-7;

//    const double hower_w = qSqrt(dron_mass * g / (4 * k));

//    const double min_w = 0.37 * hower_w;
//    const double max_w = 1.44 * hower_w;

//    const double kw = 2.0/0.18;
//    const double saturation_k = 50.0;
//}

namespace Config {

    const double dron_mass   = 1.25; // масса коптера
    const double dron_length = 0.2;  // длина луча

    // инерции
    const double Ix = 0.002353;
    const double Iy = Ix;
    const double Iz = 0.004706;
    const double Ir = 2.0e-5;

    // гравитационная постоянная
    const double g = 9.81;

    const double k = 2.92e-6;  // коэфициент тяги двигателя
    const double b = 1.12e-7;  // коэфициент "трение" двигателя

    // скорость вращения винтов при зависании (рад/с)
    const double hower_w = qSqrt(dron_mass * g / (4 * k));

    // минимальные и максимальные скорости вращения винтов
    const double min_w = 0.37 * hower_w;
    const double max_w = 1.44 * hower_w;

    const double kw = 2.0/0.18; // коэфициент прироста скорости
}


#endif // CONFIG_H
