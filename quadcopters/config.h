#ifndef CONFIG_H
#define CONFIG_H

namespace Config {
    const double dron_mass   = 0.65;
    const double dron_length = 0.23;

    const double Ix = 7.5e-3;
    const double Iy = Ix;
    const double Iz = 1.3e-2;
    const double Ir = 1.0e-4;

    const double g = 9.81;

    const double k = 0.1;
    const double b = 0.01;

    const double kw = 2.0/0.18;
    const double saturation_k = 50.0;
}

//namespace Config {
//    const double dron_mass   = 0.5;
//    const double dron_length = 0.2;

//    const double Ix = 7.5e-3;
//    const double Iy = Ix;
//    const double Iz = 1.3e-2;
//    const double Ir = 1.0e-4;

//    const double g = 9.81;

//    const double k = 2.92e-6;
//    const double b = 1.12e-7;

//    const double kw = 2.0/0.18;
//    const double saturation_k = 50.0;
//}




#endif // CONFIG_H
