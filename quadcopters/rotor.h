#ifndef ROTOR_H
#define ROTOR_H

class Rotor {
    double hower_w;
    double current_w;
    double dt;

    const double kw = 2.0 / 0.18;

public:
    Rotor(double _current_w, double _dt) {
        current_w = _current_w;
        dt = _dt;
    }

    Rotor& Update(double desiredW) {
        current_w += kw * (desiredW - current_w) * dt;
        return *this;
    }

    double GetW() const {
        return current_w;
    }
};

#endif // ROTOR_H
