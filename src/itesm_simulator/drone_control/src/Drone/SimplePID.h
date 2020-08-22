//
// Created by alberto on 28/02/19.
//

#ifndef PROJECT_SIMPLEPID_H
#define PROJECT_SIMPLEPID_H
#include <chrono>
#include <stdexcept>
#include <cmath>

struct SimplePIDConfig {
    double p = 0, i = 0, d = 0, f = 0, feedForward = 0;
    double max = 1, min = 0;
    bool clampedOutput = false;
};

class SimplePID {
public:
    explicit SimplePID(SimplePIDConfig pidConfig = {});

    const SimplePIDConfig &getConfig() const;

    void setConfig(const SimplePIDConfig &config);

    double getSetpoint() const;

    void setSetpoint(double setpoint);

    double getOutput() const;

    double update(double source);

private:
    double setpoint = 0;
    double errorIntegral = 0, errorDerivative = 0, lastError = 0;
    double output = 0;
    SimplePIDConfig config;

    std::chrono::high_resolution_clock ::time_point lastTimeUpdate = std::chrono::high_resolution_clock ::now();
};


#endif //PROJECT_SIMPLEPID_H
