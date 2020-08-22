//
// Created by alberto on 28/02/19.
//

#include "SimplePID.h"
#include "ros/ros.h"

SimplePID::SimplePID(SimplePIDConfig pidConfig) {
    this->config = pidConfig;
}

const SimplePIDConfig &SimplePID::getConfig() const {
    return config;
}

void SimplePID::setConfig(const SimplePIDConfig &config) {
    if(config.max < config.min && config.clampedOutput){
        throw std::logic_error("Max output is less than min on SimplePIDConfig");
    }
    this->config = config;
}

double SimplePID::getSetpoint() const {
    return setpoint;
}

void SimplePID::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

double SimplePID::getOutput() const {
    return output;
}

double SimplePID::update(double source) {
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock ::now();

    double timeStep = std::chrono::duration<double>(currentTime - lastTimeUpdate).count();

    double error = setpoint - source;

    errorIntegral += error * timeStep;

    errorDerivative = (error - lastError) / timeStep;

    output = error * config.p + errorIntegral * config.i + errorDerivative * config.d + setpoint * config.f + config.feedForward;

    if(config.clampedOutput){
        if(std::abs(output) > config.max){
            output = std::copysign(config.max, output);
        }

        if(std::abs(output) < config.min){
            output = std::copysign(config.min, output);
        }
    }

    lastError = error;
    lastTimeUpdate = std::chrono::high_resolution_clock::now();
    
    return output;
}

