/**
 * Alberto Jahuey Moncada
 *
 * Tool used to simulate the drone algorithms
 *
 * */

#include <iostream>
#include <SFML/Graphics.hpp>
#include <chrono>
#include "PropellerDrawable.h"
#include "ros/ros.h"
#include "Drone/Drone.h"


void keyboardControls(float &pitchTarget, float &rollTarget, float &yawTarget, float dt){
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::W)){
            pitchTarget = (16 * M_PI / 180.0);
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::S)){
            pitchTarget = -(16 * M_PI / 180.0);
        }else{
            pitchTarget = 0;
        }

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
            rollTarget = -(16 * M_PI / 180.0);
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::D)){
            rollTarget = (16 * M_PI / 180.0);
        }else{
            rollTarget = 0;
        }

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q)){
            yawTarget += (float) (30.0f * M_PI / 180.0f) * dt;
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::E)){
            yawTarget += (float) -(30.0f * M_PI / 180.0f) * dt;
        }
}

void joystickControl(float &pitchTarget, float &rollTarget, float &yawTarget, float dt){
    float maxAngle = 16 * M_PI / 180.0;
    float maxYawRate = 30 * M_PI / 180.0;

    pitchTarget = (float) (-sf::Joystick::getAxisPosition(0, sf::Joystick::Y) / 100.0) * maxAngle;
    rollTarget = (float) (sf::Joystick::getAxisPosition(0, sf::Joystick::X) / 100.0) * maxAngle;
    yawTarget += (float) (- sf::Joystick::getAxisPosition(0, sf::Joystick::U) / 100.0) * maxYawRate * dt;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");


    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(200),
                                          static_cast<unsigned int>(200)), "Drone Control", sf::Style::Close);
    window.setFramerateLimit(60);
    PropellerDrawable bottomLeftProp;
    PropellerDrawable bottomRightProp;
    PropellerDrawable topLeftProp;
    PropellerDrawable topRightProp;

    bottomLeftProp.setPosition(0,100);
    topLeftProp.setPosition(0, 0);

    bottomRightProp.setPosition(100,100);
    topRightProp.setPosition(100,0);

    Drone drone;
    drone.setK(838);

    float heightTarget = 0, yawTarget = 0, pitchTarget = 0, rollTarget = 0;

    while (window.isOpen()) {
        drone.sendHeartBeat();
        if(sf::Joystick::isConnected(0)){
            joystickControl(pitchTarget, rollTarget, yawTarget, 0.016);
        }else{
            keyboardControls(pitchTarget, rollTarget, yawTarget, 0.016);
        }
        drone.setSetpoints(pitchTarget, rollTarget, yawTarget, 1.0);

        bottomLeftProp.setCurrentSpeed(drone.getMotorOutputBL());
        bottomRightProp.setCurrentSpeed(-drone.getMotorOutputBR());

        topLeftProp.setCurrentSpeed(-drone.getMotorOutputTL());
        topRightProp.setCurrentSpeed(drone.getMotorOutputTR());

        bottomLeftProp.update();
        bottomRightProp.update();
        topLeftProp.update();
        topRightProp.update();

        sf::Event event;
        while (window.pollEvent(event)) {


            if (event.type == sf::Event::Closed) {
                drone.setAllMotorOutput(0,0,0,0);
                window.close();
            } else if (event.type == sf::Event::Resized) {
                window.setView(
                        sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
            }
        }

        window.clear(sf::Color(100,100,100));
        window.draw(bottomLeftProp);
        window.draw(bottomRightProp);
        window.draw(topLeftProp);
        window.draw(topRightProp);

        window.display();
        drone.update();
        ros::spinOnce();
    }
    return 0;
}
