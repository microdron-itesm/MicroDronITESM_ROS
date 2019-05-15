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
#include "Drone.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");


    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(200),
                                          static_cast<unsigned int>(200)), "EctoSim", sf::Style::Close);
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
    drone.useManualThrust(false);
    float currentManualThrust = 0.0;

    while (window.isOpen()) {
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
            drone.setTargetRoll(-10 *  M_PI / 180.0 );
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::D)){
            drone.setTargetRoll(10 *  M_PI / 180.0);
        }else{
            drone.setTargetRoll(0);
        }

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::W)){
            drone.setTargetPitch(-10 *  M_PI / 180.0);
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::S)){
            drone.setTargetPitch(10  *  M_PI / 180.0);
        }else{
            drone.setTargetPitch(0);
        }

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q)){
            drone.setTargetYaw(30 *  M_PI / 180.0 );
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::E)){
            drone.setTargetYaw(-30 *  M_PI / 180.0);
        }else{
            drone.setTargetYaw(0);
        }

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
            drone.setTargetHeight(1.5);
            currentManualThrust += .05 * 0.016;
        }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F)){
            drone.setTargetHeight(0.5);
            currentManualThrust -= .05 * 0.016;
        }else{
            drone.setTargetHeight(1.0);
        }

        if(currentManualThrust > 1.0){
            currentManualThrust = 1.0;
        }
        if(currentManualThrust < 0.0){
            currentManualThrust = 0.0;
        }
        drone.setManualThrust(currentManualThrust);
        drone.killMotors(sf::Keyboard::isKeyPressed(sf::Keyboard::K));

        drone.update();

        bottomLeftProp.setCurrentSpeed(drone.getBottomLeftAngularSpeed());
        bottomRightProp.setCurrentSpeed(-drone.getBottomRightAngularSpeed());

        topLeftProp.setCurrentSpeed(-drone.getTopLeftAngularSpeed());
        topRightProp.setCurrentSpeed(drone.getTopRightAngularspeed());

        bottomLeftProp.update();
        bottomRightProp.update();
        topLeftProp.update();
        topRightProp.update();

        sf::Event event;
        while (window.pollEvent(event)) {


            if (event.type == sf::Event::Closed) {
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

        ros::spinOnce();
    }

    return 0;
}