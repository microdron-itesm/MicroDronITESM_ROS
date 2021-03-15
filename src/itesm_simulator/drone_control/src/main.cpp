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
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

enum class AutomaticDirectionX {
    Off,
    Left,
    Right
};

enum class AutomaticDirectionY {
    Off,
    Front,
    Back
};

// enum AutomaticDirectionZ {
//     Off,
//     Up,
//     Down
// }

void automaticControls(float &pitchTarget, float &rollTarget, float &yawTarget, float dt, AutomaticDirectionX &x, AutomaticDirectionY &y){
        
    if(y != AutomaticDirectionY::Off){
        pitchTarget = (16 * M_PI / 180.0);
        if(y == AutomaticDirectionY::Back){
            pitchTarget = - pitchTarget;
        }
    } else {
        pitchTarget = 0;
    }

    if(x != AutomaticDirectionX::Off){
        rollTarget = (16 * M_PI / 180.0);
        if(x == AutomaticDirectionX::Left){
            rollTarget = - rollTarget;
        }
    } else {
        rollTarget = 0;
    }

    // if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q)){
    //     yawTarget += (float) (30.0f * M_PI / 180.0f) * dt;
    // }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::E)){
    //     yawTarget += (float) -(30.0f * M_PI / 180.0f) * dt;
    // }
}

void keyboardCommands(AutomaticDirectionX &x, AutomaticDirectionY &y){
    // if(sf::Keyboard::isKeyPressed(sf::Keyboard::Z)){ // Descend
    //         drone.setK(drone.getK() + 10);
    // }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::X)){ // Ascend
    //     drone.setK(drone.getK() - 10);
    // }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num0)){
        x = AutomaticDirectionX::Off;
        y = AutomaticDirectionY::Off;
    } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num1)){
        x = AutomaticDirectionX::Left;
        y = AutomaticDirectionY::Off;
    } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num2)){
        x = AutomaticDirectionX::Off;
        y = AutomaticDirectionY::Front;
    } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num3)){
        x = AutomaticDirectionX::Off;
        y = AutomaticDirectionY::Back;
    } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num4)){
        x = AutomaticDirectionX::Right;
        y = AutomaticDirectionY::Off;
    // } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num5)){
    //     x = AutomaticDirectionX::Left;
    //     y = AutomaticDirectionY::Off;
    }
}

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

std::string readFile(const std::string &fileName){
    std::ifstream ifs(fileName.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(fileSize);
    ifs.read(bytes.data(), fileSize);

    return std::string(bytes.data(), fileSize);
}

std::vector<std::vector<std::vector<double>>> organizeCoordinates(std::string file){
    std::vector<std::vector<std::vector<double>>> droneInstructions;
    int i = 0;
    std::string value = "";
    std::vector<double> coords;
    std::vector<std::vector<double>> coordCollection;

    while(file[i] != '\0'){
        char current = file [i];
        if(current == ';' && value.size() > 0){
            coords.push_back(std::stod(value));
            value = "";
        } else if(current == ','){
            if(value.size() > 0){
                coords.push_back(std::stod(value));
                value = "";
            }
            coordCollection.push_back(coords);
            coords.clear();
        } else if(current == '\n'){
            droneInstructions.push_back(coordCollection);
            coordCollection.clear();
        } else {
            value.push_back(current);
        }
        i++;
    }

    if(coordCollection.size() > 0){
        droneInstructions.push_back(coordCollection);
        coordCollection.clear();
    }

    return droneInstructions;
}

int main(int argc, char **argv) {
    if(argc < 2){
        printf("MISSING THE DIRECTIONS FILE!!\n");
        return 1;
    }

    std::vector<std::vector<std::vector<double>>> directions = organizeCoordinates(readFile(argv[1]));

    AutomaticDirectionX xDir = AutomaticDirectionX::Off;
    AutomaticDirectionY yDir = AutomaticDirectionY::Off;

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
        // if(sf::Joystick::isConnected(0)){
        //     joystickControl(pitchTarget, rollTarget, yawTarget, 0.016);
        // }else{
        //     keyboardControls(pitchTarget, rollTarget, yawTarget, 0.016);
        // }
        automaticControls(pitchTarget,rollTarget,yawTarget,0.016,xDir,yDir);
        keyboardCommands(xDir,yDir);
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
