#include "main.h"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"

//Controller & Chassis from main.cpp
extern pros::Controller controller;
extern lemlib::Chassis chassis;
void tankControl(){
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);    
}
