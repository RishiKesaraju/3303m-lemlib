#include "pneumatics.hpp"  
#include "main.h"
#include "pros/misc.h"

// controller & hardware from main.cpp
extern pros::Controller controller;
extern pros::adi::Pneumatics descoreHigh;
extern pros::adi::Pneumatics descoreMid;
extern pros::adi::Pneumatics matchloader;
bool descoreToggleState = false;
bool lastX = false;
bool matchloaderToggle = false;
bool lastRight = false;
bool midgoalToggle = false;
uint32_t lastXTime = 0;

void pneumaticsInit(){
    descoreHigh.set_value(true);
    descoreMid.set_value(false);
    matchloader.set_value(false);
}
void pneumaticsControl() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        descoreHigh.set_value(false);
    } else {
        descoreHigh.set_value(true);
    }
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        descoreMid.set_value(true);
    } else {
        descoreMid.set_value(false);
    }
bool rightPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
if (rightPressed && !lastRight ) {
    matchloaderToggle = !matchloaderToggle;
    matchloader.set_value(matchloaderToggle);
}
lastRight = rightPressed;
}

