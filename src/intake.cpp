#include "intake.hpp"
#include "main.h"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"

// controller & hardware from main.cpp
extern pros::Controller controller;
extern pros::Motor firstStage;
extern pros::Motor secondStage;

void intakeInit() {
    firstStage.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    secondStage.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void intakeControl() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        firstStage.move(127);
        secondStage.brake();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        firstStage.move(-127);
        secondStage.move(127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        firstStage.move(127);
        secondStage.move(-127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
    }
    else {
        firstStage.brake();
        secondStage.brake();
    }
}



