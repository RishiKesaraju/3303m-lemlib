#include "autonomous.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"


// ---------------- EXTERNS ----------------
extern lemlib::Chassis chassis;
extern pros::Motor firstStage;
extern pros::Motor secondStage;
extern pros::adi::Pneumatics descoreHigh;
extern pros::adi::Pneumatics descoreMid;
extern pros::adi::Pneumatics matchloader;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
AutonMode currentAuton = AutonMode::NONE;

void exitcondition(lemlib::Pose target, double exitDist) {
    chassis.waitUntil(
        fabs(chassis.getPose().distance(target)) - exitDist
    );
    chassis.cancelMotion();
}

void customMoveToPose(double targetX, double targetY, double targetHeading, int timeout, float maxSpeed = 127) {
    uint32_t startTime = pros::millis();
    
    double prevLinearError = 0;
    double prevAngularError = 0;

    // PID constants
    double linearKP = 10.0;
    double linearKD = 5.0;
    double angularKP = 6.0;
    double angularKD = 3.0;

    while (pros::millis() - startTime < (uint32_t)timeout) {
        double currentX = chassis.getPose().x;
        double currentY = chassis.getPose().y;
        double currentHeading = chassis.getPose().theta;

        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distToTarget = sqrt(dx * dx + dy * dy);

        if (distToTarget < 1.0) break;

        // Angle to target
        double angleToTarget = atan2(dx, dy) * 180.0 / M_PI;

        // Blend toward final heading as we approach
        double blendFactor = fmax(0.0, fmin(1.0, (10.0 - distToTarget) / 10.0));
        double desiredHeading = angleToTarget + blendFactor * (targetHeading - angleToTarget);

        // Angular error
        double angularError = desiredHeading - currentHeading;
        while (angularError > 180) angularError -= 360;
        while (angularError < -180) angularError += 360;

        // Linear error
        double linearError = distToTarget;

        // PID
        double linearPower  = (linearKP * linearError)  + (linearKD  * (linearError  - prevLinearError));
        double angularPower = (angularKP * angularError) + (angularKD * (angularError - prevAngularError));

        // Clamp linear
        linearPower = fmax(-maxSpeed, fmin(maxSpeed, linearPower));

        // Mix
        double leftPower  = linearPower - angularPower;
        double rightPower = linearPower + angularPower;

        // Normalize
        double maxPower = fmax(fabs(leftPower), fabs(rightPower));
        if (maxPower > maxSpeed) {
            leftPower  = leftPower  / maxPower * maxSpeed;
            rightPower = rightPower / maxPower * maxSpeed;
        }

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        prevLinearError  = linearError;
        prevAngularError = angularError;

        pros::delay(10);
    }

    leftMotors.brake();
    rightMotors.brake();
}

void test_auton() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(0, 0, 0);
    //chassis.turnToHeading(90,500, {.maxSpeed=127, .minSpeed=127});
    customMoveToPose(10, 24,0, 3000, 127);
    chassis.waitUntilDone();

}

void right4plus3(){
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,-32,1000, {.forwards=false, .maxSpeed=127, .minSpeed=127, .earlyExitRange=2});
    matchloader.set_value(true);
    firstStage.move(127);
    chassis.turnToHeading(-90,500, {.maxSpeed=127, .minSpeed=127});
	pros::delay(1000);
    chassis.moveToPoint(-5,-32,500, {.maxSpeed=100, .minSpeed=40});
    chassis.moveToPoint(37,-31.5,750, {.forwards=false, .maxSpeed=127});
	pros::delay(750);
    secondStage.move(-127);
}

void no_auton() {}
void right_nine() {}
void right_seven() {}
void right_four() {}
void right_mid() {}
void right_awp() {}
void left_nine() {}
void left_seven() {}
void left_four() {}
void left_mid() {}
void left_awp() {}
void sawp() {}

