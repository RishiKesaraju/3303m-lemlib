#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "intake.hpp"
#include "autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pneumatics.hpp"
#include "tankControl.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::Pneumatics matchloader {'A', false};
pros::adi::Pneumatics descoreHigh {'B', true};
pros::adi::Pneumatics descoreMid {'C', false};
pros::Motor firstStage(7,pros::MotorGearset::blue);
pros::Motor secondStage(5,pros::MotorGearset::blue);
pros::Distance distFront(1);
pros::Distance distBack(2);
pros::Distance distLeft(4);
pros::Distance distRight(6);
// motor groups, 
pros::MotorGroup leftMotors({-13, 17, -12}, pros::MotorGearset::blue); // left motor group - ports 13 (reversed), 17, 12 (reversed)
pros::MotorGroup rightMotors({18, -19, 20}, pros::MotorGearset::blue); // right motor group - ports 18, 19 (reversed), 20

// Inertial Sensor on port 9
pros::Imu imu(9);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 3, reversed
pros::Rotation horizontalEnc(3);
// vertical tracking wheel encoder. Rotation sensor, port 10, reversed
pros::Rotation verticalEnc(-10);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.65);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.05);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              343, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(11.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            50, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    intakeInit();
    pneumaticsInit();
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

void competition_initialize() {}
void autonomous() {
    test_auton();
}   

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    while (true) {
        tankControl();
        intakeControl();
        pneumaticsControl();
        std::cout << chassis.getPose().y << std::endl;
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        pros::delay(10);
    }
}
