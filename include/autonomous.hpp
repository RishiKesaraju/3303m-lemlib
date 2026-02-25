#pragma once


#include "lemlib/api.hpp"


// ---------------- AUTON ENUM ----------------
enum class AutonMode {
    NONE,
    RIGHT_NINE,
    RIGHT_SEVEN,
    RIGHT_FOUR,
    RIGHT_MID,
    RIGHT_AWP,
    LEFT_NINE,
    LEFT_SEVEN,
    LEFT_FOUR,
    LEFT_MID,
    LEFT_AWP,
    SAWP,
    TEST
};


extern AutonMode currentAuton;


// ---------------- GENERIC HELPERS ----------------
void exitcondition(lemlib::Pose target, double exitDist);
void no_auton();
void test_auton();
void right4plus3();


// ---------------- RIGHT AUTONS ----------------
void right_nine();
void right_seven();
void right_four();
void right_mid();
void right_awp();


// ---------------- LEFT AUTONS ----------------
void left_nine();
void left_seven();
void left_four();
void left_mid();
void left_awp();


// ---------------- SKILLS / AWP ----------------
void sawp();



