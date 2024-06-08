#include "robot_functions.h"
#include "robot_config.h"

// Define hardware objects
vex::brain Brain;
vex::motor rMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor lMotor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::gyro  myGyro(Brain.ThreeWirePort.A);
vex::sonar sonar_L(Brain.ThreeWirePort.C);
vex::sonar sonar_Top(Brain.ThreeWirePort.E);
vex::sonar sonar_Cube(Brain.ThreeWirePort.G);

int main() {
    sleepMs(2000);// sleep to allow robot and cubes to fall to the ground
    // Start here
    pre_auton();
    autonomous();
}
