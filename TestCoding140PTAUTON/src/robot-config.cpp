#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor tilter = motor(PORT3, ratio36_1, false);
motor mogo = motor(PORT20, ratio36_1, false);
motor RightMotorA = motor(PORT12, ratio6_1, false);
motor RightMotorB = motor(PORT16, ratio6_1, true);
motor_group Right = motor_group(RightMotorA, RightMotorB);
motor LeftMotorA = motor(PORT9, ratio6_1, false);
motor LeftMotorB = motor(PORT18, ratio6_1, true);
motor_group Left = motor_group(LeftMotorA, LeftMotorB);
inertial Inertial = inertial(PORT19);
motor Claw = motor(PORT5, ratio36_1, false);
motor lift = motor(PORT10, ratio36_1, false);
motor lift2 = motor(PORT11, ratio36_1, true);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}