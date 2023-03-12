#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftDriveMotorA = motor(PORT9, ratio6_1, false);
motor leftDriveMotorB = motor(PORT8, ratio6_1, false);
motor_group leftDrive = motor_group(leftDriveMotorA, leftDriveMotorB);
motor rightDriveMotorA = motor(PORT6, ratio6_1, true);
motor rightDriveMotorB = motor(PORT7, ratio6_1, true);
motor_group rightDrive = motor_group(rightDriveMotorA, rightDriveMotorB);
controller Controller1 = controller(primary);
motor ringIntake = motor(PORT11, ratio18_1, false);
motor diskIntake = motor(PORT18, ratio18_1, false);
motor shooterMotorA = motor(PORT16, ratio6_1, true);
motor shooterMotorB = motor(PORT17, ratio6_1, false);
motor_group shooter = motor_group(shooterMotorA, shooterMotorB);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
/*vex-vision-config:begin*/
signature Vision20__SIGRED = signature (1, 10831, 12225, 11528, -1683, -1039, -1360, 3, 0);
signature Vision20__SIGBLUE = signature (2, -3917, -3171, -3544, 17367, 19059, 18213, 3, 0);
signature Vision20__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision20__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision20 = vision (PORT20, 50, Vision20__SIGRED, Vision20__SIGBLUE, Vision20__SIG_3, Vision20__SIG_4);
/*vex-vision-config:end*/
distance Distance19 = distance(PORT19);
controller Controller2 = controller(partner);
digital_out DigitalOutC = digital_out(Brain.ThreeWirePort.C);
/*vex-vision-config:begin*/
vision Vision21 = vision (PORT21, 50);
/*vex-vision-config:end*/
inertial Inertial = inertial(PORT15);
digital_out DigitalOutE = digital_out(Brain.ThreeWirePort.E);

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