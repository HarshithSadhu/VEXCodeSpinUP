using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern motor_group leftDrive;
extern motor_group rightDrive;
extern controller Controller1;
extern motor ringIntake;
extern motor diskIntake;
extern motor_group shooter;
extern digital_out DigitalOutA;
extern signature Vision20__SIGRED;
extern signature Vision20__SIGBLUE;
extern signature Vision20__SIG_3;
extern signature Vision20__SIG_4;
extern signature Vision20__SIG_5;
extern signature Vision20__SIG_6;
extern signature Vision20__SIG_7;
extern vision Vision20;
extern distance Distance19;
extern controller Controller2;
extern digital_out DigitalOutC;
extern signature Vision21__SIG_1;
extern signature Vision21__SIG_2;
extern signature Vision21__SIG_3;
extern signature Vision21__SIG_4;
extern signature Vision21__SIG_5;
extern signature Vision21__SIG_6;
extern signature Vision21__SIG_7;
extern vision Vision21;
extern inertial Inertial;
extern digital_out DigitalOutE;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );