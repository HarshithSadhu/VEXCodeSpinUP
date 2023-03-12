using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor tilter;
extern motor mogo;
extern motor_group Right;
extern motor_group Left;
extern inertial Inertial;
extern motor Claw;
extern motor lift;
extern motor lift2;
extern digital_out DigitalOutH;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );