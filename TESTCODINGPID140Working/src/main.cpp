/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Harshith Sadhu                                            */
/*    Created:      Mon Jul 12 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// tilter               motor         3               
// mogo                 motor         20              
// Right                motor_group   12, 16          
// Left                 motor_group   9, 18           
// Inertial             inertial      19              
// Claw                 motor         5               
// lift                 motor         10              
// lift2                motor         11              
// DigitalOutH          digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----



#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;


double kPAngle = 0.2;
double kIAngle = 0.1;
double kDAngle = 0.1;
double maxSpeed = 100;



void rotatePID(double angleTurn, turnType turnDir) {
  double error = 0;
  double prevError = 0;
  double derivative = 0;
  double integral = 0;

  while( fabs(Inertial.rotation() - angleTurn) > 0.5) {
    error = angleTurn - Inertial.rotation(rotationUnits::deg);
    derivative = error - prevError;
    prevError = error;

    if(fabs(error) < 5 && error != 0) {
      integral += error;
    }
    else {
      integral = 0;
    } 

    double powerDrive = error*kPAngle + derivative*kDAngle + integral*kIAngle;

    if(powerDrive > maxSpeed) {
      powerDrive = maxSpeed;
    }
    else if(powerDrive < -maxSpeed) {
      powerDrive = -maxSpeed;
    }

    //cout << "power: " << powerDrive << endl;
    //cout << "angle: " << iSensor.rotation() << endl;
    //cout << "error: " << error << endl;
    // cout << "prevError: " << prevError << endl;
    // cout << "derivative: " << derivative << endl;
    // cout << "integral: " << integral*kIDrive << endl;
    //cout << "////" << endl;

    // turning = true;
    // goalVoltage = powerDrive;

    if(turnDir == right){
      Right.spin(reverse, powerDrive, voltageUnits::volt);
      Left.spin(forward, powerDrive, voltageUnits::volt);
    }

    if(turnDir == left){
      Right.spin(forward, powerDrive, voltageUnits::volt);
      Left.spin(reverse, powerDrive, voltageUnits::volt);
    }
    
    
    this_thread::sleep_for(15);
  }
  
  // turning = false;
  // goalVoltage = 0;
  Left.stop(brake);
  Right.stop(brake);
}

int INCH = 90;
///
// Globals
//  - it's good practice to name globals in all caps
//  - we also try to avoid "hard coding" number.  use variables wherever
//  - you can, so you can change things when you eventually need to
///
const int resetTime     = 3000;  // the amount of time it takes for the sensors to reset
bool      resetSuccess = false; // boolean for checking wether the reset was successful or there was a problem


const int delayTime =  10; // How long the loop should wait before resetting

const int mobileGoal   =  490; // What position the back lift should turn to

const int tilOut = -350;  // How far out the tilter should default to
const int tilIn  =  100; // How far in the tilter should default to
const double onedegree = 2.33766234;
// Lift constants
const int positions = 3; // Number of different height settings
const int liftHeight[positions] = {0, 520, 1000}; // Positions for the lift, motor spinning degrees
const int speedUp   = 100;
const int speedDown = 100;
int liftPosition;

// instantiates voltage
const int scaleVal = 120;
void setTankMethod(int l, int r) {
  Left.spin(fwd, l*scaleVal, voltageUnits::mV);

  Right.spin(fwd, r*scaleVal, voltageUnits::mV);

     

}
// Different type of braking systems
void BRAKE() {

  Left.setStopping(hold);
  Right.setStopping(hold);
}
void COAST() {

  Left.setStopping(coast);
  Right.setStopping(coast);
}

//functions for spinning the back lift, front lift, and tilter x degrees
void mogoSet  (int input) { mogo.  spin(fwd, input*scaleVal, voltageUnits::mV); }
void tilterSET(int input) { tilter.spin(fwd, input*scaleVal, voltageUnits::mV); }
void liftSet  (int input) { lift.  spin(fwd, input*scaleVal, voltageUnits::mV); }

// Sets instant position for the back lift, front lift, and tilter x degrees
void putMogoPosition  (int pos, int speed) { mogo.  startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); }
void putTilterPosition(int pos, int speed) { tilter.startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); }
//poop
void set_lift_position  (int pos, int speed) { 

  lift2.startRotateTo(pos, rotationUnits::deg, 100, velocityUnits::pct);
  lift.startRotateTo(pos, rotationUnits::deg, 100, velocityUnits::pct);
  //tilter.startRotateTo(200, rotationUnits::deg, 100, velocityUnits::pct);
  

  //lift.spinFor(pos, rotationUnits::deg, speed, velocityUnits::pct); 

  //lift.spinFor(pos, rotationUnits::deg, speed, velocityUnits::pct);
  //lift.setVelocity(70, velocityUnits::pct); 
  //lift.spinFor(forward, pos, degrees, false);
  //lift.rotateFor(directionType dir, double rotation, rotationUnits units)
  
  
  
  }



///
// Reset Sensors
//  - To ensure computational accuracy, it is imperative for all of the motors to start in their relative positions
//  - These functions moves all the mechanics/motors to their starting positions
//  - limit for sensor
///
void zero_sensors() {
  bool run = true;
  bool tilter_zero = false;
  bool mogo_zero   = false;
  bool lift_zero   = false;

  bool last_tilter = false;
  bool last_mogo   = false;
  bool last_lift   = false;

  int timeout_timer = 0;

  tilterSET(0);
  mogoSet(0);
  liftSet(-40);
  liftPosition = -40;

  wait(200, msec);
  while (run) {
    last_tilter = tilter_zero;
    last_mogo   = mogo_zero;
    last_lift   = lift_zero;

    // Brings tilter until it is back to its starting or 0 position
    if (tilter.velocity(percentUnits::pct) == 0) {
      tilterSET(0);
      tilter_zero = true;
      tilter.resetPosition();
    }

    // Bring mobile goal lift back until it is back to its starting or 0 position
    if (mogo.velocity(percentUnits::pct) == 0) {
      mogoSet(0);
      mogo_zero = true;
      mogo.resetPosition();
    }

    // Brings lift down until it is back to its starting position
    if (lift.velocity(percentUnits::pct) == 0) {
      liftSet(0);
      liftPosition = 0;
      lift_zero = true;
      lift.resetPosition();
    }

    // stops code for a while
    if (tilter_zero && mogo_zero && lift_zero) {
      run = false;
    }

    // If any subsystems reset, reset the timer to 0
    if (last_tilter!=tilter_zero || last_mogo!=mogo_zero || last_lift!=lift_zero) {
      timeout_timer = 0;
    } else {
      // Stop loop once the timer is greater than the time it takes to reset
      timeout_timer+=delayTime;
      if (timeout_timer>resetTime) {
        run = false;
      }
    }

    //to limit CPU strain
    wait(delayTime, msec);
  
  }
  // Makes sure all motors stop running
  tilterSET(0);
  mogoSet  (0);
  liftSet  (0);
  liftPosition = 0;

  // Final reset of all mechanisms
  if (!tilter_zero) tilter.resetPosition();
  if (!mogo_zero)   mogo.  resetPosition();
  if (!lift_zero)   lift.  resetPosition();

  //Sets the parameters to make sure the code knows it is ready for execution
  resetSuccess = true;
}



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  zero_sensors();
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

bool did_auto_finish = false;
void autonomous(void) {

  //Beginning of Autonomous PID Control
  Inertial.calibrate(2000);
  this_thread::sleep_for(2000);
  
  
  

  //Should rotate the bot 90 degrees to the right
  Inertial.setRotation(0, rotationUnits::deg);
  //Brain.Screen.print(Inertial.rotation());

  
  
  //Brain.Screen.print(Inertial.rotation());
  //wait(60, seconds);
  //rotatePID(-90, right);
  //wait(2, seconds);
  //rotatePID(0, right);
  //wait(2, seconds);
  //rotatePID(90, right);
  //wait(2, seconds);

  //rotatePID(360, right);

  //rotatePID(180, right);
  //this_thread::sleep_for(1000);
  //rotatePID(90, left);
  //this_thread::sleep_for(1000);
  //rotatePID(0, right);
  //this_thread::sleep_for(1000);
  

  //wait(60, seconds);

  tilter.setVelocity(100, velocityUnits::pct); 
  tilter.spinFor(-555, degrees, false);
  wait(0.5, seconds);

  //clamps down to the blue mobile goal
  Left.setVelocity(50, velocityUnits::pct); 
  Right.setVelocity(50, velocityUnits::pct);
  Left.spinFor(forward, INCH * 14, degrees, false);
  Right.spinFor(forward, INCH * 14, degrees);
  tilter.setVelocity(100, velocityUnits::pct); 
  tilter.spinFor(555, degrees, false);
  wait(2, seconds);


//Spins so that the robot is aligned
  lift.spinFor(forward, 50, degrees, false);
  lift2.spinFor(forward, 50, degrees, false);
  Right.setVelocity(50, velocityUnits::pct);
  Right.spinFor(reverse, 1570, degrees);

  //spins the mogolift down
  mogo.setVelocity(100, velocityUnits::pct);
   mogo.spinFor(forward, 450, degrees, false);
   wait(0.5, seconds);


  //goes all the way to the yellow goal and picks it up
  Left.setVelocity(60, velocityUnits::pct); 
  Right.setVelocity(60, velocityUnits::pct);
  Left.spinFor(reverse, INCH * 32, degrees, false);
  Right.spinFor(reverse, INCH * 32, degrees);
  Left.setVelocity(15, velocityUnits::pct); 
  Right.setVelocity(15, velocityUnits::pct);
  Left.spinFor(reverse, INCH * 3, degrees, false);
  Right.spinFor(reverse, INCH * 3, degrees);
  mogo.setVelocity(100, velocityUnits::pct);
  mogo.spinFor(reverse, 450, degrees, false);
   wait(2, seconds);

  //Lifts up the goal and starts spinning the robot
  lift.setVelocity(100, velocityUnits::pct);
  lift2.setVelocity(100, velocityUnits::pct);
  lift.spinFor(forward, 500, degrees, false);
  lift2.spinFor(forward, 500, degrees, false);
  Left.setVelocity(70, velocityUnits::pct); 
  Right.setVelocity(70, velocityUnits::pct);
  rotatePID(-31, right);
  //Left.spinFor(reverse, 1015, degrees, false);
  //Right.spinFor(forward, 1015, degrees);

  //wait(60, seconds);

  //slows down and goes toward the platform
  Left.setVelocity(40, velocityUnits::pct); 
  Right.setVelocity(40, velocityUnits::pct);
  Left.spinFor(forward, INCH * 33.5, degrees, false);
  Right.spinFor(forward, INCH * 33.5, degrees);
  //Left.spinFor(reverse, 650, degrees, false);
  //Right.spinFor(forward, 650, degrees);
  rotatePID(-90, right);
  //wait(60, seconds);
  Left.spinFor(forward, 15 * INCH, degrees, false);
  Right.spinFor(forward, 15 * INCH, degrees, false);
  wait(3, seconds);

  //down and up for lift
  lift.spinFor(reverse, 100, degrees, false);
  lift2.spinFor(reverse, 100, degrees, false);
  wait(2, seconds);
  //drops the goal on the platform
  tilter.spinFor(-405, degrees, false);
  wait(1, seconds);


  lift.spinFor(forward, 200, degrees, false);
  lift2.spinFor(forward, 200, degrees);


  //goes away from the platform and drops the lift
  Left.spinFor(reverse, 11 * INCH, degrees, false);
  lift.spinFor(reverse, 700, degrees, false);
  lift2.spinFor(reverse, 700, degrees, false);
  Right.spinFor(reverse, 11 * INCH, degrees);
  //lift.spinFor(reverse, 500, degrees);
  
  //Spinning right after the robot slams into blue alliance goal
  //Left.spinFor(forward, 1370, degrees, false);
  //Right.spinFor(reverse, 1370, degrees);
  rotatePID(27, right);

  
  //wait(120, seconds);
  //35

  //goes to the yellow goal and clamps
  Left.setVelocity(30, velocityUnits::pct); 
  Right.setVelocity(30, velocityUnits::pct);
  Left.spinFor(forward, 27 * INCH, degrees, false);
  Right.spinFor(forward, 27 * INCH, degrees);
  wait(0.5, seconds);
  tilter.spinFor(405, degrees, false);
  wait(1, seconds);
  Left.setVelocity(40, velocityUnits::pct); 
  Right.setVelocity(40, velocityUnits::pct);

  //picks up the goal and starts turns towards the platform
  lift.spinFor(forward, 560, degrees, false);
  lift2.spinFor(forward, 560, degrees, false);
  
  wait(1, seconds);
  //Left.spinFor(forward, 600, degrees, false);
  //Right.spinFor(reverse, 600, degrees);
  rotatePID(121, right);
  //wait(60, seconds);


  //wait(100, seconds);

  //goes towards the platform and drops
  Left.spinFor(forward, 37 * INCH, degrees, false);
  Right.spinFor(forward, 37 * INCH, degrees);
  tilter.spinFor(-405, degrees, false);
  wait(1, seconds);

  //drops the lift and goes back
  lift.spinFor(forward, 200, degrees, false);
  lift2.spinFor(forward, 200, degrees);

  Left.spinFor(reverse, 2 * INCH, degrees, false);
  Right.spinFor(reverse, 2 * INCH, degrees);
  lift.spinFor(reverse, 810, degrees, false);
  lift2.spinFor(reverse, 810, degrees, false);
  wait(1, seconds);


  //aligns for blue goal and goes forward
  //
  //wait(60, seconds);
  Left.spinFor(reverse, 930, degrees, false);
  Right.spinFor(forward, 930, degrees);
  //while(Inertial.rotation() > 1){
    //Right.spin(forward, 10, voltageUnits::volt);
    //  Left.spin(reverse, 10, voltageUnits::volt);
  //}

  
  while(Inertial.rotation() > 4){
    Right.spin(forward, 5, voltageUnits::volt);
    Left.spin(reverse, 5, voltageUnits::volt);
  }


  Left.stop(brake);
  Right.stop(brake);


  Left.setVelocity(40, velocityUnits::pct); 
  Right.setVelocity(40, velocityUnits::pct);
  Left.spinFor(forward, 30 * INCH, degrees, false);
  Right.spinFor(forward, 30 * INCH, degrees, false);
  wait(3, seconds);
  tilter.spinFor(405, degrees, false);
  wait(3, seconds);

  //goes back and spins in place
  lift.spinFor(forward, 600, degrees, false);
  lift2.spinFor(forward, 600, degrees, false);
  Left.spinFor(reverse, 15.1 * INCH, degrees, false);
  Right.spinFor(reverse, 15.1 * INCH, degrees);

  rotatePID(-120, right);

  wait(1, seconds);

  //drives towards the blue platform and drops the blue goal
  Left.spinFor(forward, 60 * INCH, degrees, false);
  Right.spinFor(forward, 60 * INCH, degrees, false);
  wait(8, seconds);
  //wait(60, seconds);
  tilter.spinFor(-405, degrees, false);
  wait(0.5, seconds);
  

  //positions for red goal
  Left.setVelocity(100, velocityUnits::pct); 
  Right.setVelocity(100, velocityUnits::pct);
  Left.spinFor(reverse, 600 * INCH, degrees, false);
  Right.spinFor(reverse, 600 * INCH, degrees);

  wait(60, seconds);

  //END OF AUTONOMOUS
  lift.spinFor(reverse, 600, degrees, false);
  lift2.spinFor(reverse, 600, degrees);
  Left.spinFor(reverse, 550, degrees, false);
  Right.spinFor(forward, 550, degrees);

  Left.setVelocity(100, velocityUnits::pct); 
  Right.setVelocity(100, velocityUnits::pct);

  Left.spinFor(forward, 44 * INCH, degrees, false);
  Right.spinFor(forward, 44 * INCH, degrees);

  tilter.spinFor(405, degrees, false);
  wait(1, seconds);

  wait(60, seconds);

  //Code for Getting the last dragging point

  tilter.setVelocity(100, velocityUnits::pct); 
  tilter.spinFor(405, degrees, false);
  wait(1, seconds);

  lift.spinFor(forward, 100, degrees, false);
  lift2.spinFor(forward, 100, degrees, false);

  Left.setVelocity(50, velocityUnits::pct); 
  Right.setVelocity(50, velocityUnits::pct);
  Left.spinFor(forward, 270, degrees, false);
  Right.spinFor(reverse, 270, degrees);

  Left.spinFor(reverse, 80 * INCH, degrees, false);
  Right.spinFor(reverse, 80* INCH, degrees);

  //end of code


















  Left.spinFor(forward, 5 * INCH, degrees, false);
  Right.spinFor(forward, 5 * INCH, degrees);
  wait(60, seconds);








  wait(60, seconds);
 //Bring Tilter down
   //Swing Turn
   mogo.setVelocity(100, velocityUnits::pct);
   mogo.spinFor(forward, 450, degrees, false);
   wait(1, seconds);

  Left.setVelocity(50, velocityUnits::pct); 
  Right.setVelocity(50, velocityUnits::pct);
  Left.spinFor(reverse, INCH * 18, degrees, false);
  Right.spinFor(reverse, INCH * 18, degrees, false);

  wait(1, seconds);

  mogo.setVelocity(70, velocityUnits::pct);
   mogo.spinFor(reverse, 450, degrees, false);

   wait(1, seconds);
   
   Left.setVelocity(30, velocityUnits::pct); 
  Right.setVelocity(10, velocityUnits::pct);
  Left.spinFor(forward, 1300, degrees);
   Right.spinFor(forward, 800, degrees, false);
  

Left.setVelocity(50, velocityUnits::pct); 
  Right.setVelocity(50, velocityUnits::pct);
  Right.spinFor(reverse, 170, degrees, false);
  Left.spinFor(forward, 170, degrees);

  tilter.setVelocity(70, velocityUnits::pct); 
  tilter.spinFor(-405, degrees, false);


Right.spinFor(forward, INCH * 29, degrees, false);
  Left.spinFor(forward, INCH * 29, degrees);
  
  //Brings lift midway while the robot starts to move
  //lift.spinFor(forward, 275, degrees, false);
  //lift2.spinFor(forward, 275, degrees, false);
  wait(140, seconds);
  //Spins in place
  Left.setVelocity(60, velocityUnits::pct); 
  Right.setVelocity(10, velocityUnits::pct);
  Left.spinFor(forward, 320*onedegree*1.75, degrees, false);
  Right.spinFor(forward, 330*onedegree-480, degrees);

  //Moves Forward
  Left.setVelocity(60, velocityUnits::pct); 
  Right.setVelocity(60, velocityUnits::pct);
  Left.spinFor(forward, 20*onedegree + 350, degrees, false);
  Right.spinFor(forward, 20*onedegree + 350, degrees);
  tilter.setVelocity(45, velocityUnits::pct);
 
  //turn for less than turn for more

  //Bring Tilter down
  
  Left.spinFor(forward, 175, degrees, false);
  Right.spinFor(forward, 175, degrees);
  
  tilter.spinFor(455, degrees, false);
  wait(0.5, seconds);
  

  //Brings lift downward while the robot starts to move
  //lift.setVelocity(100, velocityUnits::pct);
  //lift2.setVelocity(100, velocityUnits::pct);
  //lift.spinFor(reverse, 510, degrees, false);
  //lift2.spinFor(reverse, 510, degrees, false);
  //wait(1, seconds)

  //Brings Robot backward
  Left.spinFor(reverse, 900, degrees, false);
  Right.spinFor(reverse, 900, degrees);
 
  //Bring tilter up
  //tilter.spinFor(350, degrees, false);

  //Rotates in Place
  Left.setVelocity(70, velocityUnits::pct); 
  Right.setVelocity(70, velocityUnits::pct);
  Left.spinFor(forward, -345*onedegree*2.43 * 1.15, degrees, false);
  Right.spinFor(forward, 345*onedegree*2.43 * 1.15, degrees);
  
  
  COAST();
  //Move Forward
  Left.setVelocity(30, velocityUnits::pct); 
  Right.setVelocity(30, velocityUnits::pct);

  //Brings Mobile Goal Lift down
   mogo.spinFor(forward, 450, degrees, false);
   wait(1.5, seconds);

  //Moves Forward
  Left.spinFor(reverse, 385*onedegree*2.2+700, degrees, false);
  Right.spinFor(reverse, 385*onedegree*2.2+700, degrees, false);

  
 
  wait(2.5, seconds);

  //Brings Mobile Goal Lift Up
  mogo.spinFor(reverse, 450, degrees, false);
  wait(2, seconds);
  Left.setVelocity(70, velocityUnits::pct); 
  Right.setVelocity(70, velocityUnits::pct);


  //Moves backward with mobile goal
  Left.spinFor(forward, 360*onedegree*2.3+2200, degrees, false);
  Right.spinFor(forward, 360*onedegree*2.3+2200, degrees, false);
  

  


  

  

  
  //set_tank(127, 127);
  wait(800, msec);
  setTankMethod(0, 0);  

  did_auto_finish = true;
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
 
void usercontrol(void) {
  // User control code here, inside the loop
  // Parameters for user control
  bool tilter_up;
  int tilter_lock = 0;
  int tilter_timer = 0;
  bool down = false;

  bool mogo_up  = true;
  int mogo_lock = 0;
  bool is_up = true;

  bool buttonA = true;
  bool buttonX = true;
  bool buttonY = false;
  bool buttonB = false;

  int up_lock    = 0;
  int down_lock  = 0;
  int lift_state = 0;
  int lift_speed = speedUp;
  bool upLift = false;
  bool downLift = true;
  

  //if (did_auto_finish)
    //tilter_up = false;

  //Checks if the reset was succesful, if not, it will reset it one final time
  while(resetSuccess == false){
    wait(10, msec);
  }

  //activates coast braking system
  COAST();
  
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.



    ///
    // Joysticks
    ///



    // Sets the Left motor group drive the amount pulled up by the axis 3
   int driveLeft = Controller1.Axis3.value() + Controller1.Axis1.value();
   int driveRight = Controller1.Axis3.value() - Controller1.Axis1.value();
   setTankMethod(driveLeft, driveRight);
    
    

    if(Controller1.ButtonA.pressing() && buttonA){

      Brain.Screen.print("ButtonA Pressed");
      set_lift_position(600, 100);
      buttonA = false;
      buttonB = true;
      upLift = true;
      downLift = false;

    }
    
    if(Controller1.ButtonX.pressing() && buttonX){

      Brain.Screen.print("ButtonX Pressed");
      
      
      //dig1.set(true);
      DigitalOutH.set(true);
      buttonX = false;
      buttonY = true;

    }
    if(Controller1.ButtonY.pressing() && buttonY){

      Brain.Screen.print("ButtonY Pressed");
      
      
      //dig1.set(true);
      DigitalOutH.set(false);
      buttonY = false;
      buttonX = true;

    }

    if(Controller1.ButtonB.pressing() && buttonB){
      Brain.Screen.print("ButtonB Pressed");
      set_lift_position(0, 100);
      buttonA = true;
      buttonB = false;
      upLift = false;
      downLift = true;
    }

    ///
    // Tilter
    // The tilter mechanic has 2 set positions, as a result, there will be 2 positions
    ///
    // Once button is pressed, sets the booleans accordingly
    if (Controller1.ButtonL2.pressing() && tilter_lock==0) {
      if (down)
        tilter_up = false;
      else
        tilter_up = !tilter_up;
      tilter_lock = 1;
      down=false;

    
    } 
    else if (Controller1.ButtonL2.pressing()) {
      tilter_timer+=delayTime;
      if (tilter_timer>300) {
        down=true;
      }
    }
    else if (!Controller1.ButtonL2.pressing()) {
      tilter_lock = 0;
      tilter_timer=0;
    }

    // Puts the tilter position to its releative position based on its inital position
    if (down)
      putTilterPosition(-940, 100);
    else if (tilter_up) 
      putTilterPosition(tilIn, 100);
    else  //tilter_up == false
      putTilterPosition(tilOut, 100);
    


    ///
    // Mobile goal
    //  - L1 interchanges the current back mobile goal position, down or up
    ///

    // Changes boolean relative to where the mobile goal lift currently is positioned
    if (Controller1.ButtonL1.pressing() && mogo_lock==0) {
      mogo_up = !mogo_up;
      mogo_lock = 1;
      // mogo_lock allows the code not to go into an infinite loop or cause the motors to spam
    } 
    else if (!Controller1.ButtonL1.pressing()) {
      mogo_lock = 0;
    }

    // Turns the motor to a relative position based on the current position of the motor

    if (mogo_up) {
      if (mogo.rotation(deg)<150) {
        if (mogo.velocity(pct)==0) {
          is_up = true;
          mogoSet(0);
        }
        else {
          mogoSet(is_up?0:-30);
        }
      }
      else {
        is_up = false;
        mogoSet(-127);
      }
    }
    else {
      if (mogo.rotation(deg)>mobileGoal-100) {
        if (mogo.velocity(pct)==0) 
          mogoSet(0);
        else 
          mogoSet(30);
      }
      else {
        mogoSet(127);
      }
    }



    ///
    // Lift
    //  - R1 Brings the lift up, R2 Brings the lift down
    ///
    
    // R1 brings the lift up
    if (Controller1.ButtonR1.pressing() && up_lock==0) {
      // If the ladder is at the top, bring it down
      if (lift_state==positions-1)  {
        lift_state = 0;
        lift_speed = speedDown;
      }
    
      else {
        lift_state++;
        lift_speed = speedUp;
      }
      up_lock = 1;
    }
    else if (!Controller1.ButtonR1.pressing()) {
      up_lock = 0;
    }

    // R2 Brings the ladder down
    if (Controller1.ButtonR2.pressing() && down_lock==0) {
      // Brings the lift up if it is currently at a lower position
      if (lift_state==0) {
        lift_state = positions-1;
        lift_speed = speedUp;
      }
      // Otherwise, bring it down
      else {
        lift_state--;
        lift_speed = speedDown;
      }
      down_lock = 1;
    }
    else if (!Controller1.ButtonR2.pressing()) {
      down_lock = 0;
    }
    // Sets lift position back to its height specified at the top of the code
    

    if(upLift){
      set_lift_position(100, 100);
    }
    else if(downLift && lift_state == 0){
      set_lift_position(0, 100);
    }
    else{
      set_lift_position(liftHeight[lift_state], 100);
    }
    
    //ExpansionLimitSafeGaurd
    //if(mogo_up == false && lift == true &&  Controller1.ButtonR2.pressing()){
    //    mogo_up = true;

    //}




    wait(delayTime, msec); // Tries to prevent the CPU from overheating
  }
}



//
// Main will set up the competition functions and callbacks.
//
int main() {

//Brain.Screen.print("Daddy ;)");
  //Brain.Screen.drawRectangle(1,1,50,20,color::red);
    //Brain.Screen.drawCircle(50,50,20,color::red);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
