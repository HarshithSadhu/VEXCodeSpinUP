/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// leftDrive            motor_group   9, 8            
// rightDrive           motor_group   6, 7            
// Controller1          controller                    
// ringIntake           motor         11              
// diskIntake           motor         18              
// shooter              motor_group   16, 17          
// DigitalOutA          digital_out   A               
// Vision20             vision        20              
// Distance19           distance      19              
// Controller2          controller                    
// DigitalOutC          digital_out   C               
// Vision21             vision        21              
// Inertial             inertial      15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"


using namespace vex;

// A global instance of competition
int up_lock    = 0;
  int down_lock  = 0;
  int lift_state = 0;
  const int speedUp   = 100;
  int lift_speed = speedUp;
  bool upLift = false;
  bool downLift = true;
  bool isLooking = true;
  
const int speedDown = 100;
int liftPosition;
const int positions = 3; // Number of different height settings

competition Competition;
bool isManual;
int speed = 100;
int speed2 = 100;
bool rightBtn = true;
bool leftBtn = true;
const int liftHeight[positions] = {0, 60, 100}; // Positions for the lift, motor spinning degrees

// define your global instances of motors and other devices here
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

double kP = 0.01;
//double kI = 0.0;
//double kD = 0.01;

double tkP = 0.01;
//double tkI = 0.0;
double tkD = 0.01;

int desiredValue = 0;
int desiredTurnValue = 0;

int error;
int prevError = 0;
int derivative;
int totalError = 0;

int errorT;
int prevErrorT = 0;
int derivativeT;
int totalErrorT = 0;

bool reset = false;
bool reset2 = false;

bool enableDrivePID = true;
bool enableTurnPID = true;

turnType turnD;

double kPAngle = 0.25;
//double kIAngle = 0;
//double kDAngle = 0.1;
double maxSpeed = 100;


void rotatePID(double angleTurn, turnType turnDir) {
  double error2 = 0;
  double prevError = 0;
  double derivative = 0;
  double integral = 0;

  while( fabs(Inertial.rotation() - angleTurn) > 0.5) {
    error2 = angleTurn - Inertial.rotation(rotationUnits::deg);
    derivative = error2 - prevError;
    prevError = error2;

    if(fabs(error2) < 5 && error2 != 0) {
      integral += error2;
    }
    else {
      integral = 0;
    } 

    double powerDrive = error2*kPAngle;
    //+ derivative*kDAngle + integral*kIAngle

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
      rightDrive.spin(reverse, powerDrive, voltageUnits::volt);
      leftDrive.spin(forward, powerDrive, voltageUnits::volt);
    }

    if(turnDir == left){
      rightDrive.spin(forward, powerDrive, voltageUnits::volt);
      leftDrive.spin(reverse, powerDrive, voltageUnits::volt);
    }
    
    
    this_thread::sleep_for(15);
  }
  
  // turning = false;
  // goalVoltage = 0;
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}

int pistonShoot() {
    
  
  bool pistonShootState = false;
  bool pistonShootLast = false;
    while (true) {

      if (Controller1.ButtonB.pressing () == true && !pistonShootLast) {
        pistonShootState = !pistonShootState;
        pistonShootLast = true;
      }

      else if (!Controller1.ButtonB.pressing () == true) {
        pistonShootLast = false;
      }

      if (pistonShootState) {
        DigitalOutE.set(true);
      }

      else {
        DigitalOutE.set(false);
      }
      task::sleep (50);
    }
    
    return 1; //Tasks must return 1 for some reason.
  }



int turnPID(){
  
  while(enableTurnPID){

    //enableDrivePID = false;
    if(reset2){
      reset2 = false;


      leftDrive.setPosition(0, degrees);
      rightDrive.setPosition(0, degrees);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////

    // turning

    
    int avgPosT = fabs((leftDrive.position(degrees) - rightDrive.position(degrees)));//(leftMotor1Position + leftMotor2Position) - (rightMotor1Position + rightMotor2Position);


    /*if(turnD == right){
      avgPosT = (leftDrive.position(degrees) - rightDrive.position(degrees));//(leftMotor1Position + leftMotor2Position) - (rightMotor1Position + rightMotor2Position);
    }
    if(turnD == left){
      avgPosT = (leftDrive.position(degrees) + rightDrive.position(degrees));//(leftMotor1Position + leftMotor2Position) - (rightMotor1Position + rightMotor2Position);
    }*/

    errorT = desiredTurnValue - avgPosT;

    derivativeT = errorT - prevErrorT;

    //totalErrorT += errorT;

    double motorPowerTurn = (errorT * tkP);// + derivativeT * tkD);// + totalErrorT * tkI); /// 12.0;

    if (motorPowerTurn <= 10){
      motorPowerTurn = 0;
      leftDrive.setStopping(hold);
      rightDrive.setStopping(hold);
      leftDrive.stop();
      rightDrive.stop();
      enableTurnPID = false;
      return 1;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    if(turnD == left){
      rightDrive.spin(reverse, motorPowerTurn, voltageUnits::volt);
      leftDrive.spin(forward, motorPowerTurn, voltageUnits::volt);
    }

    if(turnD == right){
      rightDrive.spin(forward, motorPowerTurn, voltageUnits::volt);
      leftDrive.spin(reverse, motorPowerTurn, voltageUnits::volt);
    }

    prevErrorT = errorT;

    

    vex::task::sleep(20);
  }
  
  return 1;
}


int drivePID(){
  
  while(enableDrivePID){

    //enableDrivePID = false;
    if(reset){
      reset = false;


      leftDrive.setPosition(0, degrees);
      rightDrive.setPosition(0, degrees);
    }

    /*int leftMotor1Position = leftMotor1.position(degrees);
    int leftMotor2Position = leftMotor2.position(degrees);
    int rightMotor1Position = rightMotor1.position(degrees);
    int rightMotor2Position = rightMotor2.position(degrees);

    // lateral movement

    int avgPos = (leftMotor1Position + leftMotor2Position + rightMotor1Position + rightMotor2Position) / 4;*/

    int avgPos = (leftDrive.position(degrees) + rightDrive.position(degrees));

    error = desiredValue - avgPos;

    derivative = error - prevError;

    //totalError += error;

    double motorPower = (error * kP); //+ derivative * kD);// + totalError *kI); /// 12.0;

// + totalErrorT * tkI); /// 12.0;
    if (motorPower <= 10){
      motorPower = 0;
      leftDrive.setStopping(hold);
      rightDrive.setStopping(hold);
      leftDrive.stop();
      rightDrive.stop();
      enableDrivePID = false;
      return 1;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    leftDrive.spin(forward, motorPower, velocityUnits::pct);
    rightDrive.spin(forward, motorPower, velocityUnits::pct);

    prevError = error;
    prevErrorT = errorT;

    vex::task::sleep(20);
  }
  
  return 1;
}

void setDrive(int x){
  reset = true;
  enableDrivePID = true;
  desiredValue = x;
  drivePID();
}

void setTurn(int y, turnType turnDir){
  reset2 = true;
  enableTurnPID = true;
  desiredTurnValue = y;
  turnD = turnDir;
  turnPID();
}

void pre_auton(void) {

  //DigitalOutE.set(low);
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  DigitalOutC.set(false);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void driveForward(int degreesReal, int speed){
    leftDrive.setVelocity(speed, velocityUnits::pct); 
    rightDrive.setVelocity(speed, velocityUnits::pct);  

    leftDrive.spinFor(forward, degreesReal, degrees, false);
    rightDrive.spinFor(forward, degreesReal, degrees);
    
}


void driveReverse(int degreesReal, int speed){
    leftDrive.setVelocity(speed, velocityUnits::pct); 
    rightDrive.setVelocity(speed, velocityUnits::pct);  

    leftDrive.spinFor(reverse, degreesReal, degrees, false);
    rightDrive.spinFor(reverse, degreesReal, degrees);
    
}





void turnRight (int degreesReal, int powerReal){

  while(Inertial.rotation() < degreesReal){
    rightDrive.spin(reverse, powerReal, voltageUnits::volt);
    leftDrive.spin(forward, powerReal, voltageUnits::volt);
  }



}

void turnLeft (int degreesReal, int powerReal){

  while(Inertial.rotation() > degreesReal){
    rightDrive.spin(forward, powerReal, voltageUnits::volt);
    leftDrive.spin(reverse, powerReal, voltageUnits::volt);
  }



}



void shoot(){


  
    DigitalOutA.set(true);
    wait(0.2, seconds);
    DigitalOutA.set(false);
    wait(0.2, seconds);
    

}

void spinRoller(){

  while(isLooking){
    Vision20.takeSnapshot(Vision20__SIGBLUE);

    if(Vision20.largestObject.exists){
    
      ringIntake.spin(forward, 25, percent);

    }

    else{
      ringIntake.setStopping(hold);
      ringIntake.stop();
      isLooking = false;
    }

  }


}


void autonomous(void) {

  Inertial.setHeading(1, degrees);
  
  

  

  


  
  //keyword
  shooter.setVelocity(100, pct);
  ringIntake.setVelocity(30, percent);
  leftDrive.setVelocity(15, pct);
  rightDrive.setVelocity(15, pct);






  shooter.spin(forward);

  leftDrive.spinFor(reverse, 100, degrees, false);
  rightDrive.spinFor(reverse, 100, degrees);

  spinRoller();

  



  

  //wait(4, seconds);
  //shoot();
  //shoot();

  leftDrive.spinFor(forward, 250, degrees, false);
  rightDrive.spinFor(forward, 250, degrees);

  wait(7, seconds);

  shoot();
  shoot();


  /*

  Inertial.setHeading(1, degrees);

  //mewow
  //Turns towards Middle
  leftDrive.spin(reverse, 15, percent);
  rightDrive.spin(forward, 15, percent);
  while(Inertial.heading() < 30)
  {

    Brain.Screen.clearLine();
    Brain.Screen.print(Inertial.heading());
    
  }

  leftDrive.setStopping(hold);
  rightDrive.setStopping(hold);
  leftDrive.stop();
  rightDrive.stop();

  setDrive(10000);

  Brain.Screen.print(Inertial.heading());
  //wait(5, seconds);

  //Rotates towards the blue goal
  leftDrive.spin(forward, 15, percent);
  rightDrive.spin(reverse, 15, percent);
  while(Inertial.heading() > 325 || Inertial.heading() < 324)
  {

    Brain.Screen.clearLine();
    Brain.Screen.print(Inertial.heading());
    
  }

  leftDrive.setStopping(hold);
  rightDrive.setStopping(hold);
  leftDrive.stop();
  rightDrive.stop();

  shoot();
  shoot();


  //Rotates towards the other roller
  leftDrive.spin(reverse, 20, percent);
  rightDrive.spin(forward, 20, percent);
  while(Inertial.heading() < 25 || Inertial.heading() > 26)
  {

    Brain.Screen.clearLine();
    Brain.Screen.print(Inertial.heading());
    
  }

  leftDrive.setStopping(hold);
  rightDrive.setStopping(hold);
  leftDrive.stop();
  rightDrive.stop();

  setDrive(9000);

  //Rotates towards the final roller
  leftDrive.spin(forward, 20, percent);
  rightDrive.spin(reverse, 20, percent);
  while(Inertial.heading() > 270 || Inertial.heading() < 269)
  {

    Brain.Screen.clearLine();
    Brain.Screen.print(Inertial.heading());
    
  }

  leftDrive.setStopping(hold);
  rightDrive.setStopping(hold);
  leftDrive.stop();
  rightDrive.stop();

  leftDrive.setVelocity(20, percent);
  rightDrive.setVelocity(20, percent);


  leftDrive.spinFor(reverse, 350, degrees, false);
  rightDrive.spinFor(reverse, 350, degrees);

  //ringIntake.setVelocity(30, percent);
  isLooking = true;
  spinRoller();


  wait(60, seconds);

  setTurn(2550, left);

  shoot();
  shoot();

  setTurn(2500, left);

  



  
  

  /*setTurn(1750, right);
  wait(2, seconds);
  setTurn(1750, left);
  wait(2, seconds);
  setDrive(10000);*/







/*
    //Turning Parameters
    //First Parameter: Degrees
    //Second Parameter: Speed
    Inertial.calibrate(2000);
  this_thread::sleep_for(2000);

    Inertial.setHeading(0, degrees);

    //Starts shooter at the start
    shooter.setVelocity(100, percent);
    shooter.spin(forward);

    Brain.Screen.print(Inertial.heading());

    //Goes to roller spins and goes out
    driveReverse(100, 50);
    diskIntake.spinFor(reverse, 500, degrees);
    driveForward(400, 30);


    //turns left to align to blue goal
    while(Inertial.heading() < 355 && Inertial.heading() > 340){
    
    Brain.Screen.print(Inertial.heading());
    leftDrive.spin(reverse, 1, voltageUnits::volt);
    rightDrive.spin(forward, 1, voltageUnits::volt);
  }

    //Shoots 3 disks
    shoot();
    shoot();
    shoot();

    //Positions to drive backward
    while(Inertial.heading() < 330 && Inertial.heading() > 320){
    
    Brain.Screen.print(Inertial.heading());
    leftDrive.spin(reverse, 1, voltageUnits::volt);
    rightDrive.spin(forward, 1, voltageUnits::volt);
  }

  //drives backward for a lot
  driveReverse(900, 50);


  //Turns again to face roller
  while(Inertial.heading() < 40 && Inertial.heading() > 30){
    
    Brain.Screen.print(Inertial.heading());
    leftDrive.spin(forward, 1, voltageUnits::volt);
    rightDrive.spin(reverse, 1, voltageUnits::volt);
  }

  //drives to tuch the roller
  driveReverse(300, 50);


  //Spins disk
  diskIntake.spinFor(reverse, 500, degrees);


  wait(60, seconds);



    //while(true){
      Brain.Screen.clearLine();
      Brain.Screen.print(Inertial.heading());
   // }



    






    shoot();
    shoot();
    shoot();
    shooter.stop();

    turnLeft(-130, 5);
    
    driveReverse(4000, 50);

    
    


    
    leftDrive.setVelocity(30, velocityUnits::pct); 
    rightDrive.setVelocity(30, velocityUnits::pct); 


    leftDrive.spinFor(reverse, 1700, degrees, false);
    rightDrive.spinFor(reverse, 1700, degrees);

    leftDrive.spinFor(reverse, 700, degrees, false);
    rightDrive.spinFor(forward, 700, degrees);

    leftDrive.setVelocity(20, velocityUnits::pct); 
    rightDrive.setVelocity(20, velocityUnits::pct); 
    leftDrive.spinFor(reverse, 20000, degrees, false);
    rightDrive.spinFor(reverse, 20000, degrees, false);

    leftDrive.setVelocity(100, velocityUnits::pct); 
    diskIntake.setVelocity(100, velocityUnits::pct); 
    diskIntake.spin(reverse);
    wait(0.5, seconds);
    diskIntake.stop();

    leftDrive.setVelocity(20, velocityUnits::pct); 
    rightDrive.setVelocity(20, velocityUnits::pct); 
    leftDrive.spinFor(forward, 400, degrees, false);
    rightDrive.spinFor(forward, 400, degrees);
    leftDrive.spinFor(forward, 340, degrees, false);
    rightDrive.spinFor(reverse, 340, degrees);

    leftDrive.setVelocity(60, velocityUnits::pct); 
    rightDrive.setVelocity(60, velocityUnits::pct); 

    shooter.setVelocity(100, velocityUnits::pct);
    shooter.spin(forward);

    leftDrive.spinFor(forward, 4300, degrees, false);
    rightDrive.spinFor(forward, 4300, degrees);

    leftDrive.spinFor(reverse, 770, degrees, false);
    rightDrive.spinFor(forward, 770, degrees);

    
    
    wait(2, seconds);
    DigitalOutA.set(true);
    wait(0.2, seconds);
    DigitalOutA.set(false);
    wait(2, seconds);
    DigitalOutA.set(true);
  */

}

int pistonMech() {
  bool pistonMechState = false;
  bool pistonMechLast = false;
    while (true) {

      if (Controller1.ButtonL1.pressing()) {
        DigitalOutA.set(true);
        wait(0.1, seconds);
        DigitalOutA.set(false);
      }

      /*
      if (Controller1.ButtonR1.pressing () == true && !pistonMechLast) {
        pistonMechState = !pistonMechState;
        pistonMechLast = true;
        
      }

      else if (!Controller1.ButtonR1.pressing () == true) {
        pistonMechLast = false;
      }

      if (pistonMechState) {
        DigitalOutA.set(false);
      }

      else {
        DigitalOutA.set(true);
        Brain.Screen.print("Test");
      }


      */

      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }



int manualOverride() {
  bool manualOverrideState = false;
  bool manualOverrideLast = false;
    while (true) {

      if (Controller1.ButtonDown.pressing () == true && !manualOverrideLast) {
        manualOverrideState = !manualOverrideState;
        manualOverrideLast = true;
      }

      else if (!Controller1.ButtonDown.pressing () == true) {
        manualOverrideLast = false;
      }

      if (manualOverrideState) {
        isManual = true;
      }

      else {
        isManual = false;
      }
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }







int diskIntakeMech() {
  bool diskIntakeMechState = false;
  bool diskIntakeMechLast = false;
    while (true) {

      if (Controller1.ButtonX.pressing () == true && !diskIntakeMechLast) {
        diskIntakeMechState = !diskIntakeMechState;
        diskIntakeMechLast = true;
      }

      else if (!Controller1.ButtonX.pressing () == true) {
        diskIntakeMechLast = false;
      }

      if (diskIntakeMechState && isManual) {
          diskIntake.spin(forward, 100, pct); 
          
              
      }

      else if (diskIntakeMechState) {
        
       diskIntake.spin(reverse, 100, pct);
       

      }



      else {
        diskIntake.stop();
        
      }
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }



  int endGameMech() {
    
  
  bool endGameMechState = false;
  bool endGameMechLast = false;
    while (true) {

      if (Controller1.ButtonB.pressing () == true && !endGameMechLast) {
        endGameMechState = !endGameMechState;
        endGameMechLast = true;
      }

      else if (!Controller1.ButtonB.pressing () == true) {
        endGameMechLast = false;
      }

      if (endGameMechState) {
        diskIntake.spin(forward, 100, pct);
      }

      else {
        diskIntake.stop();
      }
      task::sleep (50);
    }
    
    return 1; //Tasks must return 1 for some reason.
  }


  int increaseVelocity() {
  bool increaseVelocityState = false;
  bool increaseVelocityLast = false;
    while (true) {

      if (Controller2.ButtonRight.pressing () == true && !increaseVelocityLast) {
        increaseVelocityState = !increaseVelocityState;
        increaseVelocityLast = true;
      }

      else if (!Controller2.ButtonRight.pressing () == true) {
        increaseVelocityLast = false;
      }

      if (increaseVelocityState) {
        speed += 10;

        Controller2.Screen.print("Velocity: " + speed);
        
      }

      else {
        ringIntake.stop();
      }
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }

  int decreaseVelocity() {
  bool decreaseVelocityState = false;
  bool decreaseVelocityLast = false;
    while (true) {

      if (Controller1.ButtonLeft.pressing () == true && !decreaseVelocityLast) {
        decreaseVelocityState = !decreaseVelocityState;
        decreaseVelocityLast = true;
      }

      else if (!Controller1.ButtonLeft.pressing () == true) {
        decreaseVelocityLast = false;
      }

      if (decreaseVelocityState) {
        speed -= 10;
        Controller2.Screen.clearLine();
        Controller2.Screen.print("Velocity: " + speed);
        
      }

      else {
        ringIntake.stop();
      }
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }







int ringMech() {
  bool ringMechState = false;
  bool ringMechLast = false;
    while (true) {

      if (Controller1.ButtonL2.pressing () == true && !ringMechLast) {
        ringMechState = !ringMechState;
        ringMechLast = true;
      }

      else if (!Controller1.ButtonL2.pressing () == true) {
        ringMechLast = false;
      }

      if (ringMechState) {
        ringIntake.spin(reverse, 25, pct);
      }

      else {
        ringIntake.stop();
      }
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }

int shooterMech() {
  bool shooterMechState = false;
  bool shooterMechLast = false;
    while (true) {

      if (Controller1.ButtonR2.pressing () == true && !shooterMechLast) {
        shooterMechState = !shooterMechState;
        shooterMechLast = true;
      }

      else if (!Controller1.ButtonR2.pressing () == true) {
        shooterMechLast = false;
      }

      if (shooterMechState) {
          shooter.spin(forward, 100, pct);      
      }

      
      else{

          shooter.spin(forward, 60, pct);
      }


    
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }



const int scaleVal = 120;
void setTankMethod(int l, int r) {
  leftDrive.spin(fwd, l*scaleVal, voltageUnits::mV);

  rightDrive.spin(fwd, r*scaleVal, voltageUnits::mV);

}



void percentChangeDown(void) {
 if (speed2 > 20) {
   speed2 -= 20;
 }
 wait(20,msec);
}
void percentChangeUp(void){
 if (speed2 < 100) {
   speed2 += 20;
 }
 wait(20,msec);
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

void set_lift_position  (int pos, int speed) { 

  
  shooter.spin(forward, pos, pct);
  //tilter.startRotateTo(200, rotationUnits::deg, 100, velocityUnits::pct);
  

  //lift.spinFor(pos, rotationUnits::deg, speed, velocityUnits::pct); 

  //lift.spinFor(pos, rotationUnits::deg, speed, velocityUnits::pct);
  //lift.setVelocity(70, velocityUnits::pct); 
  //lift.spinFor(forward, pos, degrees, false);
  //lift.rotateFor(directionType dir, double rotation, rotationUnits units)
  
  
  
  }


int backClamp() {
  bool backClampState = false;
  bool backClampLast = false;
    while (true) {

      if (Controller1.ButtonA.pressing () == true && !backClampLast) {
        backClampState = !backClampState;
        backClampLast = true;
      }

      else if (!Controller1.ButtonA.pressing () == true) {
        backClampLast = false;
      }

      if (backClampState) {
        DigitalOutC.set(false);
        Brain.Screen.print("test");
      }

      else {
        DigitalOutC.set(true);
      }
      task::sleep (50);
    }
    return 1; //Tasks must return 1 for some reason.
  }



void usercontrol(void) {

  vex::task t1(ringMech);
  vex::task t2(diskIntakeMech);
  vex::task t3(shooterMech);
  vex::task t4(pistonMech);
  vex::task t8(backClamp);
  vex::task t9(pistonShoot);
  //vex::task t9(endGameMech);

  //vex::task t5(ringMech2);
  //vex::task t5(increaseVelocity);
  //vex::task t6(decreaseVelocity);
  // User control code here, inside the loop
  while (1) {


    if(Controller2.ButtonDown.pressing() || Controller1.ButtonDown.pressing()){
      isManual = true;
      Brain.Screen.clearLine();
      Brain.Screen.print("IsManual");
    }

    if(Controller2.ButtonUp.pressing() || Controller1.ButtonUp.pressing()){
      isManual = false;
      Brain.Screen.clearLine();
      Brain.Screen.print("IsNotManual");
      
    }

      // change percent up when clicked up or down
   /*if (Controller2.ButtonLeft.pressing() || Controller1.ButtonLeft.pressing()) {
     percentChangeDown();
   }
   if (Controller2.ButtonRight.pressing() || Controller1.ButtonRight.pressing()) {
     percentChangeUp();
   }
   */
   //Controller1.Screen.clearLine(3);
   //displaying percentage on screen
   //Controller1.Screen.setCursor(3,0);
   //Controller1.Screen.print("Percent: %.0d", speed2);
   //Controller2.Screen.clearLine(3);
   //Controller2.Screen.setCursor(3,0);
   //Controller2.Screen.print("Percent: %.0d", speed2);


  /*

    if(Controller2.ButtonDown.pressing() || Controller1.ButtonDown.pressing()){
      isManual = true;
      Brain.Screen.clearLine();
      Brain.Screen.print("IsManual");
    }

    if(Controller2.ButtonUp.pressing() || Controller1.ButtonUp.pressing()){
      isManual = false;
      Brain.Screen.clearLine();
      Brain.Screen.print("IsNotManual");
      
    }

    /*
    if(Controller2.ButtonRight.pressing() && rightBtn == true){
      leftBtn = true;
      speed += 10;
      Brain.Screen.clearLine();
      Controller2.Screen.print("Velocity: " + speed);
      rightBtn = false;
    }

    if(Controller2.ButtonLeft.pressing() && leftBtn == true){
      rightBtn = true;
      speed -= 10;
      Controller2.Screen.clearLine();
      Controller2.Screen.print("Velocity: " + speed);
      leftBtn = false;
    }
    */
    


    



    


    /*
    if(Controller1.ButtonB.pressing()){
      Brain.Screen.clearLine();
      Brain.Screen.print("Distance in Inches: ");
      Brain.Screen.print("%.2f", Distance19.objectDistance(inches));
      double distance = Distance19.objectDistance(inches);
      distanceScale = (distance/100) * 100;
      shooter.setVelocity(distanceScale, velocityUnits::pct); 

      

    }
    */

    
    


    

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





      int l_drive = Controller1.Axis3.value() - Controller1.Axis1.value();
   int r_drive = Controller1.Axis3.value() + Controller1.Axis1.value();
   setTankMethod(l_drive, r_drive);

   


    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
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
