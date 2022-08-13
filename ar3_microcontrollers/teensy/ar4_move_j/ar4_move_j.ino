//VERSION 1.1

/*  AR4 - Stepper motor robot control software
    Copyright (c) 2021, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

*/


// VERSION LOG
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5


#include <math.h>
#include <avr/pgmspace.h>
#include <Encoder.h>
#include <AccelStepper.h>

const char* VERSION = "0.0.1";

String inData;
String function;
int WayPtDel;
volatile byte state = LOW;

const int debugg = 0;

const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int TRstepPin = 12;
const int TRdirPin = 13;

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;


const int Input32 = 32;
const int Input33 = 33;
const int Input34 = 34;
const int Input35 = 35;
const int Input36 = 36;

const int Output37 = 37;
const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;


//set encoder multiplier
const float J1encMult = 10;
const float J2encMult = 10;
const float J3encMult = 10;
const float J4encMult = 10;
const float J5encMult = 5;
const float J6encMult = 10;

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(17, 16);
Encoder J3encPos(19, 18);
Encoder J4encPos(20, 21);
Encoder J5encPos(23, 22);
Encoder J6encPos(24, 25);


// GLOBAL VARS //

//define axis limits in degrees
float J1axisLimPos = 170;
float J1axisLimNeg = 170;
float J2axisLimPos = 90;
float J2axisLimNeg = 42;
float J3axisLimPos = 52;
float J3axisLimNeg = 89;
float J4axisLimPos = 165;
float J4axisLimNeg = 165;
float J5axisLimPos = 105;
float J5axisLimNeg = 105;
float J6axisLimPos = 155;
float J6axisLimNeg = 155;
float TRaxisLimPos = 3450;
float TRaxisLimNeg = 0;

//define total axis travel
float J1axisLim = J1axisLimPos + J1axisLimNeg;
float J2axisLim = J2axisLimPos + J2axisLimNeg;
float J3axisLim = J3axisLimPos + J3axisLimNeg;
float J4axisLim = J4axisLimPos + J4axisLimNeg;
float J5axisLim = J5axisLimPos + J5axisLimNeg;
float J6axisLim = J6axisLimPos + J6axisLimNeg;
float TRaxisLim = TRaxisLimPos + TRaxisLimNeg;

//motor steps per degree
float J1StepDeg = 44.44444444;
float J2StepDeg = 55.55555556;
float J3StepDeg = 55.55555556;
float J4StepDeg = 42.72664356;
float J5StepDeg = 21.86024888;
float J6StepDeg = 22.22222222;
float TRStepDeg = 14.28571429;

//steps full movement of each axis
int J1StepLim = J1axisLim * J1StepDeg;
int J2StepLim = J2axisLim * J2StepDeg;
int J3StepLim = J3axisLim * J3StepDeg;
int J4StepLim = J4axisLim * J4StepDeg;
int J5StepLim = J5axisLim * J5StepDeg;
int J6StepLim = J6axisLim * J6StepDeg;
int TRStepLim = TRaxisLim * TRStepDeg;

//step and axis zero
int J1zeroStep = J1axisLimNeg * J1StepDeg;
int J2zeroStep = J2axisLimNeg * J2StepDeg;
int J3zeroStep = J3axisLimNeg * J3StepDeg;
int J4zeroStep = J4axisLimNeg * J4StepDeg;
int J5zeroStep = J5axisLimNeg * J5StepDeg;
int J6zeroStep = J6axisLimNeg * J6StepDeg;

//start master step count at Jzerostep
int J1StepM = J1zeroStep;
int J2StepM = J2zeroStep;
int J3StepM = J3zeroStep;
int J4StepM = J4zeroStep;
int J5StepM = J5zeroStep;
int J6StepM = J6zeroStep;
int TRStepM = TRaxisLimNeg * TRStepDeg;


//degrees from limit switch to offset calibration
float J1calBaseOff = -1;
float J2calBaseOff = 2;
float J3calBaseOff = 4.1;
float J4calBaseOff = -1.5;
float J5calBaseOff = 3;
float J6calBaseOff = -7;
float TRcalBaseOff = 0;

//reset collision indicators
int J1collisionTrue = 0;
int J2collisionTrue = 0;
int J3collisionTrue = 0;
int J4collisionTrue = 0;
int J5collisionTrue = 0;
int J6collisionTrue = 0;
int TotalCollision = 0;
int KinematicError = 0;

float axis7length;
float axis7rot;
float axis7steps;

float lineDist;

String WristCon;
int Quadrant;

// Additional Params for TRAJ Moves


// const int J1rotdir = 1;
const int J1rotdir = 0; 
const int J2rotdir = 1;
const int J3rotdir = 1;
const int J4rotdir = 0;
const int J5rotdir = 1;
const int J6rotdir = 1;
const int ROT_DIRS[] = { J1rotdir, J2rotdir, J3rotdir, J4rotdir, J5rotdir, J6rotdir };

// approx encoder counts at rest position, 0 degree joint angle
//const int REST_ENC_POSITIONS[] = { 38681, 11264, 0, 36652, 5839, 15671 };
//const int REST_ENC_POSITIONS[] = { 38681, 11264, 0, 36652, 5839, 15671 };
// LimNegDeg * MStep per Deg * EncStep per MStep //ok
const int REST_ENC_POSITIONS[] = {J1axisLimNeg*J1StepDeg*J1encMult, J2axisLimNeg*J2StepDeg*J2encMult, J3axisLimNeg*J3StepDeg*J3encMult, J4axisLimNeg*J4StepDeg*J4encMult, J5axisLimNeg*J5StepDeg*J5encMult, J6axisLimNeg*J6StepDeg*J6encMult}; //ok

const int STEP_PINS[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin }; //ok
const int DIR_PINS[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin }; //ok

Encoder JOINT_ENCODERS[] = { J1encPos, J2encPos, J3encPos, J4encPos, J5encPos, J6encPos }; //ok
int ENC_DIR[] = {1, 1, 1, 1, 1, 1 }; // +1 if encoder direction matches motor direction //ok

const int CAL_PINS[] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin }; //ok
const int LIMIT_SWITCH_HIGH[] = { 1, 1, 1, 1, 1, 1 }; // to account for both NC and NO limit switches
const int CAL_DIR[] = {1, -1, 1, -1, -1, 1 }; // joint rotation direction to limit switch
const int CAL_SPEED = 600; // motor steps per second
const int CAL_SPEED_MULT[] = { 1, 1, 1, 2, 1, 1 }; // multiplier to account for motor steps/rev

// num of steps in range of motion of joint, may vary depending on your limit switch
// defaults 77363, 36854, 40878, 71967, 23347, 32358
// const int ENC_RANGE_STEPS[] = {77363, 36854, 40878, 71967, 23347, 32358};
const int ENC_RANGE_STEPS[] = {J1axisLim*J1StepDeg*J1encMult, J2axisLim*J2StepDeg*J2encMult, J3axisLim*J3StepDeg*J3encMult, J4axisLim*J4StepDeg*J4encMult, J5axisLim*J5StepDeg*J5encMult, J6axisLim*J6StepDeg*J6encMult}; //ok
const float ENC_MULT[] = {J1encMult, J2encMult, J3encMult, J4encMult, J5encMult, J6encMult};

// motor and encoder steps per revolution
// this is based on teh DIP swtich config of the motor drivers
const int MOTOR_STEPS_PER_REV[] = { 400, 400, 400, 400, 800, 400 };

const int NUM_JOINTS = 6;
const float ENC_STEPS_PER_DEG[] = {J1StepDeg*J1encMult, J2StepDeg*J2encMult, J3StepDeg*J3encMult, J4StepDeg*J4encMult, J5StepDeg*J5encMult, J6StepDeg*J6encMult};

float JOINT_MAX_SPEED[] = { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 }; // deg/s
float JOINT_MAX_ACCEL[] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 }; // deg/s^2
int MOTOR_MAX_SPEED[] = { 1500, 1500, 1500, 2000, 1500, 1500 }; // motor steps per sec
int MOTOR_MAX_ACCEL[] = { 250, 250, 250, 250, 250, 250 }; // motor steps per sec^2
float MOTOR_ACCEL_MULT[] = { 1.0, 1.0, 2.0, 1.0, 1.0, 1.0 }; // for tuning position control

AccelStepper stepperJoints[NUM_JOINTS];

enum SM { STATE_ARCS, STATE_TRAJ, STATE_ERR };
SM STATE = STATE_ARCS; // default to ARCS state


unsigned long J1DebounceTime = 0;
unsigned long J2DebounceTime = 0;
unsigned long J3DebounceTime = 0;
unsigned long J4DebounceTime = 0;
unsigned long J5DebounceTime = 0;
unsigned long J6DebounceTime = 0;
unsigned long debounceDelay = 50;

String Alarm = "0";
String speedViolation = "0";
float maxSpeedDelay = 3000;
float minSpeedDelay = 350;
float linWayDistSP = 2;
String debug = "";
String flag = "";
const int TRACKrotdir = 0;

int J1EncSteps;
int J2EncSteps;
int J3EncSteps;
int J4EncSteps;
int J5EncSteps;
int J6EncSteps;

#define ROBOT_nDOFs 6
typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[6];

//declare in out vars
float xyzuvw_Out[6];
float xyzuvw_In[7];

float JangleOut[ROBOT_nDOFs];
float JangleIn[ROBOT_nDOFs];
float joints_estimate[ROBOT_nDOFs];
float SolutionMatrix[ROBOT_nDOFs][4];

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

float pose[16];

//DENAVIT HARTENBERG PARAMETERS SAME AS ROBODK

float DHparams[6][4] = {
  {    0,      0,  169.77,      0  },
  {  -90,    -90,       0,   64.2  },
  {    0,      0,       0,    305  },
  {    0,    -90,  222.63,      0  },
  {    0,     90,       0,      0  },
  {  180,    -90,   36.25,      0  }
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRIX OPERATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This allow to return a array as an argument instead of using global pointer

#define Matrix_Multiply(out,inA,inB)\
  (out)[0] = (inA)[0]*(inB)[0] + (inA)[4]*(inB)[1] + (inA)[8]*(inB)[2];\
  (out)[1] = (inA)[1]*(inB)[0] + (inA)[5]*(inB)[1] + (inA)[9]*(inB)[2];\
  (out)[2] = (inA)[2]*(inB)[0] + (inA)[6]*(inB)[1] + (inA)[10]*(inB)[2];\
  (out)[3] = 0;\
  (out)[4] = (inA)[0]*(inB)[4] + (inA)[4]*(inB)[5] + (inA)[8]*(inB)[6];\
  (out)[5] = (inA)[1]*(inB)[4] + (inA)[5]*(inB)[5] + (inA)[9]*(inB)[6];\
  (out)[6] = (inA)[2]*(inB)[4] + (inA)[6]*(inB)[5] + (inA)[10]*(inB)[6];\
  (out)[7] = 0;\
  (out)[8] = (inA)[0]*(inB)[8] + (inA)[4]*(inB)[9] + (inA)[8]*(inB)[10];\
  (out)[9] = (inA)[1]*(inB)[8] + (inA)[5]*(inB)[9] + (inA)[9]*(inB)[10];\
  (out)[10] = (inA)[2]*(inB)[8] + (inA)[6]*(inB)[9] + (inA)[10]*(inB)[10];\
  (out)[11] = 0;\
  (out)[12] = (inA)[0]*(inB)[12] + (inA)[4]*(inB)[13] + (inA)[8]*(inB)[14] + (inA)[12];\
  (out)[13] = (inA)[1]*(inB)[12] + (inA)[5]*(inB)[13] + (inA)[9]*(inB)[14] + (inA)[13];\
  (out)[14] = (inA)[2]*(inB)[12] + (inA)[6]*(inB)[13] + (inA)[10]*(inB)[14] + (inA)[14];\
  (out)[15] = 1;

#define Matrix_Inv(out,in)\
  (out)[0] = (in)[0];\
  (out)[1] = (in)[4];\
  (out)[2] = (in)[8];\
  (out)[3] = 0;\
  (out)[4] = (in)[1];\
  (out)[5] = (in)[5];\
  (out)[6] = (in)[9];\
  (out)[7] = 0;\
  (out)[8] = (in)[2];\
  (out)[9] = (in)[6];\
  (out)[10] = (in)[10];\
  (out)[11] = 0;\
  (out)[12] = -((in)[0]*(in)[12] + (in)[1]*(in)[13] + (in)[2]*(in)[14]);\
  (out)[13] = -((in)[4]*(in)[12] + (in)[5]*(in)[13] + (in)[6]*(in)[14]);\
  (out)[14] = -((in)[8]*(in)[12] + (in)[9]*(in)[13] + (in)[10]*(in)[14]);\
  (out)[15] = 1;

#define Matrix_Copy(out,in)\
  (out)[0]=(in)[0];\
  (out)[1]=(in)[1];\
  (out)[2]=(in)[2];\
  (out)[3]=(in)[3];\
  (out)[4]=(in)[4];\
  (out)[5]=(in)[5];\
  (out)[6]=(in)[6];\
  (out)[7]=(in)[7];\
  (out)[8]=(in)[8];\
  (out)[9]=(in)[9];\
  (out)[10]=(in)[10];\
  (out)[11]=(in)[11];\
  (out)[12]=(in)[12];\
  (out)[13]=(in)[13];\
  (out)[14]=(in)[14];\
  (out)[15]=(in)[15];

#define Matrix_Eye(inout)\
  (inout)[0] = 1;\
  (inout)[1] = 0;\
  (inout)[2] = 0;\
  (inout)[3] = 0;\
  (inout)[4] = 0;\
  (inout)[5] = 1;\
  (inout)[6] = 0;\
  (inout)[7] = 0;\
  (inout)[8] = 0;\
  (inout)[9] = 0;\
  (inout)[10] = 1;\
  (inout)[11] = 0;\
  (inout)[12] = 0;\
  (inout)[13] = 0;\
  (inout)[14] = 0;\
  (inout)[15] = 1;

#define Matrix_Multiply_Cumul(inout,inB){\
    Matrix4x4 out;\
    Matrix_Multiply(out,inout,inB);\
    Matrix_Copy(inout,out);}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//NEW DECLARATION OF VARIABLES ADDED BY OLIVIER ALLARD
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/// DHM Table parameters
#define DHM_Alpha 0
#define DHM_A 1
#define DHM_Theta 2
#define DHM_D 3


/// Custom robot base (user frame)
Matrix4x4 Robot_BaseFrame = {1, 0, 0, 0 , 0, 1, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1};

/// Custom robot tool (tool frame, end of arm tool or TCP)
Matrix4x4 Robot_ToolFrame = {1, 0, 0, 0 , 0, 1, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1};

/// Robot parameters
/// All robot data is held in a large array
tRobot Robot_Data = {0};


//These global variable are also pointers, allowing to put the variables inside the Robot_Data
/// DHM table
float *Robot_Kin_DHM_Table = Robot_Data + 0 * Table_Size;

/// xyzwpr of the base
float *Robot_Kin_Base = Robot_Data + 6 * Table_Size;

/// xyzwpr of the tool
float *Robot_Kin_Tool = Robot_Data + 7 * Table_Size;

/// Robot lower limits
float *Robot_JointLimits_Upper = Robot_Data + 8 * Table_Size;

/// Robot upper limits
float *Robot_JointLimits_Lower = Robot_Data + 9 * Table_Size;

/// Robot axis senses
float *Robot_Senses = Robot_Data + 10 * Table_Size;

// A value mappings

float *Robot_Kin_DHM_L1 = Robot_Kin_DHM_Table + 0 * Table_Size;
float *Robot_Kin_DHM_L2 = Robot_Kin_DHM_Table + 1 * Table_Size;
float *Robot_Kin_DHM_L3 = Robot_Kin_DHM_Table + 2 * Table_Size;
float *Robot_Kin_DHM_L4 = Robot_Kin_DHM_Table + 3 * Table_Size;
float *Robot_Kin_DHM_L5 = Robot_Kin_DHM_Table + 4 * Table_Size;
float *Robot_Kin_DHM_L6 = Robot_Kin_DHM_Table + 5 * Table_Size;


float &Robot_Kin_DHM_A2(Robot_Kin_DHM_Table[1 * Table_Size + 1]);
float &Robot_Kin_DHM_A3(Robot_Kin_DHM_Table[2 * Table_Size + 1]);
float &Robot_Kin_DHM_A4(Robot_Kin_DHM_Table[3 * Table_Size + 1]);

// D value mappings
float &Robot_Kin_DHM_D1(Robot_Kin_DHM_Table[0 * Table_Size + 3]);
float &Robot_Kin_DHM_D2(Robot_Kin_DHM_Table[1 * Table_Size + 3]);
float &Robot_Kin_DHM_D4(Robot_Kin_DHM_Table[3 * Table_Size + 3]);
float &Robot_Kin_DHM_D6(Robot_Kin_DHM_Table[5 * Table_Size + 3]);

// Theta value mappings (mastering)
float &Robot_Kin_DHM_Theta1(Robot_Kin_DHM_Table[0 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta2(Robot_Kin_DHM_Table[1 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta3(Robot_Kin_DHM_Table[2 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta4(Robot_Kin_DHM_Table[3 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta5(Robot_Kin_DHM_Table[4 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta6(Robot_Kin_DHM_Table[5 * Table_Size + 2]);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Call Function Prototypes
void robot_data_reset();
template <typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]);
template <typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw1[6], T joints[ROBOT_nDOFs], const T *joints_estimate);
void updatePos();

// new functions
bool initStateTraj(String inData);
bool reachedLimitSwitch(int joint);


//This function set the variable inside Robot_Data to the DHparams
void robot_set_AR3() {
  robot_data_reset();

  // Alpha parameters
  Robot_Kin_DHM_L1[DHM_Alpha] = DHparams[0][1] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Alpha] = DHparams[1][1] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Alpha] = DHparams[2][1] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Alpha] = DHparams[3][1] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Alpha] = DHparams[4][1] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Alpha] = DHparams[5][1] * M_PI / 180;

  // Theta parameters
  Robot_Kin_DHM_L1[DHM_Theta] = DHparams[0][0] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Theta] = DHparams[1][0] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Theta] = DHparams[2][0] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Theta] = DHparams[3][0] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Theta] = DHparams[4][0] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Theta] = DHparams[5][0] * M_PI / 180;

  // A parameters
  Robot_Kin_DHM_L1[DHM_A] = DHparams[0][3];
  Robot_Kin_DHM_L2[DHM_A] = DHparams[1][3];
  Robot_Kin_DHM_L3[DHM_A] = DHparams[2][3];
  Robot_Kin_DHM_L4[DHM_A] = DHparams[3][3];
  Robot_Kin_DHM_L5[DHM_A] = DHparams[4][3];
  Robot_Kin_DHM_L6[DHM_A] = DHparams[5][3];

  // D parameters
  Robot_Kin_DHM_L1[DHM_D] = DHparams[0][2];
  Robot_Kin_DHM_L2[DHM_D] = DHparams[1][2];
  Robot_Kin_DHM_L3[DHM_D] = DHparams[2][2];
  Robot_Kin_DHM_L4[DHM_D] = DHparams[3][2];
  Robot_Kin_DHM_L5[DHM_D] = DHparams[4][2];
  Robot_Kin_DHM_L6[DHM_D] = DHparams[5][2];


  Robot_JointLimits_Lower[0] = J1axisLimNeg;
  Robot_JointLimits_Upper[0] = J1axisLimPos;
  Robot_JointLimits_Lower[1] = J2axisLimNeg;
  Robot_JointLimits_Upper[1] = J2axisLimPos;
  Robot_JointLimits_Lower[2] = J3axisLimNeg;
  Robot_JointLimits_Upper[2] = J3axisLimPos;
  Robot_JointLimits_Lower[3] = J4axisLimNeg;
  Robot_JointLimits_Upper[3] = J4axisLimPos;
  Robot_JointLimits_Lower[4] = J5axisLimNeg;
  Robot_JointLimits_Upper[4] = J5axisLimPos;
  Robot_JointLimits_Lower[5] = J6axisLimNeg;
  Robot_JointLimits_Upper[5] = J6axisLimPos;

}

void robot_data_reset() {
  // Reset user base and tool frames
  Matrix_Eye(Robot_BaseFrame);
  Matrix_Eye(Robot_ToolFrame);

  // Reset internal base frame and tool frames
  for (int i = 0; i < 6; i++) {
    Robot_Kin_Base[i] = 0.0;
  }

  // Reset joint senses and joint limits
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Robot_Senses[i] = +1.0;

  }

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRICE OPERATIONS ADDED BY OLIVIER ALLARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
bool robot_joints_valid(const T joints[ROBOT_nDOFs]) {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if (joints[i] < -Robot_JointLimits_Lower[i] || joints[i] > Robot_JointLimits_Upper[i]) {
      return false;
    }
  }
  return true;
}


//This function return a 4x4 matrix as an argument (pose) following the modified DH rules for the inputs T rx, T tx, T rz and T tz source : https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
template <typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose)
{
  T crx;
  T srx;
  T crz;
  T srz;
  crx = cos(rx);
  srx = sin(rx);
  crz = cos(rz);
  srz = sin(rz);
  pose[0] = crz;
  pose[4] = -srz;
  pose[8] = 0.0;
  pose[12] = tx;
  pose[1] = crx * srz;
  pose[5] = crx * crz;
  pose[9] = -srx;
  pose[13] = -tz * srx;
  pose[2] = srx * srz;
  pose[6] = crz * srx;
  pose[10] = crx;
  pose[14] = tz * crx;
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


//This function tranform a coordinate system xyzwpr into a 4x4 matrix and return it as an argument.
template <typename T>
void xyzwpr_2_pose(const T xyzwpr[6], Matrix4x4 pose)
{
  T srx;
  T crx;
  T sry;
  T cry;
  T srz;
  T crz;
  T H_tmp;
  srx = sin(xyzwpr[3]);
  crx = cos(xyzwpr[3]);
  sry = sin(xyzwpr[4]);
  cry = cos(xyzwpr[4]);
  srz = sin(xyzwpr[5]);
  crz = cos(xyzwpr[5]);
  pose[0] = cry * crz;
  pose[4] = -cry * srz;
  pose[8] = sry;
  pose[12] = xyzwpr[0];
  H_tmp = crz * srx;
  pose[1] = crx * srz + H_tmp * sry;
  crz *= crx;
  pose[5] = crz - srx * sry * srz;
  pose[9] = -cry * srx;
  pose[13] = xyzwpr[1];
  pose[2] = srx * srz - crz * sry;
  pose[6] = H_tmp + crx * sry * srz;
  pose[10] = crx * cry;
  pose[14] = xyzwpr[2];
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


/// Calculate the [x,y,z,u,v,w] position with rotation vector for a pose target
template <typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T out[6])
{
  T sin_angle;
  T angle;
  T vector[3];
  int iidx;
  int vector_tmp;
  signed char b_I[9];
  out[0] = pose[12];
  out[1] = pose[13];
  out[2] = pose[14];
  sin_angle = (((pose[0] + pose[5]) + pose[10]) - 1.0) * 0.5;
  if (sin_angle <= -1.0) {
    sin_angle = -1.0;
  }

  if (sin_angle >= 1.0) {
    sin_angle = 1.0;
  }

  angle = acos(sin_angle);
  if (angle < 1.0E-6) {
    vector[0] = 0.0;
    vector[1] = 0.0;
    vector[2] = 0.0;
  } else {
    sin_angle = sin(angle);
    if (abs(sin_angle) < 1.0E-6) { //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      sin_angle = pose[0];
      iidx = 0;
      if (pose[0] < pose[5]) {
        sin_angle = pose[5];
        iidx = 1;
      }

      if (sin_angle < pose[10]) {
        sin_angle = pose[10];
        iidx = 2;
      }

      for (vector_tmp = 0; vector_tmp < 9; vector_tmp++) {
        b_I[vector_tmp] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      sin_angle = 2.0 * (1.0 + sin_angle);
      if (sin_angle <= 0.0) {
        sin_angle = 0.0;
      } else {
        sin_angle = sqrt(sin_angle);
      }

      vector_tmp = iidx << 2;
      vector[0] = (pose[vector_tmp] + static_cast<T>(b_I[3 * iidx])) /
                  sin_angle;
      vector[1] = (pose[1 + vector_tmp] + static_cast<T>(b_I[1 + 3 * iidx]))
                  / sin_angle;
      vector[2] = (pose[2 + vector_tmp] + static_cast<T>(b_I[2 + 3 * iidx]))
                  / sin_angle;
      angle = M_PI;
    } else {
      sin_angle = 1.0 / (2.0 * sin_angle);
      vector[0] = (pose[6] - pose[9]) * sin_angle;
      vector[1] = (pose[8] - pose[2]) * sin_angle;
      vector[2] = (pose[1] - pose[4]) * sin_angle;
    }
  }

  sin_angle = angle * 180.0 / M_PI;
  out[3] = vector[0] * sin_angle * M_PI / 180.0;
  out[4] = vector[1] * sin_angle * M_PI / 180.0;
  out[5] = vector[2] * sin_angle * M_PI / 180.0;
}


//This function tranform a coordinate system xyzwpr into a 4x4 matrix using UR euler rules and return it as an argument.
template <typename T>
void xyzuvw_2_pose(const T xyzuvw[6], Matrix4x4 pose)
{
  T s;
  T angle;
  T axisunit[3];
  T ex;
  T c;
  T pose_tmp;
  T b_pose_tmp;
  s = sqrt((xyzuvw[3] * xyzuvw[3] + xyzuvw[4] * xyzuvw[4]) + xyzuvw[5] *
           xyzuvw[5]);
  angle = s * 180.0 / M_PI;
  if (abs(angle) < 1.0E-6) { //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
    memset(&pose[0], 0, sizeof(T) << 4);
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;
    pose[15] = 1.0;
  } else {
    axisunit[1] = abs(xyzuvw[4]);
    axisunit[2] = abs(xyzuvw[5]);
    ex = abs(xyzuvw[3]);
    if (abs(xyzuvw[3]) < axisunit[1]) {
      ex = axisunit[1];
    }

    if (ex < axisunit[2]) {
      ex = axisunit[2];
    }

    if (ex < 1.0E-6) { //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      memset(&pose[0], 0, sizeof(T) << 4);
      pose[0] = 1.0;
      pose[5] = 1.0;
      pose[10] = 1.0;
      pose[15] = 1.0;
    } else {
      axisunit[0] = xyzuvw[3] / s;
      axisunit[1] = xyzuvw[4] / s;
      axisunit[2] = xyzuvw[5] / s;
      s = angle * 3.1415926535897931 / 180.0;
      c = cos(s);
      s = sin(s);
      angle = axisunit[0] * axisunit[0];
      pose[0] = angle + c * (1.0 - angle);
      angle = axisunit[0] * axisunit[1] * (1.0 - c);
      ex = axisunit[2] * s;
      pose[4] = angle - ex;
      pose_tmp = axisunit[0] * axisunit[2] * (1.0 - c);
      b_pose_tmp = axisunit[1] * s;
      pose[8] = pose_tmp + b_pose_tmp;
      pose[1] = angle + ex;
      angle = axisunit[1] * axisunit[1];
      pose[5] = angle + (1.0 - angle) * c;
      angle = axisunit[1] * axisunit[2] * (1.0 - c);
      ex = axisunit[0] * s;
      pose[9] = angle - ex;
      pose[2] = pose_tmp - b_pose_tmp;
      pose[6] = angle + ex;
      angle = axisunit[2] * axisunit[2];
      pose[10] = angle + (1.0 - angle) * c;
      pose[3] = 0.0;
      pose[7] = 0.0;
      pose[11] = 0.0;
      pose[15] = 1.0;
    }
  }

  pose[12] = xyzuvw[0];
  pose[13] = xyzuvw[1];
  pose[14] = xyzuvw[2];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FOWARD KINEMATIC ADDED BY OLIVIER ALLARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function input the JxangleIn into an array, send it to the foward kinematic solver and output the result into the position variables
void SolveFowardKinematic() {

  robot_set_AR3();

  float target_xyzuvw[6];
  float joints[ROBOT_nDOFs];

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints[i] = JangleIn[i];
  }


  forward_kinematics_robot_xyzuvw(joints, target_xyzuvw);

  xyzuvw_Out[0] = target_xyzuvw[0];
  xyzuvw_Out[1] = target_xyzuvw[1];
  xyzuvw_Out[2] = target_xyzuvw[2];
  xyzuvw_Out[3] = target_xyzuvw[3] / M_PI * 180;
  xyzuvw_Out[4] = target_xyzuvw[4] / M_PI * 180;
  xyzuvw_Out[5] = target_xyzuvw[5] / M_PI * 180;


}



template <typename T>
void forward_kinematics_arm(const T *joints, Matrix4x4 pose) {
  xyzwpr_2_pose(Robot_Kin_Base, pose);
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Matrix4x4 hi;
    float *dhm_i = Robot_Kin_DHM_Table + i * Table_Size;
    T ji_rad = joints[i] * Robot_Senses[i] * M_PI / 180.0;
    DHM_2_pose(dhm_i[0], dhm_i[1], dhm_i[2] + ji_rad, dhm_i[3], hi);
    Matrix_Multiply_Cumul(pose, hi);
  }
  Matrix4x4 tool_pose;
  xyzwpr_2_pose(Robot_Kin_Tool, tool_pose);
  Matrix_Multiply_Cumul(pose, tool_pose);
}


template <typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]) {
  Matrix4x4 pose;
  forward_kinematics_robot(joints, pose);    //send the joints values and return the pose matrix as an argument
  pose_2_xyzuvw(pose, target_xyzuvw);        //send the pose matrix and return the xyzuvw values in an array as an argument
}

//Calculate de foward kinematic of the robot without the tool
template <typename T>
void forward_kinematics_robot(const T joints[ROBOT_nDOFs], Matrix4x4 target) {
  Matrix4x4 invBaseFrame;
  Matrix4x4 pose_arm;
  Matrix_Inv(invBaseFrame, Robot_BaseFrame); // invRobot_Tool could be precalculated, the tool does not change so often
  forward_kinematics_arm(joints, pose_arm);
  Matrix_Multiply(target, invBaseFrame, pose_arm);
  Matrix_Multiply_Cumul(target, Robot_ToolFrame);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//REVERSE KINEMATIC ADDED BY OLIVIER ALLARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updatejoints() {

  for (int i = 0; i > ROBOT_nDOFs; i++) {
    JangleIn[i] = JangleOut[i];
  }

}

void JointEstimate() {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints_estimate[i] = JangleIn[i];
  }
}

void SolveInverseKinematic() {

  float joints[ROBOT_nDOFs];
  float target[6];

  float solbuffer[ROBOT_nDOFs] = {0};
  int NumberOfSol = 0;
  int solVal = 0;

  KinematicError = 0;

  JointEstimate();
  target[0] = xyzuvw_In[0];
  target[1] = xyzuvw_In[1];
  target[2] = xyzuvw_In[2];
  target[3] = xyzuvw_In[3] * M_PI / 180;
  target[4] = xyzuvw_In[4] * M_PI / 180;
  target[5] = xyzuvw_In[5] * M_PI / 180;

 // Serial.println("X : " + String(target[0]) + " Y : " + String(target[1]) + " Z : " + String(target[2]) + " rx : " + String(xyzuvw_In[3]) + " ry : " + String(xyzuvw_In[4]) + " rz : " + String(xyzuvw_In[5]));


  for (int i = -3; i <= 3; i++) {
    joints_estimate[4] = i * 30;
    int success = inverse_kinematics_robot_xyzuvw<float>(target, joints, joints_estimate);
    if (success) {
      if (solbuffer[4] != joints[4]) {
        if (robot_joints_valid(joints)) {
          for (int j = 0; j < ROBOT_nDOFs; j++) {
            solbuffer[j] = joints[j];
            SolutionMatrix[j][NumberOfSol] = solbuffer[j];
          }
          if (NumberOfSol <= 6) {
            NumberOfSol++;
          }
        }
      }
    } else {
      KinematicError = 1;
    }
  }

  joints_estimate[4] = JangleIn[4];


  solVal = 0;
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if ((abs(joints_estimate[i] - SolutionMatrix[i][0]) > 20) and NumberOfSol > 1) {
      solVal = 1;
    } else if ((abs(joints_estimate[i] - SolutionMatrix[i][1]) > 20) and NumberOfSol > 1) {
      solVal = 0;
    }


   // Serial.println(String(i) + "  Joint estimate : " + String(joints_estimate[i]) + " // Joint sol 1 : " + String(SolutionMatrix[i][0]) + " // Joint sol 2 : " + String(SolutionMatrix[i][1]));
  }

  if (NumberOfSol==0) {
    KinematicError = 1;
  }


 // Serial.println("Sol : " + String(solVal) + " Nb sol : " + String(NumberOfSol));

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    JangleOut[i] = SolutionMatrix[i][solVal];
  }

}





template <typename T>
int inverse_kinematics_robot(const Matrix4x4 target, T joints[ROBOT_nDOFs], const T *joints_estimate) {
  Matrix4x4 invToolFrame;
  Matrix4x4 pose_arm;
  int nsol;
  Matrix_Inv(invToolFrame, Robot_ToolFrame); // invRobot_Tool could be precalculated, the tool does not change so often
  Matrix_Multiply(pose_arm, Robot_BaseFrame, target);
  Matrix_Multiply_Cumul(pose_arm, invToolFrame);
  if (joints_estimate != nullptr) {
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_estimate, joints, &nsol);
  } else {
    // Warning! This is dangerous if joints does not have a valid/reasonable result
    T joints_approx[6];
    memcpy(joints_approx, joints, ROBOT_nDOFs * sizeof(T));
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_approx, joints, &nsol);
  }
  if (nsol == 0) {
    return 0;
  }

  return 1;
}


template <typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw1[6], T joints[ROBOT_nDOFs], const T *joints_estimate) {

  Matrix4x4 pose;
  xyzuvw_2_pose(target_xyzuvw1, pose);
  return inverse_kinematics_robot(pose, joints, joints_estimate);
}


template <typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int *nsol)
{
  int i0;
  T base[16];
  T joints_approx[6];
  T tool[16];
  int i;
  T Hout[16];
  T b_Hout[9];
  T dv0[4];
  bool guard1 = false;
  T make_sqrt;
  T P04[4];
  T q1;
  int i1;
  T c_Hout[16];
  T k2;
  T k1;
  T ai;
  T B;
  T C;
  T s31;
  T c31;
  T q13_idx_2;
  T bb_div_cc;
  T q13_idx_0;
  for (i0 = 0; i0 < 6; i0++) {
    joints_approx[i0] = DK[60 + i0] * joints_approx_in[i0];
  }

  xyzwpr_2_pose(*(T (*)[6])&DK[36], base);
  xyzwpr_2_pose(*(T (*)[6])&DK[42], tool);
  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    Hout[i] = base[i0];
    Hout[1 + i] = base[i0 + 4];
    Hout[2 + i] = base[i0 + 8];
    Hout[3 + i] = base[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    Hout[3 + i] = 0.0;
    b_Hout[3 * i0] = -Hout[i];
    b_Hout[1 + 3 * i0] = -Hout[1 + i];
    b_Hout[2 + 3 * i0] = -Hout[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    Hout[12 + i0] = (b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13]) + b_Hout[i0 + 6] * base[14];
  }

  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    base[i] = tool[i0];
    base[1 + i] = tool[i0 + 4];
    base[2 + i] = tool[i0 + 8];
    base[3 + i] = tool[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    base[3 + i] = 0.0;
    b_Hout[3 * i0] = -base[i];
    b_Hout[1 + 3 * i0] = -base[1 + i];
    b_Hout[2 + 3 * i0] = -base[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    base[12 + i0] = (b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13]) + b_Hout[i0 + 6] * tool[14];
  }

  dv0[0] = 0.0;
  dv0[1] = 0.0;
  dv0[2] = -DK[33];
  dv0[3] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      c_Hout[i0 + i1] = ((Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]) + Hout[i0 + 8] * pose[2 + i1]) + Hout[i0 + 12] * pose[3 + i1];
    }

    P04[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      make_sqrt = ((c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]) + c_Hout[i0 + 8] * base[2 + i1]) + c_Hout[i0 + 12] * base[3 + i1];
      tool[i0 + i1] = make_sqrt;
      P04[i0] += make_sqrt * dv0[i];
    }
  }

  guard1 = false;
  if (DK[9] == 0.0) {
    q1 = atan2(P04[1], P04[0]);
    guard1 = true;
  } else {
    make_sqrt = (P04[0] * P04[0] + P04[1] * P04[1]) - DK[9] * DK[9];
    if (make_sqrt < 0.0) {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    } else {
      q1 = atan2(P04[1], P04[0]) - atan2(DK[9], sqrt(make_sqrt));
      guard1 = true;
    }
  }

  if (guard1) {
    k2 = P04[2] - DK[3];
    k1 = (cos(q1) * P04[0] + sin(q1) * P04[1]) - DK[7];
    ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] * DK[19];
    B = 2.0 * DK[21] * DK[13];
    C = 2.0 * DK[19] * DK[13];
    s31 = 0.0;
    c31 = 0.0;
    if (C == 0.0) {
      s31 = -ai / B;
      make_sqrt = 1.0 - s31 * s31;
      if (make_sqrt >= 0.0) {
        c31 = sqrt(make_sqrt);
      }
    } else {
      q13_idx_2 = C * C;
      bb_div_cc = B * B / q13_idx_2;
      make_sqrt = 2.0 * ai * B / q13_idx_2;
      make_sqrt = make_sqrt * make_sqrt - 4.0 * ((1.0 + bb_div_cc) * (ai * ai / q13_idx_2 - 1.0));
      if (make_sqrt >= 0.0) {
        s31 = (-2.0 * ai * B / q13_idx_2 + sqrt(make_sqrt)) / (2.0 * (1.0 + bb_div_cc));
        c31 = (ai + B * s31) / C;
      }
    }

    if ((make_sqrt >= 0.0) && (abs(s31) <= 1.0)) {
      B = atan2(s31, c31);
      make_sqrt = cos(B);
      ai = sin(B);
      C = (DK[13] - DK[21] * ai) + DK[19] * make_sqrt;
      make_sqrt = DK[21] * make_sqrt + DK[19] * ai;
      q13_idx_0 = q1 + -DK[2];
      k2 = atan2(C * k1 - make_sqrt * k2, C * k2 + make_sqrt * k1) + (-DK[8] - M_PI / 2);
      q13_idx_2 = B + -DK[14];
      bb_div_cc = joints_approx[3] * M_PI / 180.0 - (-DK[20]);
      q1 = q13_idx_0 + DK[2];
      B = k2 + DK[8];
      C = q13_idx_2 + DK[14];
      make_sqrt = B + C;
      s31 = cos(make_sqrt);
      c31 = cos(q1);
      Hout[0] = s31 * c31;
      ai = sin(q1);
      Hout[4] = s31 * ai;
      make_sqrt = sin(make_sqrt);
      Hout[8] = -make_sqrt;
      Hout[12] = (DK[3] * make_sqrt - DK[7] * s31) - DK[13] * cos(C);
      Hout[1] = -sin(B + C) * c31;
      Hout[5] = -sin(B + C) * ai;
      Hout[9] = -s31;
      Hout[13] = (DK[3] * s31 + DK[7] * make_sqrt) + DK[13] * sin(C);
      Hout[2] = -ai;
      Hout[6] = c31;
      Hout[10] = 0.0;
      Hout[14] = 0.0;
      Hout[3] = 0.0;
      Hout[7] = 0.0;
      Hout[11] = 0.0;
      Hout[15] = 1.0;
      for (i0 = 0; i0 < 4; i0++) {
        for (i = 0; i < 4; i++) {
          i1 = i << 2;
          base[i0 + i1] = ((Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]) + Hout[i0 + 8] * tool[2 + i1]) + Hout[i0 + 12] * tool[3 + i1];
        }
      }

      make_sqrt = 1.0 - base[9] * base[9];
      if (make_sqrt <= 0.0) {
        make_sqrt = 0.0;
      } else {
        make_sqrt = sqrt(make_sqrt);
      }

      if (make_sqrt < 1.0E-6) {
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(bb_div_cc);
        ai = cos(bb_div_cc);
        make_sqrt = atan2(make_sqrt * base[0] + ai * base[2], make_sqrt * base[2] - ai * base[0]);
      } else if (joints_approx[4] >= 0.0) {
        bb_div_cc = atan2(base[10] / make_sqrt, -base[8] / make_sqrt);
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      } else {
        bb_div_cc = atan2(-base[10] / make_sqrt, base[8] / make_sqrt);
        C = atan2(-make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      }

      joints[0] = q13_idx_0;
      joints[3] = bb_div_cc + -DK[20];
      joints[1] = k2;
      joints[4] = C + -DK[26];
      joints[2] = q13_idx_2;
      joints[5] = make_sqrt + (-DK[32] + M_PI);
      make_sqrt = joints[5];
      if (joints[5] > 3.1415926535897931) {
        make_sqrt = joints[5] - M_PI * 2;
      } else {
        if (joints[5] <= -M_PI) {
          make_sqrt = joints[5] + M_PI * 2;
        }
      }

      joints[5] = make_sqrt;
      for (i0 = 0; i0 < 6; i0++) {
        joints[i0] = DK[60 + i0] * (joints[i0] * 180.0 / M_PI);
      }

      *nsol = 1.0;
    } else {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    }
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALCULATE POSITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendRobotPos() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + TRStepM;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";

}

void updatePos() {

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  SolveFowardKinematic();
}



void correctRobotPos () {

  J1StepM = J1encPos.read() / J1encMult;
  J2StepM = J2encPos.read() / J2encMult;
  J3StepM = J3encPos.read() / J3encMult;
  J4StepM = J4encPos.read() / J4encMult;
  J5StepM = J5encPos.read() / J5encMult;
  J6StepM = J6encPos.read() / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


  SolveFowardKinematic();

  //String sendPos = "A" + String(J1StepM) + "B" + String(J2StepM) + "C" + String(J3StepM) + "D" + String(J4StepM) + "E" + String(J5StepM) + "F" + String(J6StepM) + "G" + String(xyzuvw_Out[0], 2) + "H" + String(xyzuvw_Out[1], 2) + "I" + String(xyzuvw_Out[2], 2) + "J" + String(xyzuvw_Out[3], 2) + "K" + String(xyzuvw_Out[4], 2) + "L" + String(xyzuvw_Out[5], 2) + "M" + speedViolation + "N" + debug + "O"+ flag + "P" + TRStepM;
  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + TRStepM;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";



}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(int J1Step, int J2Step, int J3Step, int J4Step, int J5Step, int J6Step, float SpeedVal) {

    //RESET COUNTERS
    int J1done = 0;
    int J2done = 0;
    int J3done = 0;
    int J4done = 0;
    int J5done = 0;
    int J6done = 0;

  int J1complete = 0;
  int J2complete = 0;
  int J3complete = 0;
  int J4complete = 0;
  int J5complete = 0;
  int J6complete = 0;

  int calcStepGap = ((maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay)) + minSpeedDelay + 300);

  //SET CAL DIRECTION
  digitalWrite(J1dirPin, HIGH);
  digitalWrite(J2dirPin, HIGH);
  digitalWrite(J3dirPin, LOW);
  digitalWrite(J4dirPin, LOW);
  digitalWrite(J5dirPin, HIGH);
  digitalWrite(J6dirPin, LOW);

  //DRIVE MOTORS FOR CALIBRATION

  int curRead;
  int J1CurState;
  int J2CurState;
  int J3CurState;
  int J4CurState;
  int J5CurState;
  int J6CurState;
  int DriveLimInProc = 1;

  if (J1Step <= 0 ) {
    J1complete = 1;
  }
  if (J2Step <= 0) {
    J2complete = 1;
  }
  if (J3Step <= 0) {
    J3complete = 1;
  }
  if (J4Step <= 0) {
    J4complete = 1;
  }
  if (J5Step <= 0) {
    J5complete = 1;
  }
  if (J6Step <= 0) {
    J6complete = 1;
  }


  while (DriveLimInProc == 1) {

    //EVAL J1
    if (digitalRead(J1calPin) == LOW) {
      J1CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J1calPin) == LOW) {
        J1CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J1calPin) == LOW) {
          J1CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J1calPin) == LOW) {
            J1CurState = LOW;
          }
          else {
            J1CurState = digitalRead(J1calPin);
          }
        }
      }
    }

    //EVAL J2
    if (digitalRead(J2calPin) == LOW) {
      J2CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J2calPin) == LOW) {
        J2CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J2calPin) == LOW) {
          J2CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J2calPin) == LOW) {
            J2CurState = LOW;
          }
          else {
            J2CurState = digitalRead(J2calPin);
          }
        }
      }
    }

    //EVAL J3
    if (digitalRead(J3calPin) == LOW) {
      J3CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J3calPin) == LOW) {
        J3CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J3calPin) == LOW) {
          J3CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J3calPin) == LOW) {
            J3CurState = LOW;
          }
          else {
            J3CurState = digitalRead(J3calPin);
          }
        }
      }
    }

    //EVAL J4
    if (digitalRead(J4calPin) == LOW) {
      J4CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J4calPin) == LOW) {
        J4CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J4calPin) == LOW) {
          J4CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J4calPin) == LOW) {
            J4CurState = LOW;
          }
          else {
            J4CurState = digitalRead(J4calPin);
          }
        }
      }
    }

    //EVAL J5
    if (digitalRead(J5calPin) == LOW) {
      J5CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J5calPin) == LOW) {
        J5CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J5calPin) == LOW) {
          J5CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J5calPin) == LOW) {
            J5CurState = LOW;
          }
          else {
            J5CurState = digitalRead(J5calPin);
          }
        }
      }
    }

    //EVAL J6
    if (digitalRead(J6calPin) == LOW) {
      J6CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J6calPin) == LOW) {
        J6CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J6calPin) == LOW) {
          J6CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J6calPin) == LOW) {
            J6CurState = LOW;
          }
          else {
            J6CurState = digitalRead(J6calPin);
          }
        }
      }
    }

    if (J1done < J1Step && J1CurState == LOW) {
      digitalWrite(J1stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J1stepPin, HIGH);
      J1done = ++J1done;
    }
    else {
      J1complete = 1;
    }
    if (J2done < J2Step && J2CurState == LOW) {
      digitalWrite(J2stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J2stepPin, HIGH);
      J2done = ++J2done;
    }
    else {
      J2complete = 1;
    }
    if (J3done < J3Step && J3CurState == LOW) {
      digitalWrite(J3stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J3stepPin, HIGH);
      J3done = ++J3done;
    }
    else {
      J3complete = 1;
    }
    if (J4done < J4Step && J4CurState == LOW) {
      digitalWrite(J4stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J4stepPin, HIGH);
      J4done = ++J4done;
    }
    else {
      J4complete = 1;
    }
    if (J5done < J5Step && J5CurState == LOW) {
      digitalWrite(J5stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J5stepPin, HIGH);
      J5done = ++J5done;
    }
    else {
      J5complete = 1;
    }
    if (J6done < J6Step && J6CurState == LOW) {
      digitalWrite(J6stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J6stepPin, HIGH);
      J6done = ++J6done;
    }
    else {
      J6complete = 1;
    }
    //jump out if complete
    if (J1complete + J2complete + J3complete + J4complete + J5complete + J6complete == 6) {
      DriveLimInProc = 0;
    }
    ///////////////DELAY BEFORE RESTARTING LOOP
    delayMicroseconds(calcStepGap);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CHECK ENCODERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void resetEncoders() {

  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  //set encoders to current position
  J1encPos.write(J1StepM * J1encMult);
  J2encPos.write(J2StepM * J2encMult);
  J3encPos.write(J3StepM * J3encMult);
  J4encPos.write(J4StepM * J4encMult);
  J5encPos.write(J5StepM * J5encMult);
  J6encPos.write(J6StepM * J6encMult);
  //delayMicroseconds(5);

}

void checkEncoders() {
  //read encoders
  J1EncSteps = J1encPos.read() / J1encMult;
  J2EncSteps = J2encPos.read() / J2encMult;
  J3EncSteps = J3encPos.read() / J3encMult;
  J4EncSteps = J4encPos.read() / J4encMult;
  J5EncSteps = J5encPos.read() / J5encMult;
  J6EncSteps = J6encPos.read() / J6encMult;

  if (abs((J1EncSteps - J1StepM)) >= 15) {
    J1collisionTrue = 1;
  }
  if (abs((J2EncSteps - J2StepM)) >= 15) {
    J2collisionTrue = 1;
  }
  if (abs((J3EncSteps - J3StepM)) >= 15) {
    J3collisionTrue = 1;
  }
  if (abs((J4EncSteps - J4StepM)) >= 15) {
    J4collisionTrue = 1;
  }
  if (abs((J5EncSteps - J5StepM)) >= 15) {
    J5collisionTrue = 1;
  }
  if (abs((J6EncSteps - J6StepM)) >= 15) {
    J6collisionTrue = 1;
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }

  //debug = String(J5StepM) + "-" + String(J5EncSteps);



}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int TRstep, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int TRdir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (TRstep > HighStep)
  {
    HighStep = TRstep;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int TRactive = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (TRstep >= 1)
  {
    TRactive = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int TR_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int TR_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int TR_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int TR_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int TR_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int TRcur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int TR_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int TR_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int TR_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;

  float speedSP;
  float moveDist;

  //int J4EncSteps;

  //SET DIRECTIONS

  /// J1 ///
  if (J1dir) {
    digitalWrite(J1dirPin, HIGH);
  }
  else {
    digitalWrite(J1dirPin, LOW);
  }
  /// J2 ///
  if (J2dir) {
    digitalWrite(J2dirPin, LOW);
  }
  else {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir) {
    digitalWrite(J3dirPin, LOW);
  }
  else {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir) {
    digitalWrite(J4dirPin, HIGH);
  }
  else {
    digitalWrite(J4dirPin, LOW);
  }
  /// J5 ///
  if (J5dir) {
    digitalWrite(J5dirPin, LOW);
  }
  else {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir) {
    digitalWrite(J6dirPin, LOW);
  }
  else {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (TRdir) {
    digitalWrite(TRdirPin, HIGH);
  }
  else {
    digitalWrite(TRdirPin, LOW);
  }

  /////CALC SPEEDS//////
  float calcStepGap;

  //determine steps
  float ACCStep = HighStep * (ACCspd / 100);
  float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
  float DCCStep = HighStep * (DCCspd / 100);

  //set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * .8;
  }
  else if (SpeedType == "m") {
    lineDist = pow((pow((xyzuvw_In[0] - xyzuvw_Out[0]), 2) + pow((xyzuvw_In[1] - xyzuvw_Out[1]), 2) + pow((xyzuvw_In[2] - xyzuvw_Out[2]), 2)), .5);
    speedSP = ((lineDist / SpeedVal) * 1000000) * .8;
  }

  //calc step gap for seconds or mm per sec
  if (SpeedType == "s" or SpeedType == "m" ) {
    float zeroStepGap = speedSP / HighStep;
    float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
    float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
    float zeroNORtime = NORStep * zeroStepGap;
    float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
    float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
    float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
    float overclockPerc = speedSP / zeroTOTtime;
    calcStepGap = zeroStepGap * overclockPerc;
    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  }

  //calc step gap for percentage
  else if (SpeedType == "p") {
    calcStepGap = (maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay));
    if (calcStepGap < minSpeedDelay) {
      calcStepGap = minSpeedDelay;
    }
  }

  //calculate final step increments
  float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
  float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
  float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;

  //set starting delay
  curDelay = calcACCstartDel;

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
  {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep) {
      curDelay = curDelay - (calcACCstepInc);
    }
    else if (highStepCur >= (HighStep - DCCStep)) {
      curDelay = curDelay + (calcDCCstepInc);
    }
    else {
      curDelay = calcStepGap;
    }



    float distDelay = 60;
    if (debugg == 1) {
      distDelay = 0;
    }
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// TR ////////////////////////////////
    ///find pulse every
    if (TRcur < TRstep)
    {
      TR_PE = (HighStep / TRstep);
      ///find left over 1
      TR_LO_1 = (HighStep - (TRstep * TR_PE));
      ///find skip 1
      if (TR_LO_1 > 0)
      {
        TR_SE_1 = (HighStep / TR_LO_1);
      }
      else
      {
        TR_SE_1 = 0;
      }
      ///find left over 2
      if (TR_SE_1 > 0)
      {
        TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
      }
      else
      {
        TR_LO_2 = 0;
      }
      ///find skip 2
      if (TR_LO_2 > 0)
      {
        TR_SE_2 = (HighStep / TR_LO_2);
      }
      else
      {
        TR_SE_2 = 0;
      }
      /////////  TR  ///////////////
      if (TR_SE_2 == 0)
      {
        TR_SE_2cur = (TR_SE_2 + 1);
      }
      if (TR_SE_2cur != TR_SE_2)
      {
        TR_SE_2cur = ++TR_SE_2cur;
        if (TR_SE_1 == 0)
        {
          TR_SE_1cur = (TR_SE_1 + 1);
        }
        if (TR_SE_1cur != TR_SE_1)
        {
          TR_SE_1cur = ++TR_SE_1cur;
          TR_PEcur = ++TR_PEcur;
          if (TR_PEcur == TR_PE)
          {
            TRcur = ++TRcur;
            TR_PEcur = 0;
            digitalWrite(TRstepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (TRdir == 0) {
              TRStepM == --TRStepM;
            }
            else {
              TRStepM == ++TRStepM;
            }
          }
        }
        else
        {
          TR_SE_1cur = 0;
        }
      }
      else
      {
        TR_SE_2cur = 0;
      }
    }

    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(TRstepPin, HIGH);
    if (debugg == 0) {
      delayMicroseconds(curDelay - disDelayCur);
    }

  }



}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int TRstep, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int TRdir, float curDelay) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (TRstep > HighStep)
  {
    HighStep = TRstep;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int TRactive = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (TRstep >= 1)
  {
    TRactive = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int TR_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int TR_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int TR_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int TR_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int TR_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int TRcur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int TR_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int TR_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int TR_SE_2cur = 0;

  int highStepCur = 0;

  float speedSP;
  float moveDist;

  //SET DIRECTIONS

  /// J1 ///
  if (J1dir) {
    digitalWrite(J1dirPin, HIGH);
  }
  else {
    digitalWrite(J1dirPin, LOW);
  }
  /// J2 ///
  if (J2dir) {
    digitalWrite(J2dirPin, LOW);
  }
  else {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir) {
    digitalWrite(J3dirPin, LOW);
  }
  else {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir) {
    digitalWrite(J4dirPin, HIGH);
  }
  else {
    digitalWrite(J4dirPin, LOW);
  }
  /// J5 ///
  if (J5dir) {
    digitalWrite(J5dirPin, LOW);
  }
  else {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir) {
    digitalWrite(J6dirPin, LOW);
  }
  else {
    digitalWrite(J6dirPin, HIGH);
  }

  /// TR ///
  if (TRdir) {
    digitalWrite(TRdirPin, HIGH);
  }
  else {
    digitalWrite(TRdirPin, LOW);
  }


  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
  {

    float distDelay = 60;
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////

    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// TR ////////////////////////////////
    ///find pulse every
    if (TRcur < TRstep)
    {
      TR_PE = (HighStep / TRstep);
      ///find left over 1
      TR_LO_1 = (HighStep - (TRstep * TR_PE));
      ///find skip 1
      if (TR_LO_1 > 0)
      {
        TR_SE_1 = (HighStep / TR_LO_1);
      }
      else
      {
        TR_SE_1 = 0;
      }
      ///find left over 2
      if (TR_SE_1 > 0)
      {
        TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
      }
      else
      {
        TR_LO_2 = 0;
      }
      ///find skip 2
      if (TR_LO_2 > 0)
      {
        TR_SE_2 = (HighStep / TR_LO_2);
      }
      else
      {
        TR_SE_2 = 0;
      }
      /////////  TR  ///////////////
      if (TR_SE_2 == 0)
      {
        TR_SE_2cur = (TR_SE_2 + 1);
      }
      if (TR_SE_2cur != TR_SE_2)
      {
        TR_SE_2cur = ++TR_SE_2cur;
        if (TR_SE_1 == 0)
        {
          TR_SE_1cur = (TR_SE_1 + 1);
        }
        if (TR_SE_1cur != TR_SE_1)
        {
          TR_SE_1cur = ++TR_SE_1cur;
          TR_PEcur = ++TR_PEcur;
          if (TR_PEcur == TR_PE)
          {
            TRcur = ++TRcur;
            TR_PEcur = 0;
            digitalWrite(TRstepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (TRdir == 0) {
              TRStepM == --TRStepM;
            }
            else {
              TRStepM == ++TRStepM;
            }
          }
        }
        else
        {
          TR_SE_1cur = 0;
        }
      }
      else
      {
        TR_SE_2cur = 0;
      }
    }

    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(TRstepPin, HIGH);
    delayMicroseconds(curDelay - disDelayCur);

  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  // run once:
  Serial.begin(9600);

  pinMode(TRstepPin, OUTPUT);
  pinMode(TRdirPin, OUTPUT);
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);

  pinMode(Input32, INPUT_PULLUP);
  pinMode(Input33, INPUT_PULLUP);
  pinMode(Input34, INPUT_PULLUP);
  pinMode(Input35, INPUT_PULLUP);
  pinMode(Input36, INPUT_PULLUP);

  pinMode(Output37, OUTPUT);
  pinMode(Output38, OUTPUT);
  pinMode(Output39, OUTPUT);
  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);


  digitalWrite(TRstepPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);

}


void stateARCS() {

  //start loop
  WayPtDel = 0;
  while (Serial.available())
  {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      inData.trim();
      String function = inData.substring(0, 2);
      inData = inData.substring(2);
      KinematicError = 0;

      //----- INIT STATE_TRAJ-----//
      if (function == "ST")
      {
        if (initStateTraj(inData))
        {
          //STATE = STATE_TRAJ;
          //return;
        }
      }

      //-----COMMAND TO CLOSE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "CL")
      {
        delay(5);
        Serial.end();
      }

      //-----COMMAND TEST LIMIT SWITCHES---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TL") {

        String J1calTest = "0";
        String J2calTest = "0";
        String J3calTest = "0";
        String J4calTest = "0";
        String J5calTest = "0";
        String J6calTest = "0";

        if (digitalRead(J1calPin) == HIGH) {
          J1calTest = "1";
        }
        if (digitalRead(J2calPin) == HIGH) {
          J2calTest = "1";
        }
        if (digitalRead(J3calPin) == HIGH) {
          J3calTest = "1";
        }
        if (digitalRead(J4calPin) == HIGH) {
          J4calTest = "1";
        }
        if (digitalRead(J5calPin) == HIGH) {
          J5calTest = "1";
        }
        if (digitalRead(J6calPin) == HIGH) {
          J6calTest = "1";
        }
        String TestLim = " J1 = " + J1calTest + "   J2 = " + J2calTest + "   J3 = " + J3calTest + "   J4 = " + J4calTest + "   J5 = " + J5calTest + "   J6 = " + J6calTest;
        delay(5);
        Serial.println(TestLim);
      }


      //-----COMMAND SET ENCODERS TO 1000---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "SE")
      {
        J1encPos.write(1000);
        J2encPos.write(1000);
        J3encPos.write(1000);
        J4encPos.write(1000);
        J5encPos.write(1000);
        J6encPos.write(1000);
        delay(5);
        Serial.print("Done");
      }

      //-----COMMAND READ ENCODERS---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "RE")
      {
        J1EncSteps = J1encPos.read();
        J2EncSteps = J2encPos.read();
        J3EncSteps = J3encPos.read();
        J4EncSteps = J4encPos.read();
        J5EncSteps = J5encPos.read();
        J6EncSteps = J6encPos.read();
        String Read = " J1 = " + String(J1EncSteps) + "   J2 = " + String(J2EncSteps) + "   J3 = " + String(J3EncSteps) + "   J4 = " + String(J4EncSteps) + "   J5 = " + String(J5EncSteps) + "   J6 = " + String(J6EncSteps);
        delay(5);
        Serial.println(Read);
      }

      //-----COMMAND REQUEST POSITION---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "RP")
      {
        sendRobotPos();
      }


      //-----COMMAND HOME POSITION---------------------------------------------------
      //-----------------------------------------------------------------------

      //For debugging
      if (function == "HM")
      {

        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;


        String SpeedType = "p";
        float SpeedVal = 25.0;
        float ACCspd = 10.0;
        float DCCspd = 10.0;
        float ACCramp = 20.0;

        JangleIn[0] = 0.00;
        JangleIn[1] = 0.00;
        JangleIn[2] = 0.00;
        JangleIn[3] = 0.00;
        JangleIn[4] = 0.00;
        JangleIn[5] = 0.00;


        //calc destination motor steps
        int J1futStepM = J1axisLimNeg * J1StepDeg;
        int J2futStepM = J2axisLimNeg * J2StepDeg;
        int J3futStepM = J3axisLimNeg * J3StepDeg;
        int J4futStepM = J4axisLimNeg * J4StepDeg;
        int J5futStepM = J5axisLimNeg * J5StepDeg;
        int J6futStepM = J6axisLimNeg * J6StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int TRstepDif = 0;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        TRdir = 0;



        resetEncoders();
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
        delay(5);
        Serial.println("Done");
      }


      //-----COMMAND CORRECT POSITION---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "CP")
      {
        correctRobotPos();
      }

      //-----COMMAND SET TOOL FRAME---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TF")
      {
        int TFxStart = inData.indexOf('A');
        int TFyStart = inData.indexOf('B');
        int TFzStart = inData.indexOf('C');
        int TFrzStart = inData.indexOf('D');
        int TFryStart = inData.indexOf('E');
        int TFrxStart = inData.indexOf('F');
        Robot_Kin_Tool[0] = inData.substring(TFxStart + 1, TFyStart).toFloat();
        Robot_Kin_Tool[1] = inData.substring(TFyStart + 1, TFzStart).toFloat();
        Robot_Kin_Tool[2] = inData.substring(TFzStart + 1, TFrzStart).toFloat();
        Robot_Kin_Tool[3] = inData.substring(TFrzStart + 1, TFryStart).toFloat() * M_PI / 180;
        Robot_Kin_Tool[4] = inData.substring(TFryStart + 1, TFrxStart).toFloat() * M_PI / 180;
        Robot_Kin_Tool[5] = inData.substring(TFrxStart + 1).toFloat() * M_PI / 180;
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND CALIBRATE TRACK---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "CT")
      {
        int axis7lengthStart = inData.indexOf('A');
        int axis7rotStart = inData.indexOf('B');
        int axis7stepsStart = inData.indexOf('C');
        axis7length = inData.substring(axis7lengthStart + 1, axis7rotStart).toFloat();
        axis7rot = inData.substring(axis7rotStart + 1, axis7stepsStart).toFloat();
        axis7steps = inData.substring(axis7stepsStart + 1).toFloat();
        TRaxisLimNeg = 0;
        TRaxisLimPos = axis7length;
        TRaxisLim = TRaxisLimPos + TRaxisLimNeg;
        TRStepDeg = axis7steps / axis7rot;
        TRStepLim = TRaxisLim * TRStepDeg;
        delay(5);
        Serial.print("Done");
      }

      //-----COMMAND ZERO TRACK---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ZT")
      {
        TRStepM = 0;
        sendRobotPos();
      }

      /*
            //-----SET ERROR IF COLLISION STATUS IS TRUE------------------------------------
            //-----------------------------------------------------------------------
            if (J1collisionTrue == 1) {
              function = "XX";
              Serial.println("EC100000");
            }
            if (J2collisionTrue == 1) {
              function = "XX";
              Serial.println("EC010000");
            }
            if (J3collisionTrue == 1) {
              function = "XX";
              Serial.println("EC001000");
            }
            if (J4collisionTrue == 1) {
              function = "XX";
              Serial.println("EC000100");
            }
            if (J5collisionTrue == 1) {
              function = "XX";
              Serial.println("EC000010");
            }
            if (J6collisionTrue == 1) {
              function = "XX";
              Serial.println("EC000001");
            }
      */


      //-----COMMAND TO WAIT TIME---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WT")
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.println("Done");
      }

      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "JF")
      {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH)
        {
          delay(5);
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW)
        {
          delay(5);
          Serial.println("F");
        }
      }
      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        delay(5);
        Serial.println("Done");
      }
      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        delay(5);
        Serial.println("Done");
      }
      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WI")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(100);
        }
        delay(5);
        Serial.println("Done");
      }
      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WO")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == HIGH) {
          delay(100);
        }
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND SEND POSITION---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "SP")
      {
        int J1angStart = inData.indexOf('A');
        int J2angStart = inData.indexOf('B');
        int J3angStart = inData.indexOf('C');
        int J4angStart = inData.indexOf('D');
        int J5angStart = inData.indexOf('E');
        int J6angStart = inData.indexOf('F');
        int TRstepStart = inData.indexOf('G');
        J1StepM = ((inData.substring(J1angStart + 1, J2angStart).toFloat()) + J1axisLimNeg) * J1StepDeg;
        J2StepM = ((inData.substring(J2angStart + 1, J3angStart).toFloat()) + J2axisLimNeg) * J2StepDeg;
        J3StepM = ((inData.substring(J3angStart + 1, J4angStart).toFloat()) + J3axisLimNeg) * J3StepDeg;
        J4StepM = ((inData.substring(J4angStart + 1, J5angStart).toFloat()) + J4axisLimNeg) * J4StepDeg;
        J5StepM = ((inData.substring(J5angStart + 1, J6angStart).toFloat()) + J5axisLimNeg) * J5StepDeg;
        J6StepM = ((inData.substring(J6angStart + 1, TRstepStart).toFloat()) + J6axisLimNeg) * J6StepDeg;
        TRStepM = inData.substring(TRstepStart + 1).toFloat();
        delay(5);
        Serial.println("Done");
      }


      //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TM")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int WristConStart = inData.indexOf('W');
        JangleIn[0] = inData.substring(J1start + 1, J2start).toFloat();
        JangleIn[1] = inData.substring(J2start + 1, J3start).toFloat();
        JangleIn[2] = inData.substring(J3start + 1, J4start).toFloat();
        JangleIn[3] = inData.substring(J4start + 1, J5start).toFloat();
        JangleIn[4] = inData.substring(J5start + 1, J6start).toFloat();
        JangleIn[5] = inData.substring(J6start + 1, WristConStart).toFloat();
        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        SolveInverseKinematic();

        String echo = "";
        delay(5);
        Serial.println(inData);


      }
      //-----COMMAND TO CALIBRATE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LL")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int J1calstart = inData.indexOf('G');
        int J2calstart = inData.indexOf('H');
        int J3calstart = inData.indexOf('I');
        int J4calstart = inData.indexOf('J');
        int J5calstart = inData.indexOf('K');
        int J6calstart = inData.indexOf('L');
        ///
        int J1req = inData.substring(J1start + 1, J2start).toInt();
        int J2req = inData.substring(J2start + 1, J3start).toInt();
        int J3req = inData.substring(J3start + 1, J4start).toInt();
        int J4req = inData.substring(J4start + 1, J5start).toInt();
        int J5req = inData.substring(J5start + 1, J6start).toInt();
        int J6req = inData.substring(J6start + 1, J1calstart).toInt();
        float J1calOff = inData.substring(J1calstart + 1, J2calstart).toFloat();
        float J2calOff = inData.substring(J2calstart + 1, J3calstart).toFloat();
        float J3calOff = inData.substring(J3calstart + 1, J4calstart).toFloat();
        float J4calOff = inData.substring(J4calstart + 1, J5calstart).toFloat();
        float J5calOff = inData.substring(J5calstart + 1, J6calstart).toFloat();
        float J6calOff = inData.substring(J6calstart + 1).toFloat();
        ///
        float SpeedIn;
        ///
        int J1Step = 0;
        int J2Step = 0;
        int J3Step = 0;
        int J4Step = 0;
        int J5Step = 0;
        int J6Step = 0;
        ///
        int J1stepCen = 0;
        int J2stepCen = 0;
        int J3stepCen = 0;
        int J4stepCen = 0;
        int J5stepCen = 0;
        int J6stepCen = 0;
        Alarm = "0";

        //--IF JOINT IS CALLED FOR CALIBRATION PASS ITS STEP LIMIT OTHERWISE PASS 0---
        if (J1req == 1) {
          J1Step = J1StepLim;
        }
        if (J2req == 1) {
          J2Step = J2StepLim;
        }
        if (J3req == 1) {
          J3Step = J3StepLim;
        }
        if (J4req == 1) {
          J4Step = J4StepLim;
        }
        if (J5req == 1) {
          J5Step = J5StepLim;
        }
        if (J6req == 1) {
          J6Step = J6StepLim;
        }

        //--CALL FUNCT TO DRIVE TO LIMITS--
        SpeedIn = 80;
        driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, SpeedIn);
        delay(500);

        //BACKOFF
        digitalWrite(J1dirPin, LOW);
        digitalWrite(J2dirPin, LOW);
        digitalWrite(J3dirPin, HIGH);
        digitalWrite(J4dirPin, HIGH);
        digitalWrite(J5dirPin, LOW);
        digitalWrite(J6dirPin, HIGH);

        int BacOff = 0;
        while (BacOff <= 250)
        {
          if (J1req == 1) {
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J1stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J2req == 1) {
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J2stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J3req == 1) {
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J3stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J4req == 1) {
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J4stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J5req == 1) {
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J5stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J6req == 1) {
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J6stepPin, HIGH);
            delayMicroseconds(5);
          }
          BacOff = ++BacOff;
          delayMicroseconds(4000);
        }

        //--CALL FUNCT TO DRIVE BACK TO LIMITS SLOWLY--
        SpeedIn = .02;
        driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, SpeedIn);

        //OVERDRIVE - MAKE SURE LIMIT SWITCH STAYS MADE
        digitalWrite(J1dirPin, HIGH);
        digitalWrite(J2dirPin, HIGH);
        digitalWrite(J3dirPin, LOW);
        digitalWrite(J4dirPin, LOW);
        digitalWrite(J5dirPin, HIGH);
        digitalWrite(J6dirPin, LOW);

        int OvrDrv = 0;
        while (OvrDrv <= 30)
        {
          if (J1req == 1) {
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J1stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J2req == 1) {
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J2stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J3req == 1) {
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J3stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J4req == 1) {
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J4stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J5req == 1) {
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J5stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J6req == 1) {
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J6stepPin, HIGH);
            delayMicroseconds(5);
          }
          OvrDrv = ++OvrDrv;
          delayMicroseconds(3000);
        }

        //SEE IF ANY SWITCHES NOT MADE
        delay(500);
        ///
        if (J1req == 1) {
          if (digitalRead(J1calPin) == LOW) {
            Alarm = "1";
          }
        }
        if (J2req == 1) {
          if (digitalRead(J2calPin) == LOW) {
            Alarm = "2";
          }
        }
        if (J3req == 1) {
          if (digitalRead(J3calPin) == LOW) {
            Alarm = "3";
          }
        }
        if (J4req == 1) {
          if (digitalRead(J4calPin) == LOW) {
            Alarm = "4";
          }
        }
        if (J5req == 1) {
          if (digitalRead(J5calPin) == LOW) {
            Alarm = "5";
          }
        }
        if (J6req == 1) {
          if (digitalRead(J6calPin) == LOW) {
            Alarm = "6";
          }
        }
        ///
        if (Alarm == "0") {

          //set master steps and center step
          if (J1req == 1) {
            J1StepM = ((J1axisLim) + J1calBaseOff + J1calOff) * J1StepDeg;
            J1stepCen = ((J1axisLimPos) + J1calBaseOff + J1calOff) * J1StepDeg;
          }
          if (J2req == 1) {
            J2StepM = (0 + J2calBaseOff + J2calOff) * J2StepDeg;
            J2stepCen = ((J2axisLimNeg) - J2calBaseOff - J2calOff) * J2StepDeg;
          }
          if (J3req == 1) {
            J3StepM = ((J3axisLim) + J3calBaseOff + J3calOff) * J3StepDeg;
            J3stepCen = ((J3axisLimPos) + J3calBaseOff + J3calOff) * J3StepDeg;
          }
          if (J4req == 1) {
            J4StepM = (0 + J4calBaseOff + J4calOff) * J4StepDeg;
            J4stepCen = ((J4axisLimNeg) - J4calBaseOff - J4calOff) * J4StepDeg;
          }
          if (J5req == 1) {
            J5StepM = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
            J5stepCen = ((J5axisLimNeg) - J5calBaseOff - J5calOff) * J5StepDeg;
          }
          if (J6req == 1) {
            J6StepM = ((J6axisLim) + J6calBaseOff + J6calOff) * J6StepDeg;
            J6stepCen = ((J6axisLimNeg) + J6calBaseOff + J6calOff) * J6StepDeg;
          }
          //move to center
          int J1dir = 0;
          int J2dir = 1;
          int J3dir = 0;
          int J4dir = 1;
          int J5dir = 1;
          int J6dir = 0;
          int TRdir = 0;
          int TRstep = 0;
          float ACCspd = 10;
          float DCCspd = 10;
          String SpeedType = "p";
          float SpeedVal = 80;
          float ACCramp = 50;

          driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5stepCen, J6stepCen, TRstep, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          sendRobotPos();

        }
        else {
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
      }



      //----- MOVE J ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MJ")
      {
        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        Alarm = "0";

        int xStart = inData.indexOf("X");
        int yStart = inData.indexOf("Y");
        int zStart = inData.indexOf("Z");
        int rzStart = inData.indexOf("Rz");
        int ryStart = inData.indexOf("Ry");
        int rxStart = inData.indexOf("Rx");
        int tStart = inData.indexOf("Tr");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
        xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
        xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
        xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
        xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
        xyzuvw_In[5] = inData.substring(rxStart + 2, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();

        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();

        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        SolveInverseKinematic();

        //calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;
        int TRfutStepM = (xyzuvw_In[6] + TRaxisLimNeg) * TRStepDeg;


        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int TRstepDif = TRStepM - TRfutStepM;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        if (TRstepDif <= 0) {
          TRdir = 1;
        }
        else {
          TRdir = 0;
        }



        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0))) {
          TRaxisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;




        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          checkEncoders();
          sendRobotPos();
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }


        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }







      //----- LIVE CARTESIAN JOG  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LC")
      {
        delay(5);
        Serial.println();


        updatePos();

        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        bool JogInPoc = true;
        Alarm = "0";


        int VStart = inData.indexOf("V");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");


        float Vector = inData.substring(VStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = 100;
        float DCCspd = 100;
        float ACCramp = 100;


        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        inData = ""; // Clear recieved buffer

        while (JogInPoc = true) {

          xyzuvw_In[0] = xyzuvw_Out[0];
          xyzuvw_In[1] = xyzuvw_Out[1];
          xyzuvw_In[2] = xyzuvw_Out[2];
          xyzuvw_In[3] = xyzuvw_Out[3];
          xyzuvw_In[4] = xyzuvw_Out[4];
          xyzuvw_In[5] = xyzuvw_Out[5];

          if (Vector == 10) {
            xyzuvw_In[0] = xyzuvw_Out[0] - 1;
          }
          if (Vector == 11) {
            xyzuvw_In[0] = xyzuvw_Out[0] + 1;
          }

          if (Vector == 20) {
            xyzuvw_In[1] = xyzuvw_Out[1] - 1;
          }
          if (Vector == 21) {
            xyzuvw_In[1] = xyzuvw_Out[1] + 1;
          }

          if (Vector == 30) {
            xyzuvw_In[2] = xyzuvw_Out[2] - 1;
          }
          if (Vector == 31) {
            xyzuvw_In[2] = xyzuvw_Out[2] + 1;
          }

          if (Vector == 40) {
            xyzuvw_In[3] = xyzuvw_Out[3] - 1;
          }
          if (Vector == 41) {
            xyzuvw_In[3] = xyzuvw_Out[3] + 1;
          }

          if (Vector == 50) {
            xyzuvw_In[4] = xyzuvw_Out[4] - 1;
          }
          if (Vector == 51) {
            xyzuvw_In[4] = xyzuvw_Out[4] + 1;
          }

          if (Vector == 60) {
            xyzuvw_In[5] = xyzuvw_Out[5] - 1;
          }
          if (Vector == 61) {
            xyzuvw_In[5] = xyzuvw_Out[5] + 1;
          }

          SolveInverseKinematic();

          //calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          //determine motor directions
          if (J1stepDif <= 0) {
            J1dir = 1;
          }
          else {
            J1dir = 0;
          }

          if (J2stepDif <= 0) {
            J2dir = 1;
          }
          else {
            J2dir = 0;
          }

          if (J3stepDif <= 0) {
            J3dir = 1;
          }
          else {
            J3dir = 0;
          }

          if (J4stepDif <= 0) {
            J4dir = 1;
          }
          else {
            J4dir = 0;
          }

          if (J5stepDif <= 0) {
            J5dir = 1;
          }
          else {
            J5dir = 0;
          }

          if (J6stepDif <= 0) {
            J6dir = 1;
          }
          else {
            J6dir = 0;
          }

          if (TRstepDif <= 0) {
            TRdir = 1;
          }
          else {
            TRdir = 0;
          }


          //determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
            J6axisFault = 1;
          }
          if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0))) {
            TRaxisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;


          //send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0) {
            resetEncoders();
            driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
            checkEncoders();
            updatePos();
          }

          //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

          char recieved = Serial.read();
          inData += recieved;
          if (recieved == '\n') {
            break;
          }

          //end loop
        }

        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          sendRobotPos();
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }







      //----- LIVE JOINT JOG  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LJ")
      {
        delay(5);
        Serial.println();


        updatePos();

        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        bool JogInPoc = true;
        Alarm = "0";


        int VStart = inData.indexOf("V");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");


        float Vector = inData.substring(VStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = 100;
        float DCCspd = 100;
        float ACCramp = 100;


        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        inData = ""; // Clear recieved buffer

        while (JogInPoc = true) {

          float J1Angle = JangleIn[0];
          float J2Angle = JangleIn[1];
          float J3Angle = JangleIn[2];
          float J4Angle = JangleIn[3];
          float J5Angle = JangleIn[4];
          float J6Angle = JangleIn[5];
          float xyzuvw_In[6];

          if (Vector == 10) {
            J1Angle = JangleIn[0] - .5;
          }
          if (Vector == 11) {
            J1Angle = JangleIn[0] + .5;
          }

          if (Vector == 20) {
            J2Angle = JangleIn[1] - .5;
          }
          if (Vector == 21) {
            J2Angle = JangleIn[1] + .5;
          }

          if (Vector == 30) {
            J3Angle = JangleIn[2] - .5;
          }
          if (Vector == 31) {
            J3Angle = JangleIn[2] + .5;
          }

          if (Vector == 40) {
            J4Angle = JangleIn[3] - .5;
          }
          if (Vector == 41) {
            J4Angle = JangleIn[3] + .5;
          }

          if (Vector == 50) {
            J5Angle = JangleIn[4] - .5;
          }
          if (Vector == 51) {
            J5Angle = JangleIn[4] + .5;
          }

          if (Vector == 60) {
            J6Angle = JangleIn[5] - .5;
          }
          if (Vector == 61) {
            J6Angle = JangleIn[5] + .5;
          }

          //calc destination motor steps
          int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          //determine motor directions
          if (J1stepDif <= 0) {
            J1dir = 1;
          }
          else {
            J1dir = 0;
          }

          if (J2stepDif <= 0) {
            J2dir = 1;
          }
          else {
            J2dir = 0;
          }

          if (J3stepDif <= 0) {
            J3dir = 1;
          }
          else {
            J3dir = 0;
          }

          if (J4stepDif <= 0) {
            J4dir = 1;
          }
          else {
            J4dir = 0;
          }

          if (J5stepDif <= 0) {
            J5dir = 1;
          }
          else {
            J5dir = 0;
          }

          if (J6stepDif <= 0) {
            J6dir = 1;
          }
          else {
            J6dir = 0;
          }

          if (TRstepDif <= 0) {
            TRdir = 1;
          }
          else {
            TRdir = 0;
          }

          //determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
            J6axisFault = 1;
          }
          if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0))) {
            TRaxisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;

          //send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0) {
            resetEncoders();
            driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
            checkEncoders();
            updatePos();
          }

          //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

          char recieved = Serial.read();
          inData += recieved;
          if (recieved == '\n') {
            break;
          }

          //end loop
        }

        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          sendRobotPos();
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }








      //----- LIVE TOOL JOG  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LT")
      {
        delay(5);
        Serial.println();


        updatePos();

        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        float Xtool = Robot_Kin_Tool[0];
        float Ytool = Robot_Kin_Tool[1];
        float Ztool = Robot_Kin_Tool[2];
        float RZtool = Robot_Kin_Tool[3];
        float RYtool = Robot_Kin_Tool[4];
        float RXtool = Robot_Kin_Tool[5];

        bool JogInPoc = true;
        Alarm = "0";


        int VStart = inData.indexOf("V");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");


        float Vector = inData.substring(VStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = 100;
        float DCCspd = 100;
        float ACCramp = 100;


        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        inData = ""; // Clear recieved buffer

        while (JogInPoc = true) {

          Xtool = Robot_Kin_Tool[0];
          Ytool = Robot_Kin_Tool[1];
          Ztool = Robot_Kin_Tool[2];
          RXtool = Robot_Kin_Tool[3];
          RYtool = Robot_Kin_Tool[4];
          RZtool = Robot_Kin_Tool[5];

          if (Vector == 10) {
            Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - 1;
          }
          if (Vector == 11) {
            Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + 1;
          }

          if (Vector == 20) {
            Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - 1;
          }
          if (Vector == 21) {
            Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + 1;
          }

          if (Vector == 30) {
            Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - 1;
          }
          if (Vector == 31) {
            Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + 1;
          }

          if (Vector == 60) {
            Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - 1 * M_PI / 180;
          }
          if (Vector == 61) {
            Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + 1 * M_PI / 180;
          }

          if (Vector == 50) {
            Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - 1 * M_PI / 180;
          }
          if (Vector == 51) {
            Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + 1 * M_PI / 180;
          }

          if (Vector == 40) {
            Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - 1 * M_PI / 180;
          }
          if (Vector == 41) {
            Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + 1 * M_PI / 180;
          }

          JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
          JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
          JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
          JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
          JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
          JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

          xyzuvw_In[0] = xyzuvw_Out[0];
          xyzuvw_In[1] = xyzuvw_Out[1];
          xyzuvw_In[2] = xyzuvw_Out[2];
          xyzuvw_In[3] = xyzuvw_Out[3];
          xyzuvw_In[4] = xyzuvw_Out[4];
          xyzuvw_In[5] = xyzuvw_Out[5];

          SolveInverseKinematic();

          Robot_Kin_Tool[0] = Xtool;
          Robot_Kin_Tool[1] = Ytool;
          Robot_Kin_Tool[2] = Ztool;
          Robot_Kin_Tool[3] = RXtool;
          Robot_Kin_Tool[4] = RYtool;
          Robot_Kin_Tool[5] = RZtool;

          //calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          //determine motor directions
          if (J1stepDif <= 0) {
            J1dir = 1;
          }
          else {
            J1dir = 0;
          }

          if (J2stepDif <= 0) {
            J2dir = 1;
          }
          else {
            J2dir = 0;
          }

          if (J3stepDif <= 0) {
            J3dir = 1;
          }
          else {
            J3dir = 0;
          }

          if (J4stepDif <= 0) {
            J4dir = 1;
          }
          else {
            J4dir = 0;
          }

          if (J5stepDif <= 0) {
            J5dir = 1;
          }
          else {
            J5dir = 0;
          }

          if (J6stepDif <= 0) {
            J6dir = 1;
          }
          else {
            J6dir = 0;
          }

          if (TRstepDif <= 0) {
            TRdir = 1;
          }
          else {
            TRdir = 0;
          }


          //determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
            J6axisFault = 1;
          }
          if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0))) {
            TRaxisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;


          //send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0) {
            resetEncoders();
            driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
            checkEncoders();
            updatePos();
          }

          //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

          char recieved = Serial.read();
          inData += recieved;
          if (recieved == '\n') {
            break;
          }

          //end loop
        }

        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          sendRobotPos();
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }










      //----- MOVE J IN JOINTS  ---------------------------------------------------
      //-----------------------------------------------------------------------

      if (function == "RJ") {
        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        int J1stepStart = inData.indexOf("A");
        int J2stepStart = inData.indexOf("B");
        int J3stepStart = inData.indexOf("C");
        int J4stepStart = inData.indexOf("D");
        int J5stepStart = inData.indexOf("E");
        int J6stepStart = inData.indexOf("F");
        int tStart = inData.indexOf("G");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        float J1Angle;
        float J2Angle;
        float J3Angle;
        float J4Angle;
        float J5Angle;
        float J6Angle;
        float xyzuvw_In[6];
        float railpos;

        J1Angle = inData.substring(J1stepStart + 1, J2stepStart).toFloat();
        J2Angle = inData.substring(J2stepStart + 1, J3stepStart).toFloat();
        J3Angle = inData.substring(J3stepStart + 1, J4stepStart).toFloat();
        J4Angle = inData.substring(J4stepStart + 1, J5stepStart).toFloat();
        J5Angle = inData.substring(J5stepStart + 1, J6stepStart).toFloat();
        J6Angle = inData.substring(J6stepStart + 1, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();


        int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
        int TRfutStepM = (xyzuvw_In[6] + TRaxisLimNeg) * TRStepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int TRstepDif = TRStepM - TRfutStepM;


        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        if (TRstepDif <= 0) {
          TRdir = 1;
        }
        else {
          TRdir = 0;
        }


        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0))) {
          TRaxisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          checkEncoders();
          sendRobotPos();
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }


        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////

      }








      //----- Jog T ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "JT")
      {
        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        float Xtool = Robot_Kin_Tool[0];
        float Ytool = Robot_Kin_Tool[1];
        float Ztool = Robot_Kin_Tool[2];
        float RZtool = Robot_Kin_Tool[3];
        float RYtool = Robot_Kin_Tool[4];
        float RXtool = Robot_Kin_Tool[5];

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TotalAxisFault = 0;

        String Alarm = "0";

        int SPstart = inData.indexOf('S');
        int AcStart = inData.indexOf('G');
        int DcStart = inData.indexOf('H');
        int RmStart = inData.indexOf('I');

        String Dir = inData.substring(0, 2); // this should be Z0 or Z1
        float Dist = inData.substring(2, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 3, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 1, DcStart).toInt();
        float DCCspd = inData.substring(DcStart + 1, RmStart).toInt();
        float ACCramp = inData.substring(RmStart + 1).toInt();



        if (Dir == "X0") {
          Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + Dist;
        }
        else if (Dir == "X1") {
          Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - Dist;
        }
        else if (Dir == "Y0") {
          Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + Dist;
        }
        else if (Dir == "Y1") {
          Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - Dist;
        }
        else if (Dir == "Z0") {
          Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + Dist;
        }
        else if (Dir == "Z1") {
          Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - Dist;
        }
        else if (Dir == "R0") {
          Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + Dist * M_PI / 180;
        }
        else if (Dir == "R1") {
          Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - Dist * M_PI / 180;
        }
        else if (Dir == "P0") {
          Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + Dist * M_PI / 180;
        }
        else if (Dir == "P1") {
          Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - Dist * M_PI / 180;
        }
        else if (Dir == "W0") {
          Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + Dist * M_PI / 180;
        }
        else if (Dir == "W1") {
          Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - Dist * M_PI / 180;
        }


        JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
        JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
        JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
        JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
        JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
        JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

        xyzuvw_In[0] = xyzuvw_Out[0];
        xyzuvw_In[1] = xyzuvw_Out[1];
        xyzuvw_In[2] = xyzuvw_Out[2];
        xyzuvw_In[3] = xyzuvw_Out[3];
        xyzuvw_In[4] = xyzuvw_Out[4];
        xyzuvw_In[5] = xyzuvw_Out[5];

        SolveInverseKinematic();

        Robot_Kin_Tool[0] = Xtool;
        Robot_Kin_Tool[1] = Ytool;
        Robot_Kin_Tool[2] = Ztool;
        Robot_Kin_Tool[3] = RZtool;
        Robot_Kin_Tool[4] = RYtool;
        Robot_Kin_Tool[5] = RXtool;


        //calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int TRstepDif = 0;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        TRdir = 0;

        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          checkEncoders();
          sendRobotPos();
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          delay(5);
          Serial.println(Alarm);
        }



        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }







      //----- MOVE A (Arc) ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MA") {

        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TotalAxisFault = 0;

        String Alarm = "0";
        float curWayDis;
        float speedSP;
        float Xvect;
        float Yvect;
        float Zvect;
        float calcStepGap;
        float theta;
        float axis [3];
        float axisTemp [3];
        float startVect [3];
        float Rotation [3][3];
        float DestPt [3];
        float a;
        float b;
        float c;
        float d;
        float aa;
        float bb;
        float cc;
        float dd;
        float bc;
        float ad;
        float ac;
        float ab;
        float bd;
        float cd;

        int xMidIndex = inData.indexOf("X");
        int yMidIndex = inData.indexOf("Y");
        int zMidIndex = inData.indexOf("Z");
        int rzIndex = inData.indexOf("Rz");
        int ryIndex = inData.indexOf("Ry");
        int rxIndex = inData.indexOf("Rx");

        int xEndIndex = inData.indexOf("Ex");
        int yEndIndex = inData.indexOf("Ey");
        int zEndIndex = inData.indexOf("Ez");
        int tStart = inData.indexOf("Tr");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        updatePos();

        float xBeg = xyzuvw_Out[0];
        float yBeg = xyzuvw_Out[1];
        float zBeg = xyzuvw_Out[2];
        float rzBeg = xyzuvw_Out[3];
        float ryBeg = xyzuvw_Out[4];
        float rxBeg = xyzuvw_Out[5];


        float xMid = inData.substring(xMidIndex + 1, yMidIndex).toFloat();
        float yMid = inData.substring(yMidIndex + 1, zMidIndex).toFloat();
        float zMid = inData.substring(zMidIndex + 1, rzIndex).toFloat();

        float rz = inData.substring(rzIndex + 2, ryIndex).toFloat();
        float ry = inData.substring(ryIndex + 2, rxIndex).toFloat();
        float rx = inData.substring(rxIndex + 2, xEndIndex).toFloat();

        float RZvect = rzBeg - rz;
        float RYvect = ryBeg - ry;
        float RXvect = rxBeg - rx;

        float xEnd = inData.substring(xEndIndex + 2, yEndIndex).toFloat();
        float yEnd = inData.substring(yEndIndex + 2, zEndIndex).toFloat();
        float zEnd = inData.substring(zEndIndex + 2, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        //determine length between each point (lengths of triangle)
        Xvect = xEnd - xMid;
        Yvect = yEnd - yMid;
        Zvect = zEnd - zMid;
        float aDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
        Xvect = xEnd - xBeg;
        Yvect = yEnd - yBeg;
        Zvect = zEnd - zBeg;
        float bDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
        Xvect = xMid - xBeg;
        Yvect = yMid - yBeg;
        Zvect = zMid - zBeg;
        float cDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
        //use lengths between each point (lengths of triangle) to determine radius
        float s = (aDist + bDist + cDist) / 2;
        float Radius = aDist * bDist * cDist / 4 / sqrt(s * (s - aDist) * (s - bDist) * (s - cDist));
        //find barycentric coordinates of triangle (center of triangle)
        float BCx = pow(aDist, 2) * (pow(bDist, 2) + pow(cDist, 2) - pow(aDist, 2));
        float BCy = pow(bDist, 2) * (pow(cDist, 2) + pow(aDist, 2) - pow(bDist, 2));
        float BCz = pow(cDist, 2) * (pow(aDist, 2) + pow(bDist, 2) - pow(cDist, 2));
        //find center coordinates of circle - convert barycentric coordinates to cartesian coordinates - dot product of 3 points and barycentric coordiantes divided by sum of barycentric coordinates
        float Px = ((BCx * xBeg) + (BCy * xMid) + (BCz * xEnd)) / (BCx + BCy + BCz) ;
        float Py = ((BCx * yBeg) + (BCy * yMid) + (BCz * yEnd)) / (BCx + BCy + BCz) ;
        float Pz = ((BCx * zBeg) + (BCy * zMid) + (BCz * zEnd)) / (BCx + BCy + BCz) ;
        //define start vetor
        startVect [0] = (xBeg - Px);
        startVect [1] = (yBeg - Py);
        startVect [2] = (zBeg - Pz);
        //get 3 vectors from center of circle to begining target, mid target and end target then normalize
        float vect_Amag = pow((pow((xBeg - Px), 2) + pow((yBeg - Py), 2) + pow((zBeg - Pz), 2)), .5);
        float vect_Ax = (xBeg - Px) / vect_Amag;
        float vect_Ay = (yBeg - Py) / vect_Amag;
        float vect_Az = (zBeg - Pz) / vect_Amag;
        float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
        float vect_Bx = (xMid - Px) / vect_Bmag;
        float vect_By = (yMid - Py) / vect_Bmag;
        float vect_Bz = (zMid - Pz) / vect_Bmag;
        float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
        float vect_Cx = (xEnd - Px) / vect_Cmag;
        float vect_Cy = (yEnd - Py) / vect_Cmag;
        float vect_Cz = (zEnd - Pz) / vect_Cmag;
        //get cross product of vectors a & c than apply to axis matrix
        float CrossX = (vect_Ay * vect_Bz) - (vect_Az * vect_By);
        float CrossY = (vect_Az * vect_Bx) - (vect_Ax * vect_Bz);
        float CrossZ = (vect_Ax * vect_By) - (vect_Ay * vect_Bx);
        axis [0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
        axis [1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
        axis [2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
        //get radian angle between vectors using acos of dot product
        float ABradians = acos((vect_Ax * vect_Bx + vect_Ay * vect_By + vect_Az * vect_Bz) / (sqrt(pow(vect_Ax , 2) + pow(vect_Ay , 2) + pow(vect_Az , 2)) * sqrt(pow(vect_Bx , 2) + pow(vect_By , 2) + pow(vect_Bz , 2)))  );
        float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx , 2) + pow(vect_By , 2) + pow(vect_Bz , 2)) * sqrt(pow(vect_Cx , 2) + pow(vect_Cy , 2) + pow(vect_Cz , 2)))  );
        //get total degrees of both arcs
        float ABdegrees = degrees(ABradians + BCradians);
        //get arc length and calc way pt gap

        float anglepercent = ABdegrees / 360;
        float circumference = 2 * 3.14159265359 * Radius;
        float lineDist = circumference * anglepercent;
        float wayPts = lineDist / linWayDistSP;

        float wayPerc = 1 / wayPts;
        //cacl way pt angle
        float theta_Deg = (ABdegrees / wayPts);

        //determine steps
        int HighStep = lineDist / .05;
        float ACCStep = HighStep * (ACCspd / 100);
        float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
        float DCCStep = HighStep * (DCCspd / 100);

        //set speed for seconds or mm per sec
        if (SpeedType == "s") {
          speedSP = (SpeedVal * 1000000) * .2;
        }
        else if (SpeedType == "m") {
          speedSP = ((lineDist / SpeedVal) * 1000000) * .2;
        }

        //calc step gap for seconds or mm per sec
        if (SpeedType == "s" or SpeedType == "m" ) {
          float zeroStepGap = speedSP / HighStep;
          float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
          float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
          float zeroNORtime = NORStep * zeroStepGap;
          float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
          float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
          float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
          float overclockPerc = speedSP / zeroTOTtime;
          calcStepGap = zeroStepGap * overclockPerc;
          if (calcStepGap <= minSpeedDelay) {
            calcStepGap = minSpeedDelay;
            speedViolation = "1";
          }
        }

        //calc step gap for percentage
        else if (SpeedType == "p") {
          calcStepGap = ((maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay)) + minSpeedDelay);
        }

        //calculate final step increments
        float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
        float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
        float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
        float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


        //calc way pt speeds
        float ACCwayPts = wayPts * (ACCspd / 100);
        float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
        float DCCwayPts = wayPts * (DCCspd / 100);

        //calc way inc for lin way steps
        float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
        float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

        //set starting delsy
        float curDelay = calcACCstartDel;

        //set starting angle first way pt
        float cur_deg = theta_Deg;

        /////////////////////////////////////
        //loop through waypoints
        ////////////////////////////////////

        resetEncoders();

        for (int i = 0; i <= wayPts - 1; i++) {

          theta = radians(cur_deg);
          //use euler rodrigues formula to find rotation vector
          a = cos(theta / 2.0);
          b = -axis [0] * sin(theta / 2.0);
          c = -axis [1] * sin(theta / 2.0);
          d = -axis [2] * sin(theta / 2.0);
          aa = a * a;
          bb = b * b;
          cc = c * c;
          dd = d * d;
          bc = b * c;
          ad = a * d;
          ac = a * c;
          ab = a * b;
          bd = b * d;
          cd = c * d;
          Rotation [0][0] = aa + bb - cc - dd;
          Rotation [0][1] = 2 * (bc + ad);
          Rotation [0][2] = 2 * (bd - ac);
          Rotation [1][0] = 2 * (bc - ad);
          Rotation [1][1] = aa + cc - bb - dd;
          Rotation [1][2] = 2 * (cd + ab);
          Rotation [2][0] = 2 * (bd + ac);
          Rotation [2][1] = 2 * (cd - ab);
          Rotation [2][2] = aa + dd - bb - cc;

          //get product of current rotation and start vector
          DestPt[0] = (Rotation [0][0] * startVect[0]) + (Rotation [0][1] * startVect[1]) + (Rotation [0][2] * startVect[2]);
          DestPt[1] = (Rotation [1][0] * startVect[0]) + (Rotation [1][1] * startVect[1]) + (Rotation [1][2] * startVect[2]);
          DestPt[2] = (Rotation [2][0] * startVect[0]) + (Rotation [2][1] * startVect[1]) + (Rotation [2][2] * startVect[2]);

          ////DELAY CALC/////
          if (i <= ACCwayPts) {
            curDelay = curDelay - (ACCwayInc);
          }
          else if (i >= (wayPts - DCCwayPts)) {
            curDelay = curDelay + (DCCwayInc);
          }
          else {
            curDelay = calcStepGap;
          }

          //shift way pts back to orignal origin and calc kinematics for way pt movement
          float curWayPerc = wayPerc * i;
          xyzuvw_In[0] = (DestPt[0]) + Px;
          xyzuvw_In[1] = (DestPt[1]) + Py;
          xyzuvw_In[2] = (DestPt[2]) + Pz;
          xyzuvw_In[3] = rzBeg - (RZvect * curWayPerc);
          xyzuvw_In[4] = ryBeg - (RYvect * curWayPerc);
          xyzuvw_In[5] = rxBeg - (RXvect * curWayPerc);


          SolveInverseKinematic();

          //calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          //determine motor directions
          if (J1stepDif <= 0) {
            J1dir = 1;
          }
          else {
            J1dir = 0;
          }

          if (J2stepDif <= 0) {
            J2dir = 1;
          }
          else {
            J2dir = 0;
          }

          if (J3stepDif <= 0) {
            J3dir = 1;
          }
          else {
            J3dir = 0;
          }

          if (J4stepDif <= 0) {
            J4dir = 1;
          }
          else {
            J4dir = 0;
          }

          if (J5stepDif <= 0) {
            J5dir = 1;
          }
          else {
            J5dir = 0;
          }

          if (J6stepDif <= 0) {
            J6dir = 1;
          }
          else {
            J6dir = 0;
          }

          TRdir = 0;

          //determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
            J6axisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


          //send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0) {
            driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, curDelay);
          }
          else if (KinematicError == 1) {
            Alarm = "ER";
            delay(5);
            Serial.println(Alarm);
          }
          else {
            Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
            delay(5);
            Serial.println(Alarm);
          }

          //increment angle
          cur_deg += theta_Deg;

        }
        checkEncoders();
        sendRobotPos();


        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }














      //----- MOVE C (Cirlce) ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MC") {

        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TotalAxisFault = 0;

        String Alarm = "0";
        float curWayDis;
        float speedSP;
        float Xvect;
        float Yvect;
        float Zvect;
        float calcStepGap;
        float theta;
        int Cdir;
        float axis [3];
        float axisTemp [3];
        float startVect [3];
        float Rotation [3][3];
        float DestPt [3];
        float a;
        float b;
        float c;
        float d;
        float aa;
        float bb;
        float cc;
        float dd;
        float bc;
        float ad;
        float ac;
        float ab;
        float bd;
        float cd;

        int xStart = inData.indexOf("Cx");
        int yStart = inData.indexOf("Cy");
        int zStart = inData.indexOf("Cz");
        int rzStart = inData.indexOf("Rz");
        int ryStart = inData.indexOf("Ry");
        int rxStart = inData.indexOf("Rx");
        int xMidIndex = inData.indexOf("Bx");
        int yMidIndex = inData.indexOf("By");
        int zMidIndex = inData.indexOf("Bz");
        int xEndIndex = inData.indexOf("Px");
        int yEndIndex = inData.indexOf("Py");
        int zEndIndex = inData.indexOf("Pz");
        int tStart = inData.indexOf("Tr");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        float xBeg = inData.substring(xStart + 2, yStart).toFloat();
        float yBeg = inData.substring(yStart + 2, zStart).toFloat();
        float zBeg = inData.substring(zStart + 2, rzStart).toFloat();
        float rzBeg = inData.substring(rzStart + 2, ryStart).toFloat();
        float ryBeg = inData.substring(ryStart + 2, rxStart).toFloat();
        float rxBeg = inData.substring(rxStart + 2, xMidIndex).toFloat();
        float xMid = inData.substring(xMidIndex + 2, yMidIndex).toFloat();
        float yMid = inData.substring(yMidIndex + 2, zMidIndex).toFloat();
        float zMid = inData.substring(zMidIndex + 2, xEndIndex).toFloat();
        float xEnd = inData.substring(xEndIndex + 2, yEndIndex).toFloat();
        float yEnd = inData.substring(yEndIndex + 2, zEndIndex).toFloat();
        float zEnd = inData.substring(zEndIndex + 2, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        //calc vector from start point of circle (mid) to center of circle (beg)
        Xvect = xMid - xBeg;
        Yvect = yMid - yBeg;
        Zvect = zMid - zBeg;
        //get radius - distance from first point (center of circle) to second point (start point of circle)
        float Radius = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);

        //set center coordinates of circle to first point (beg) as this is the center of our circle
        float Px = xBeg ;
        float Py = yBeg ;
        float Pz = zBeg ;

        //define start vetor (mid) point is start of circle
        startVect [0] = (xMid - Px);
        startVect [1] = (yMid - Py);
        startVect [2] = (zMid - Pz);
        //get vectors from center of circle to  mid target (start) and end target then normalize
        float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
        float vect_Bx = (xMid - Px) / vect_Bmag;
        float vect_By = (yMid - Py) / vect_Bmag;
        float vect_Bz = (zMid - Pz) / vect_Bmag;
        float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
        float vect_Cx = (xEnd - Px) / vect_Cmag;
        float vect_Cy = (yEnd - Py) / vect_Cmag;
        float vect_Cz = (zEnd - Pz) / vect_Cmag;
        //get cross product of vectors b & c than apply to axis matrix
        float CrossX = (vect_By * vect_Cz) - (vect_Bz * vect_Cy);
        float CrossY = (vect_Bz * vect_Cx) - (vect_Bx * vect_Cz);
        float CrossZ = (vect_Bx * vect_Cy) - (vect_By * vect_Cx);
        axis [0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
        axis [1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
        axis [2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
        //get radian angle between vectors using acos of dot product
        float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx , 2) + pow(vect_Cy , 2) + pow(vect_Bz , 2)) * sqrt(pow(vect_Cx , 2) + pow(vect_Cy , 2) + pow(vect_Cz , 2)))  );
        //get arc degree
        float ABdegrees = degrees(BCradians);
        //get direction from angle
        if (ABdegrees > 0) {
          Cdir = 1;
        }
        else {
          Cdir = -1;
        }

        //get circumference and calc way pt gap
        float lineDist = 2 * 3.14159265359 * Radius;
        float wayPts = lineDist / linWayDistSP;

        float wayPerc = 1 / wayPts;
        //cacl way pt angle
        float theta_Deg = ((360 * Cdir) / (wayPts));

        //determine steps
        int HighStep = lineDist / .05;
        float ACCStep = HighStep * (ACCspd / 100);
        float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
        float DCCStep = HighStep * (DCCspd / 100);

        //set speed for seconds or mm per sec
        if (SpeedType == "s") {
          speedSP = (SpeedVal * 1000000) * .2;
        }
        else if (SpeedType == "m") {
          speedSP = ((lineDist / SpeedVal) * 1000000) * .2;
        }

        //calc step gap for seconds or mm per sec
        if (SpeedType == "s" or SpeedType == "m" ) {
          float zeroStepGap = speedSP / HighStep;
          float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
          float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
          float zeroNORtime = NORStep * zeroStepGap;
          float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
          float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
          float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
          float overclockPerc = speedSP / zeroTOTtime;
          calcStepGap = zeroStepGap * overclockPerc;
          if (calcStepGap <= minSpeedDelay) {
            calcStepGap = minSpeedDelay;
            speedViolation = "1";
          }
        }

        //calc step gap for percentage
        else if (SpeedType == "p") {
          calcStepGap = ((maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay)) + minSpeedDelay);
        }

        //calculate final step increments
        float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
        float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
        float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
        float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


        //calc way pt speeds
        float ACCwayPts = wayPts * (ACCspd / 100);
        float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
        float DCCwayPts = wayPts * (DCCspd / 100);

        //calc way inc for lin way steps
        float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
        float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

        //set starting delsy
        float curDelay = calcACCstartDel;

        //set starting angle first way pt
        float cur_deg = theta_Deg;

        /////////////////////////////////////
        //loop through waypoints
        ////////////////////////////////////

        resetEncoders();

        for (int i = 1; i <= wayPts; i++) {

          theta = radians(cur_deg);
          //use euler rodrigues formula to find rotation vector
          a = cos(theta / 2.0);
          b = -axis [0] * sin(theta / 2.0);
          c = -axis [1] * sin(theta / 2.0);
          d = -axis [2] * sin(theta / 2.0);
          aa = a * a;
          bb = b * b;
          cc = c * c;
          dd = d * d;
          bc = b * c;
          ad = a * d;
          ac = a * c;
          ab = a * b;
          bd = b * d;
          cd = c * d;
          Rotation [0][0] = aa + bb - cc - dd;
          Rotation [0][1] = 2 * (bc + ad);
          Rotation [0][2] = 2 * (bd - ac);
          Rotation [1][0] = 2 * (bc - ad);
          Rotation [1][1] = aa + cc - bb - dd;
          Rotation [1][2] = 2 * (cd + ab);
          Rotation [2][0] = 2 * (bd + ac);
          Rotation [2][1] = 2 * (cd - ab);
          Rotation [2][2] = aa + dd - bb - cc;

          //get product of current rotation and start vector
          DestPt[0] = (Rotation [0][0] * startVect[0]) + (Rotation [0][1] * startVect[1]) + (Rotation [0][2] * startVect[2]);
          DestPt[1] = (Rotation [1][0] * startVect[0]) + (Rotation [1][1] * startVect[1]) + (Rotation [1][2] * startVect[2]);
          DestPt[2] = (Rotation [2][0] * startVect[0]) + (Rotation [2][1] * startVect[1]) + (Rotation [2][2] * startVect[2]);

          ////DELAY CALC/////
          if (i <= ACCwayPts) {
            curDelay = curDelay - (ACCwayInc);
          }
          else if (i >= (wayPts - DCCwayPts)) {
            curDelay = curDelay + (DCCwayInc);
          }
          else {
            curDelay = calcStepGap;
          }

          //shift way pts back to orignal origin and calc kinematics for way pt movement
          xyzuvw_In[0] = (DestPt[0]) + Px;
          xyzuvw_In[1] = (DestPt[1]) + Py;
          xyzuvw_In[2] = (DestPt[2]) + Pz;
          xyzuvw_In[3] = rzBeg;
          xyzuvw_In[4] = ryBeg;
          xyzuvw_In[5] = rxBeg;

          SolveInverseKinematic();

          //calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          //determine motor directions
          if (J1stepDif <= 0) {
            J1dir = 1;
          }
          else {
            J1dir = 0;
          }

          if (J2stepDif <= 0) {
            J2dir = 1;
          }
          else {
            J2dir = 0;
          }

          if (J3stepDif <= 0) {
            J3dir = 1;
          }
          else {
            J3dir = 0;
          }

          if (J4stepDif <= 0) {
            J4dir = 1;
          }
          else {
            J4dir = 0;
          }

          if (J5stepDif <= 0) {
            J5dir = 1;
          }
          else {
            J5dir = 0;
          }

          if (J6stepDif <= 0) {
            J6dir = 1;
          }
          else {
            J6dir = 0;
          }

          TRdir = 0;

          //determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
            J6axisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;



          if (TotalAxisFault == 0 && KinematicError == 0) {
            driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, curDelay);
          }
          else if (KinematicError == 1) {
            Alarm = "ER";
            delay(5);
            Serial.println(Alarm);
          }
          else {
            Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
            delay(5);
            Serial.println(Alarm);
          }


          //increment angle
          cur_deg += theta_Deg;

        }

        checkEncoders();
        sendRobotPos();


        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }














      //----- MOVE L ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ML")
      {
        int J1dir;
        int J2dir;
        int J3dir;
        int J4dir;
        int J5dir;
        int J6dir;
        int TRdir;

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        String Alarm = "0";

        float curWayDis;
        float speedSP;

        int xStart = inData.indexOf("X");
        int yStart = inData.indexOf("Y");
        int zStart = inData.indexOf("Z");
        int rzStart = inData.indexOf("Rz");
        int ryStart = inData.indexOf("Ry");
        int rxStart = inData.indexOf("Rx");
        int tStart = inData.indexOf("Tr");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
        xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
        xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
        xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
        xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
        xyzuvw_In[5] = inData.substring(rxStart + 2, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();

        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();

        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();


        //vector
        float Xvect = xyzuvw_In[0] - xyzuvw_Out[0];
        float Yvect = xyzuvw_In[1] - xyzuvw_Out[1];
        float Zvect = xyzuvw_In[2] - xyzuvw_Out[2];
        float RZvect = xyzuvw_In[3] - xyzuvw_Out[3];
        float RYvect = xyzuvw_In[4] - xyzuvw_Out[4];
        float RXvect = xyzuvw_In[5] - xyzuvw_Out[5];


        //start pos
        float Xstart = xyzuvw_Out[0];
        float Ystart = xyzuvw_Out[1];
        float Zstart = xyzuvw_Out[2];
        float RZstart = xyzuvw_Out[3];
        float RYstart = xyzuvw_Out[4];
        float RXstart = xyzuvw_Out[5];




        //line dist and determine way point gap
        float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        if (lineDist > 0) {


          float wayPts = lineDist / linWayDistSP;
          float wayPerc =  1 / wayPts;



          //pre calculate entire move and speeds

          SolveInverseKinematic();
          //calc destination motor steps for precalc
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination fpr precalc
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;

          //FIND HIGHEST STEP FOR PRECALC
          int HighStep = J1stepDif;
          if (J2stepDif > HighStep)
          {
            HighStep = J2stepDif;
          }
          if (J3stepDif > HighStep)
          {
            HighStep = J3stepDif;
          }
          if (J4stepDif > HighStep)
          {
            HighStep = J4stepDif;
          }
          if (J5stepDif > HighStep)
          {
            HighStep = J5stepDif;
          }
          if (J6stepDif > HighStep)
          {
            HighStep = J6stepDif;
          }


          /////PRE CALC SPEEDS//////
          float calcStepGap;

          //determine steps
          float ACCStep = HighStep * (ACCspd / 100);
          float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
          float DCCStep = HighStep * (DCCspd / 100);

          //set speed for seconds or mm per sec
          if (SpeedType == "s") {
            speedSP = (SpeedVal * 1000000) * .2;
          }
          else if ((SpeedType == "m")) {
            speedSP = ((lineDist / SpeedVal) * 1000000) * .2;
          }

          //calc step gap for seconds or mm per sec
          if (SpeedType == "s" or SpeedType == "m" ) {
            float zeroStepGap = speedSP / HighStep;
            float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
            float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
            float zeroNORtime = NORStep * zeroStepGap;
            float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
            float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
            float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
            float overclockPerc = speedSP / zeroTOTtime;
            calcStepGap = zeroStepGap * overclockPerc;
            if (calcStepGap <= minSpeedDelay) {
              calcStepGap = minSpeedDelay;
              speedViolation = "1";
            }
          }

          //calc step gap for percentage
          else if (SpeedType == "p") {
            calcStepGap = ((maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay)) + minSpeedDelay);
          }

          //calculate final step increments
          float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
          float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
          float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
          float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


          //calc way pt speeds
          float ACCwayPts = wayPts * (ACCspd / 100);
          float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
          float DCCwayPts = wayPts * (DCCspd / 100);

          //calc way inc for lin way steps
          float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
          float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

          //set starting delsy
          float curDelay = calcACCstartDel;

          // calc track way pt moves
          int TRfutStepM = (xyzuvw_In[6] + TRaxisLimNeg) * TRStepDeg;
          int TRstepDif = (TRStepM - TRfutStepM) / (wayPts - 1);
          if (TRstepDif <= 0) {
            TRdir = 1;
          }
          else {
            TRdir = 0;
          }

          resetEncoders();
          /////////////////////////////////////////////////
          //loop through waypoints
          for (int i = 1; i <= wayPts; i++) {

            ////DELAY CALC/////
            if (i <= ACCwayPts) {
              curDelay = curDelay - (ACCwayInc);
            }
            else if (i >= (wayPts - DCCwayPts)) {
              curDelay = curDelay + (DCCwayInc);
            }
            else {
              curDelay = calcStepGap;
            }

            if (debugg == 1) {
              curDelay = 0;
            }

            float curWayPerc = wayPerc * i;
            xyzuvw_In[0] = Xstart + (Xvect * curWayPerc);
            xyzuvw_In[1] = Ystart + (Yvect * curWayPerc);
            xyzuvw_In[2] = Zstart + (Zvect * curWayPerc);
            xyzuvw_In[3] = RZstart + (RZvect * curWayPerc);
            xyzuvw_In[4] = RYstart + (RYvect * curWayPerc);
            xyzuvw_In[5] = RXstart + (RXvect * curWayPerc);


            SolveInverseKinematic();

            //calc destination motor steps
            int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
            int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
            int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
            int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
            int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
            int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;




            //calc delta from current to destination
            int J1stepDif = J1StepM - J1futStepM;
            int J2stepDif = J2StepM - J2futStepM;
            int J3stepDif = J3StepM - J3futStepM;
            int J4stepDif = J4StepM - J4futStepM;
            int J5stepDif = J5StepM - J5futStepM;
            int J6stepDif = J6StepM - J6futStepM;

            //determine motor directions
            if (J1stepDif <= 0) {
              J1dir = 1;
            }
            else {
              J1dir = 0;
            }

            if (J2stepDif <= 0) {
              J2dir = 1;
            }
            else {
              J2dir = 0;
            }

            if (J3stepDif <= 0) {
              J3dir = 1;
            }
            else {
              J3dir = 0;
            }

            if (J4stepDif <= 0) {
              J4dir = 1;
            }
            else {
              J4dir = 0;
            }

            if (J5stepDif <= 0) {
              J5dir = 1;
            }
            else {
              J5dir = 0;
            }

            if (J6stepDif <= 0) {
              J6dir = 1;
            }
            else {
              J6dir = 0;
            }



            //determine if requested position is within axis limits
            if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
              J1axisFault = 1;
            }
            if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
              J2axisFault = 1;
            }
            if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
              J3axisFault = 1;
            }
            if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
              J4axisFault = 1;
            }
            if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
              J5axisFault = 1;
            }
            if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
              J6axisFault = 1;
            }
            if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0))) {
              TRaxisFault = 1;
            }
            TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;


            //send move command if no axis limit error
            if (TotalAxisFault == 0 && KinematicError == 0) {
              driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, curDelay);
              updatePos();
            }
            else if (KinematicError == 1) {
              Alarm = "ER";
              delay(5);
              Serial.println(Alarm);
            }
            else {
              Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
              delay(5);
              Serial.println(Alarm);
            }

          }

        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////



        delay(5);
        checkEncoders();
        sendRobotPos();
      }



      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}

bool initStateTraj(String inData)
{
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  String softwareVersion = inData.substring(idxVersion + 1, inData.length());
  //Serial.println(softwareVersion);
  //Serial.println(VERSION);
  int versionMatches = (softwareVersion == VERSION);

  // return acknowledgement with result
  String msg = String("ST") + String("A") + String(versionMatches) + String("B") + String(VERSION) + String("\n");
  Serial.print(msg);

  return versionMatches ? true : false;
}

void readEncPos(int* encPos)
{
  encPos[0] = J1encPos.read() * ENC_DIR[0];
  encPos[1] = J2encPos.read() * ENC_DIR[1];
  encPos[2] = J3encPos.read() * ENC_DIR[2];
  encPos[3] = J4encPos.read() * ENC_DIR[3];
  encPos[4] = J5encPos.read() * ENC_DIR[4];
  encPos[5] = J6encPos.read() * ENC_DIR[5];
}

void updateStepperSpeed(String inData)
{
  int idxSpeedJ1 = inData.indexOf('A');
  int idxAccelJ1 = inData.indexOf('B');
  int idxSpeedJ2 = inData.indexOf('C');
  int idxAccelJ2 = inData.indexOf('D');
  int idxSpeedJ3 = inData.indexOf('E');
  int idxAccelJ3 = inData.indexOf('F');
  int idxSpeedJ4 = inData.indexOf('G');
  int idxAccelJ4 = inData.indexOf('H');
  int idxSpeedJ5 = inData.indexOf('I');
  int idxAccelJ5 = inData.indexOf('J');
  int idxSpeedJ6 = inData.indexOf('K');
  int idxAccelJ6 = inData.indexOf('L');

  JOINT_MAX_SPEED[0] = inData.substring(idxSpeedJ1 + 1, idxAccelJ1).toFloat();
  JOINT_MAX_ACCEL[0] = inData.substring(idxAccelJ1 + 1, idxSpeedJ2).toFloat();
  JOINT_MAX_SPEED[1] = inData.substring(idxSpeedJ2 + 1, idxAccelJ2).toFloat();
  JOINT_MAX_ACCEL[1] = inData.substring(idxAccelJ2 + 1, idxSpeedJ3).toFloat();
  JOINT_MAX_SPEED[2] = inData.substring(idxSpeedJ3 + 1, idxAccelJ3).toFloat();
  JOINT_MAX_ACCEL[2] = inData.substring(idxAccelJ3 + 1, idxSpeedJ4).toFloat();
  JOINT_MAX_SPEED[3] = inData.substring(idxSpeedJ4 + 1, idxAccelJ4).toFloat();
  JOINT_MAX_ACCEL[3] = inData.substring(idxAccelJ4 + 1, idxSpeedJ5).toFloat();
  JOINT_MAX_SPEED[4] = inData.substring(idxSpeedJ5 + 1, idxAccelJ5).toFloat();
  JOINT_MAX_ACCEL[4] = inData.substring(idxAccelJ5 + 1, idxSpeedJ6).toFloat();
  JOINT_MAX_SPEED[5] = inData.substring(idxSpeedJ6 + 1, idxAccelJ6).toFloat();
  JOINT_MAX_ACCEL[5] = inData.substring(idxAccelJ6 + 1).toFloat();
}

void calibrateJoints(int* calJoints)
{
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    calJointsDone[i] = !calJoints[i];
  }
  
  // first pass of calibration, fast speed
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  while (!calAllDone)
  {
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      // if joint is not calibrated yet
      if (!calJointsDone[i])
      {
        // check limit switches
        if (!reachedLimitSwitch(i))
        {
          // limit switch not reached, continue moving
          stepperJoints[i].runSpeed();
          calAllDone = false;
        }
        else
        {
          // limit switch reached
          stepperJoints[i].setSpeed(0); // redundancy
          calJointsDone[i] = true;
        }   
      }   
    } 
  }
  delay(2000);

  return;
}

bool reachedLimitSwitch(int joint)
{
  int pin = CAL_PINS[joint];
  // check multiple times to deal with noise
  // possibly EMI from motor cables?
  if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
  {
    if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
    {
      if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
      {
        if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
        {
          if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}

void stateTRAJ()
{
  // clear message
  inData = "";

  // initialise joint steps
  int curEncSteps[NUM_JOINTS];
  readEncPos(curEncSteps);

  int cmdEncSteps[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    cmdEncSteps[i] = curEncSteps[i];
  }

  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false); // DM542T CW
    stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
  stepperJoints[0].setPinsInverted(false, false, false); // J4 DM320T CCW
  stepperJoints[3].setPinsInverted(false, false, false); // J4 DM320T CCW

  // start loop
  while (STATE == STATE_TRAJ)
  {
    char received;
    // check for message from host
    if (Serial.available())
    {
      received = Serial.read();
      inData += received;
    }

    // process message when new line character is received
    if (received == '\n')
    {
      String function = inData.substring(0, 2);
      // update trajectory information
      if (function == "MT")
      {
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        delay(1000);
        Serial.print(msg);

        // get new position commands
        int msgIdxJ1 = inData.indexOf('A');
        int msgIdxJ2 = inData.indexOf('B');
        int msgIdxJ3 = inData.indexOf('C');
        int msgIdxJ4 = inData.indexOf('D');
        int msgIdxJ5 = inData.indexOf('E');
        int msgIdxJ6 = inData.indexOf('F');
        cmdEncSteps[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toInt();
        cmdEncSteps[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toInt();
        cmdEncSteps[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toInt();
        cmdEncSteps[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toInt();
        cmdEncSteps[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toInt();
        cmdEncSteps[5] = inData.substring(msgIdxJ6 + 1).toInt();

        // update target joint positions
        readEncPos(curEncSteps);
        for (int i = 0; i < NUM_JOINTS; ++i)
        { 
          curEncSteps[1] = J2encPos.read() * ENC_DIR[1];
          int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
          if (abs(diffEncSteps) > 2)
          {
            int diffMotSteps = diffEncSteps / ENC_MULT[i];
            if (diffMotSteps < MOTOR_STEPS_PER_REV[i])
            {
              // for the last rev of motor, introduce artificial decceleration
              // to help prevent overshoot
              diffMotSteps = diffMotSteps / 2;
            }
            stepperJoints[i].move(diffMotSteps);
            stepperJoints[i].run();
          }
        }
      }
      else if (function == "JC")
      {
        // calibrate joint 6
        int calJoint6[] = { 0, 0, 0, 0, 0, 1 }; // 000001
        calibrateJoints(calJoint6);

        // record encoder steps
        int calStepJ6 = J6encPos.read() * ENC_DIR[5];

        // calibrate joints 1 to 5
        //Serial.println("Calibrating Joint 1 to 5");
        int calJoints[] = { 1, 1, 1, 1, 1, 0 }; // 111110
        calibrateJoints(calJoints);
        //Serial.println("Calibrating Joint 1 to 5 complete");
        
        // record encoder steps
        int calStepJ1 = J1encPos.read() * ENC_DIR[0];
        int calStepJ2 = J2encPos.read() * ENC_DIR[1];
        int calStepJ3 = J3encPos.read() * ENC_DIR[2];
        int calStepJ4 = J4encPos.read() * ENC_DIR[3];
        int calStepJ5 = J5encPos.read() * ENC_DIR[4];

        // if limit switch at lower end, set encoder to 0
        // otherwise set to encoder upper limit
        //J1encPos.write(0); //Because AR4 J1 is reverse direction
        J1encPos.write(ENC_RANGE_STEPS[0]);
        J2encPos.write(0);
        J3encPos.write(ENC_RANGE_STEPS[2]);
        J4encPos.write(0);
        J5encPos.write(0);
        J6encPos.write(ENC_RANGE_STEPS[5]);

        // read current joint positions
        readEncPos(curEncSteps);

        // return to original position
        //Serial.println("Moving To Rest Position");
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
            cmdEncSteps[i] = curEncSteps[i];
            stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
        }

        bool restPosReached = false;
        int countStop = 0;
        int Jprev[] = {100000, 100000, 100000, 100000, 100000, 100000,};
        int SumDiffPrev = 1000000;
        int SumDiff = 0;
        
        while (!restPosReached)
        {
          restPosReached = true;
          // read current joint positions
          readEncPos(curEncSteps);
          //Serial.print("Distance to go ");
          for (int i = 0; i < NUM_JOINTS; ++i)
          {
            if (abs(REST_ENC_POSITIONS[i] - curEncSteps[i]) > 5)
            {
              restPosReached = false;
              stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
              stepperJoints[i].run();
              SumDiff += (REST_ENC_POSITIONS[i] - curEncSteps[i]);
            }
          }

          if (abs(SumDiff-SumDiffPrev) > 20)
          {
            countStop++;
          }
          if (countStop > 100)
          {
            break;
          }
          SumDiffPrev = SumDiff;
        }
        //Serial.println("Rest Position Reached");
        
        // calibration done, send calibration values
        String msg = String("JC") + String("A") + String(calStepJ1) + String("B") + String(calStepJ2) + String("C") + String(calStepJ3)
                  + String("D") + String(calStepJ4) + String("E") + String(calStepJ5) + String("F") + String(calStepJ6) + String("\n");
        Serial.println(msg);

        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
      }
      else if (function == "JP")
      {
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]);
        Serial.println(msg);
      }
      else if (function == "SS")
      {
        updateStepperSpeed(inData);
        // set motor speed and acceleration
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
          MOTOR_MAX_SPEED[i] = JOINT_MAX_SPEED[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
          MOTOR_MAX_ACCEL[i] = JOINT_MAX_ACCEL[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
          stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i] * MOTOR_ACCEL_MULT[i]);
          stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]);
        Serial.println(msg);
      }
      else if (function == "ST")
      {
        if (!initStateTraj(inData))
        {
          STATE = STATE_ARCS;
          return;
        }

      }
      
      // clear message
      inData = "";
    }
    // execute motor commands
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      // target joint positions are already updated, just call run()
      stepperJoints[i].run();
    }
  }
}

void loop(){
  //test traj state
  // STATE = STATE_TRAJ;

  // state control
  switch (STATE)
  {
    case STATE_ARCS:
      stateARCS();
      break;
    case STATE_TRAJ:
      stateTRAJ();
      break;
    // case STATE_ERR:
    //   stateERR();
    //   break;
    default:
      stateARCS();
      break;
  }
}