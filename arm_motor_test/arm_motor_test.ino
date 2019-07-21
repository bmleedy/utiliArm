#include "Arduino.h"
#include "ServoAxis.h"
#include "GearMotorAxis.h"
#include "AxisProtocol.h"

//todo: MECHANICAL  add zero hashmarks indicators on all joints and adaptors
//todo: MECHANICAL  add holes for flange (in addition to existing holes)
//todo: MECHANICAL  add slots/holes for installing the flange
//todo: ELECTRICAL  add buttons (rocker switch) to manually move the 
//todo: ELECTRICAL  add a killswitch to stop motion on the big axes
//todo: MECHANICAL  strengthen feet areas from cracking
//todo: MECHANICAL  "bed" the shaft attachment to secure it
//todo: MECHANICAl  widen screwdriver holes for bet
//todo: MECHANICAL  make room for washers in feet

// Shoulder Axis Constants
#define SHOULDER_AXIS A3
#define SHOULDER_AXIS_MIN -210     // actual limit is 70   //todo: use this
#define SHOULDER_AXIS_MAX  210     // actual limit is TBD  //todo: use this
#define SHOULDER_POSITIVE_PIN 8
#define SHOULDER_NEGATIVE_PIN 9
#define SHOULDER_COUNTS_AT_CENTER 590   // todo: recalibrate this

// Elbow Axis Constants
#define ELBOW_AXIS A2
#define ELBOW_AXIS_MIN  -210      // about 90 degrees 
#define ELBOW_AXIS_MAX   210      // todo: setme
#define ELBOW_NEGATIVE_PIN 7
#define ELBOW_POSITIVE_PIN 6
#define ELBOW_COUNTS_AT_CENTER 486   // todo: recalibrate this

// Elbow axis constants
#define ELBOW_AXIS A2
#define ELBOW_AXIS_MIN  -210      // about 90 degrees 
#define ELBOW_AXIS_MAX   210      // todo: setme
#define ELBOW_NEGATIVE_PIN 7
#define ELBOW_POSITIVE_PIN 6
#define ELBOW_COUNTS_AT_CENTER 486   // todo: recalibrate this

// Wrist bend axis constants
#define WRIST_BEND_AXIS_PIN     3                // digital pin 3 (PWM pin)
#define WRIST_BEND_AXIS_MIN  -900                // 90 degrees
#define WRIST_BEND_AXIS_MAX   900                // 90 degrees
#define WRIST_BEND_AXIS_COUNTS_AT_CENTER   90    //center position for the servo

// Wrist rotation axis constants
#define WRIST_ROT_AXIS_PIN    10                // digital pin 3 (PWM pin)
#define WRIST_ROT_AXIS_MIN  -900                // 90 degrees
#define WRIST_ROT_AXIS_MAX   900                // 90 degrees
#define WRIST_ROT_AXIS_COUNTS_AT_CENTER   90    //center position for the servo

// Wrist rotation axis constants
#define CLAW_AXIS_PIN     5                // digital pin 3 (PWM pin)
#define CLAW_AXIS_MIN  -800                // 90 degrees
#define CLAW_AXIS_MAX   700                // 90 degrees
#define CLAW_AXIS_COUNTS_AT_CENTER   90    //center position for the servo

// Auto pose routine constants
// Poses for the automated routine
struct pose{
  int16_t axis_pos[NUM_AXES];
  uint16_t duration_ms;
};

// Position Routine constants
// todo: save these to EEPROM
// todo: make programming utility
// todo: make teaching utility
#define NUM_TESTPOSES 12
//  shoulder,     elbow,     wrist_bend,       wrist_rot,    claw
const PROGMEM pose testposes[NUM_TESTPOSES] = {
  {.axis_pos={   0 ,    0,    0,    0,    0},.duration_ms= 3000},  //1
  {.axis_pos={ -500, 1200,    0,    0,    0},.duration_ms= 3000},  //2
  {.axis_pos={ -500, 1200, -450,  200,    0},.duration_ms= 3000},  //3
  {.axis_pos={ -500, 1200,  450,  200,    0},.duration_ms= 3000},  //4
  {.axis_pos={ -500, 1200,  450,  200,  700},.duration_ms= 3000},  //5
  {.axis_pos={ -500, 1200,  450,  200, -750},.duration_ms= 3000},  //6
  {.axis_pos={ -500, 1200,  450,  200,    0},.duration_ms= 3000},  //7
  {.axis_pos={  450,    0,    0,  200, -750},.duration_ms= 3000},  //8
  {.axis_pos={  450,    0,    0, -500, -750},.duration_ms= 1000}, //9
  {.axis_pos={  450,    0,    0,  500,  750},.duration_ms= 1000}, //10
  {.axis_pos={  450,    0,    0, -500, -750},.duration_ms= 1000}, //11
  {.axis_pos={  450,    0,    0,  500,  750},.duration_ms= 1000}  //12
};

// Serial protocol globals
AxisProtocol serial_protocol_data;  // Holds the most recent command from serial protocol (if any)
char serial_buffer[SERIAL_BUFFER_LENGTH];

// Pointers to structs for axes
ArmAxis * shoulder;
ArmAxis * elbow;
ArmAxis * wrist_bend;
ArmAxis * wrist_rot;
ArmAxis * claw;

void run_axes(){
  shoulder->run();
  elbow->run();
  wrist_rot->run();
  wrist_bend->run();
  claw->run();
}
//-----------------------SERIALEVENT-----------------------//
// ISR for serial events
void serialEvent() {
  Serial.readBytesUntil(SERIAL_MESSAGE_TERMINATION_CHAR, 
                        serial_buffer, 
                        SERIAL_BUFFER_LENGTH);
  serial_protocol_data.update(serial_buffer);
  serial_protocol_data.print();
}

//-----------------------SETUP-----------------------//
void setup() {
  // Config Serial Port and signify life
  Serial.begin(115200);
  Serial.println(F("Starting..."));
  Serial.println(F("Initializing Axis 0...(shoulder)"));
  // move shoulder to zero and check for success;
  shoulder = (ArmAxis *)(new GearmotorAxis(0, //default position, tenths deg
                                           SHOULDER_AXIS,    // analog input pin
                                           SHOULDER_POSITIVE_PIN,     // moves feedback in the positive direction
                                           SHOULDER_NEGATIVE_PIN,     // moves feedback in the negative direction
                                           SHOULDER_AXIS_MAX,                       // max position tenths of degrees from center
                                           SHOULDER_COUNTS_AT_CENTER, // raw feedback position at center (zero angle)
                                           SHOULDER_AXIS_MIN));                     // min position, tenths of degrees from center
  Serial.println(F("Initializing Axis 1...(elbow)"));
  // move shoulder to zero and check for success;
  elbow = (ArmAxis *)(new GearmotorAxis(   0,                      //default position, tenths deg
                                           ELBOW_AXIS,             // analog input pin
                                           ELBOW_POSITIVE_PIN,     // moves feedback in the positive direction
                                           ELBOW_NEGATIVE_PIN,     // moves feedback in the negative direction
                                           ELBOW_AXIS_MAX,         // max position tenths of degrees from center
                                           ELBOW_COUNTS_AT_CENTER, // raw feedback position at center (zero angle)
                                           ELBOW_AXIS_MAX));       // min position, tenths of degrees from center
  Serial.println(F("Initializing Axis 2 (wrist bend)..."));
  wrist_bend = (ArmAxis *)(new ServoAxis(0,     // default position
                                         WRIST_BEND_AXIS_PIN,                  // output pin
                                         WRIST_BEND_AXIS_MAX,                  // max position (tenths)
                                         WRIST_BEND_AXIS_COUNTS_AT_CENTER,     // what servo angle is zero angle for the joint
                                         WRIST_BEND_AXIS_MIN));                // min position (tenths)
  Serial.println(F("Initializing Axis 3 (wrist rotation)..."));
  wrist_rot = (ArmAxis *)(new ServoAxis(0,     // default position
                                         WRIST_ROT_AXIS_PIN,                  // output pin
                                         WRIST_ROT_AXIS_MAX,                  // max position (tenths)
                                         WRIST_ROT_AXIS_COUNTS_AT_CENTER,     // what servo angle is zero angle for the joint
                                         WRIST_ROT_AXIS_MIN));    
  Serial.println(F("Initializing Axis 4 (claw)..."));
  claw = (ArmAxis *)(new ServoAxis(0,     // default position
                                         CLAW_AXIS_PIN,                  // output pin
                                         CLAW_AXIS_MAX,                  // max position (tenths)
                                         CLAW_AXIS_COUNTS_AT_CENTER,     // what servo angle is zero angle for the joint
                                         CLAW_AXIS_MIN));  
                                       
  Serial.println(F("Initialization Comlete!"));
}


//-----------------------SETUP-----------------------//
#define NUM_ITERATIONS_PER_MOTION 10
#define LOOP_DELAY_MS   10
#define PRINT_EVERY 100
uint16_t loop_iterator = 0;
uint16_t auto_cycles_remaining = 1000 / LOOP_DELAY_MS;  // iterations
uint16_t last_command_serial = 0;
uint8_t current_auto_pose = 0;
void loop(){
  loop_iterator++;

  if(serial_protocol_data.get_last_serial() == 0){  // no commands received
    // take commands from auto routine
    if((auto_cycles_remaining-- == 0) &&
        (current_auto_pose++ < NUM_TESTPOSES)) {  //used up time, move to next pose
      shoulder->set_desired_position(   testposes[current_auto_pose].axis_pos[0]);
      elbow->set_desired_position(      testposes[current_auto_pose].axis_pos[1]);
      wrist_bend->set_desired_position( testposes[current_auto_pose].axis_pos[2]);
      wrist_rot->set_desired_position(  testposes[current_auto_pose].axis_pos[3]);
      claw->set_desired_position(       testposes[current_auto_pose].axis_pos[4]);
      Serial.print(F("Going to pose number "));Serial.println(current_auto_pose);
      auto_cycles_remaining = testposes[current_auto_pose].duration_ms / LOOP_DELAY_MS;
    }
  } else {
    // fill commands with last protocol message received
    if(serial_protocol_data.get_last_serial() != last_command_serial){
      shoulder->set_desired_position(serial_protocol_data.get_axis(0).position * 10);
      elbow->set_desired_position(serial_protocol_data.get_axis(1).position * 10);
      wrist_bend->set_desired_position(serial_protocol_data.get_axis(2).position * 10);
      wrist_rot->set_desired_position(serial_protocol_data.get_axis(3).position * 10);
      claw->set_desired_position(serial_protocol_data.get_axis(4).position * 10);
      last_command_serial = serial_protocol_data.get_last_serial();
      Serial.print(F("Following command number "));Serial.println(last_command_serial);
    }
  }

 run_axes();

 if( loop_iterator == PRINT_EVERY){
    Serial.print(F("Shoulder: ")); shoulder->print();
    Serial.print(F("Elbow:    ")); elbow->print();
    loop_iterator = 0;
 }

  delay(LOOP_DELAY_MS);
}
