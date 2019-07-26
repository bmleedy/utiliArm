#include "EEPROM.h"
#include "Arduino.h"
#include "ServoAxis.h"
#include "GearMotorAxis.h"
#include "StepperAxis.h"
#include "AxisProtocol.h"
#include "Wire.h"   // ~120 bytes of dynamic memory

// https://www.arduino.cc/en/Tutorial/MasterWriter
//todo: MECHANICAL  add zero hashmarks indicators on all joints and adaptors
//todo: MECHANICAL  add holes for flange (in addition to existing holes)
//todo: MECHANICAL  add slots/holes for installing the flange
//todo: ELECTRICAL  add buttons (rocker switch) to manually move the arm axes
//todo: ELECTRICAL  add a killswitch to stop motion on the big axes
//todo: MECHANICAL  strengthen feet areas from cracking
//todo: MECHANICAL  "bed" the shaft attachment to secure it
//todo: MECHANICAl  widen screwdriver holes for bet
//todo: MECHANICAL  make room for washers in feet

#define SERIAL_READ_TIMEOUT         10   // ms, should take no more than 5ms

#define I2C_SCL_PIN    A5  // clock pin
#define I2C_SDA_PIN    A4  // data pin
#define I2C_SLAVE_ADDRESS 8

#define BASE_SENSOR     12 
#define BASE_STEP_PIN   11   // todo: fixme
#define BASE_DIR_PIN     4   // todo: fixme
#define BASE_AXIS_MIN -800     // todo: fixme
#define BASE_AXIS_MAX  800     // todo: fixme
#define BASE_COUNTS_AT_CENTER  1800  // todo: fixme, just a guess

// Shoulder Axis Constants
#define SHOULDER_AXIS A3
#define SHOULDER_AXIS_MIN -800     // tenths
#define SHOULDER_AXIS_MAX  800     // tenths
#define SHOULDER_POSITIVE_PIN 8
#define SHOULDER_NEGATIVE_PIN 9
#define SHOULDER_COUNTS_AT_CENTER 764

#define VERBOSE false

// Elbow Axis Constants
#define ELBOW_AXIS A2
#define ELBOW_AXIS_MIN  -1201      // tenths
#define ELBOW_AXIS_MAX   1201      // tenths
#define ELBOW_NEGATIVE_PIN 7
#define ELBOW_POSITIVE_PIN 6
#define ELBOW_COUNTS_AT_CENTER 497   

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
#define CLAW_AXIS_MAX   755                // 90 degrees
#define CLAW_AXIS_COUNTS_AT_CENTER   90    //center position for the servo

// Auto pose routine constants
// Poses for the automated routine
struct pose{
  int16_t axis_pos[NUM_AXES];
  uint16_t duration_ms;
};

// Position Routine constants
// todo: make teaching utility
#define NUM_TESTPOSES 12

// Class to read poses off of EEPROM storage: todo: make shared instead of copied over
class RoutineStorage{
  uint16_t record_storage_start = 1;
  uint16_t current_pointer = 0;
  uint16_t current_record = 0;
  uint8_t num_records = 0;
  bool wrap;

  public:
  RoutineStorage(bool wrap=true){
    EEPROM.get(current_pointer, num_records);
    current_pointer += sizeof(num_records);
    record_storage_start = current_pointer;
    this->wrap = wrap;
  }

  uint8_t get_num_records(){return num_records;}
  uint16_t get_current_record(){return current_record;}

  void reset(){
    current_pointer = record_storage_start;
    current_record = 0;
  }

  void set_wrap(bool wrap){
    this->wrap = wrap;
  }

  uint8_t get_next_record(pose * read_position){
    if(current_record++ < num_records){
      EEPROM.get(current_pointer, *read_position);
      current_pointer += sizeof(*read_position);
    } else if(wrap) {
      this->reset();
      get_next_record(read_position);
    } else {
      //do nothing, just leave it as it is
    }
    return current_record;
  }

  void dump_pose(pose * p, int i=0){
    Serial.print(F("{.axis_pos={ ")); 
    for(int pos_i=0; pos_i < NUM_AXES; pos_i++){
      Serial.print(p->axis_pos[pos_i]); Serial.print(",");
    }
    Serial.print(F("},.duration_ms= "));Serial.print(p->duration_ms);
    Serial.print(F("},  // ")); Serial.println(i);
  }
};
RoutineStorage * storage;


// Serial protocol globals
AxisProtocol serial_protocol_data;  // Holds the most recent command from serial protocol (if any)
char serial_buffer[SERIAL_BUFFER_LENGTH];

// Pointers to structs for axes
ArmAxis * base;
ArmAxis * shoulder;
ArmAxis * elbow;
ArmAxis * wrist_bend;
ArmAxis * wrist_rot;
ArmAxis * claw;

void run_axes(){
  base->run();
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

//-----------------------I2C RECEIVEEVENT-----------------------//
// function that executes whenever data is received from master
void receiveEvent(int howMany) {
  uint8_t i = 0;
  while (Wire.available() > 0) { // loop through all bytes
    serial_buffer[i++] = Wire.read(); // read bytes into the serial buffer
  }
  serial_protocol_data.update(serial_buffer);
  serial_protocol_data.print();
}

//-----------------------SETUP-----------------------//
void setup() {
  // Config Serial Port and signify life
  Serial.begin(115200);
  Serial.println(F("Starting..."));
  Serial.println(F("Initializing Axis 0...(shoulder)"));

  Wire.begin(I2C_SLAVE_ADDRESS);           // join i2c bus with address #8
  Wire.onReceive(receiveEvent);            // register event

  // move shoulder to zero and check for success;
  base = (ArmAxis *)(new StepperAxis(0, //default position, tenths deg
                                           BASE_SENSOR,    // limit switch pin
                                           BASE_STEP_PIN,     // moves feedback in the positive direction
                                           BASE_DIR_PIN,     // moves feedback in the negative direction
                                           BASE_AXIS_MAX,                       // max position tenths of degrees from center
                                           BASE_COUNTS_AT_CENTER, // raw feedback position at center (zero angle)
                                           BASE_AXIS_MIN));                     // min position, tenths of degrees from center
                                           
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
                                           ELBOW_AXIS_MIN));       // min position, tenths of degrees from center
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
                                         
  Serial.println(F("Initializing storage reader..."));
  storage = new RoutineStorage(false);
  
  Serial.println(F("Initialization Comlete!"));

  Serial.print(F("Routine to follow: [")); Serial.print(storage->get_num_records());Serial.println("]");
  int i = 0;
  pose read_position;
  while(storage->get_next_record(&read_position) <= NUM_TESTPOSES){
    storage->dump_pose(&read_position, ++i);
  }
  storage->reset();
  storage->set_wrap(true);
  
}


//-----------------------SETUP-----------------------//
#define PRINT_EVERY_MS 1000 //ms
unsigned long last_print_ms = 0;
unsigned long last_print_micros = 0;
unsigned long last_print_iter = 0;
unsigned long auto_timeout = 0;
unsigned long loop_iterator = 0;
uint16_t last_command_serial = 0;
uint8_t current_auto_pose = 0;
void loop(){
  loop_iterator++;
  pose read_position;

  if(serial_protocol_data.get_last_serial() == 0){  // no commands received
    // take commands from auto routine
    if(millis() >= auto_timeout){  //used up time, move to next pose
      storage->get_next_record(&read_position);
      Serial.print(F("setting desired to: "));storage->dump_pose(&read_position, storage->get_current_record());
      shoulder->set_desired_position(   read_position.axis_pos[0]);
      elbow->set_desired_position(      read_position.axis_pos[1]);
      wrist_bend->set_desired_position( read_position.axis_pos[2]);
      wrist_rot->set_desired_position(  read_position.axis_pos[3]);
      claw->set_desired_position(       read_position.axis_pos[4]);
      auto_timeout = millis() + read_position.duration_ms;  //note, ignoring overflow here :-S
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

 if( (millis() - last_print_ms) >= PRINT_EVERY_MS){
    // microseconds per loop
    Serial.print( (micros()-last_print_micros)/(loop_iterator));  //todo: ignores micros overflow
    // Shoulder position
    Serial.print(F(" Hz -  Should: ")); shoulder->print();
    // Elbow position
    Serial.print(F("| Elbow:    ")); elbow->print();
    Serial.println("");
    last_print_micros = micros();
    last_print_ms = millis();
    loop_iterator = 0;
 }
}
