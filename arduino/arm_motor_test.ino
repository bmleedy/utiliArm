#include "Arduino.h"
#include "ServoAxis.h"
#include "GearMotorAxis.h"
//todo: add zero hashmarks indicators on all joints

#define NUM_ITERATIONS_PER_MOTION 10

#define SHOULDER_AXIS A3
#define SHOULDER_AXIS_MIN 60      // actual limit is 70
#define SHOULDER_AXIS_MAX 500     // actual limit is TBD
#define SHOULDER_POSITIVE_PIN 8
#define SHOULDER_NEGATIVE_PIN 9

#define ELBOW_AXIS A2
#define ELBOW_AXIS_MIN  600        // about 90 degrees
#define ELBOW_AXIS_MAX  1000        // straight?
#define ELBOW_NEGATIVE_PIN 7
#define ELBOW_POSITIVE_PIN 6

#define SERVO_PIN_1 10

// todo: use tinkercad to test serial protocol

class MultiAxisRunner{
  uint16_t last_run_time = 0;
  ArmAxis * axes[8];

  // null out the list
  MultiAxisRunner(){
    for(int i=0; i<8; i++){
      axes[i] = NULL;
    }
  }

  int8_t register_axis(uint8_t pos, ArmAxis * axis, const char name[4]){
    //check that pause is in range
    if(pos >= 8)
      return -1;
    
    // check that the axis is enabled
    if(!axis->is_enabled())
      return -1;
      
    // check that pos is not already occupied
    if(axes[pos] == NULL){
      // save the axis to the array at pos
      axes[pos] = axis;
    } else {
      return -1;
    }
    
    return pos;
  }

  bool run(){
    // run each non-null class
    for(int i=0; i<8; i++){
      if(axes[i] != NULL){
        axes[i]->run();
      }
    }
    // todo: return bitmap of success or failure
    return true;
  }

  bool deregister_axis(uint8_t pos){
    // todo: call destructor
    // todo: null that position in the array
  }
};

#define SERIAL_MESSAGE_BUFFER_SIZE 200
char serial_message_buffer[SERIAL_MESSAGE_BUFFER_SIZE];

void read_next_message(){

  // Message should take no longer than 5 ms to arrive
  Serial.setTimeout(10);
  Serial.readBytesUntil('!',serial_message_buffer, SERIAL_MESSAGE_BUFFER_SIZE);

  // tokenize the buffer to the first occurrence of '#'
  // Check for termination character
  // Check for checksum (impl later)
  // Check for ',' at all locations
  // Check for '|' at all locations
  // for each 9-byte block, strtok on '|' and extract position and rate values
  // sanity check values - discard message if out of range
  
};


int a2_state = 0;
int a3_state = 0;

void print_bracket_value(int value){
  Serial.print("[");
  Serial.print(value);
  Serial.print("]");
}

void dump_state_for(int delay_iterations){

  for(int i=0; i<delay_iterations; i++){
    a2_state = analogRead(ELBOW_AXIS);
    a3_state = analogRead(SHOULDER_AXIS);
    
    Serial.print(F(" | "));
    Serial.print(F(" A2:"));
    print_bracket_value(a2_state);
    Serial.print(F(" A3:"));
    print_bracket_value(a3_state);
    Serial.println(" ");

    delay(100);
  }
}


ArmAxis * shoulder;
ArmAxis * elbow;
ArmAxis * wrist_bend;
ArmAxis * wrist_rot;
ArmAxis * claw;

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
                                           210,                       // max position tenths of degrees from center
                                           590,                       // raw feedback position at center (zero angle)
                                          -210));                     // min position, tenths of degrees from center


  Serial.println(F("Initializing Axis 1...(elbow)"));
  // move shoulder to zero and check for success;
  elbow = (ArmAxis *)(new GearmotorAxis(   0,                    //default position, tenths deg
                                           ELBOW_AXIS,             // analog input pin
                                           ELBOW_POSITIVE_PIN,     // moves feedback in the positive direction
                                           ELBOW_NEGATIVE_PIN,     // moves feedback in the negative direction
                                           210,                    // max position tenths of degrees from center
                                           486,                    // raw feedback position at center (zero angle)
                                          -210));                  // min position, tenths of degrees from center

  Serial.println(F("Initializing Axis 2 (wrist rotation)..."));
  wrist_rot = (ArmAxis *)(new ServoAxis(  0,     // default position
                                         10,     // output pin
                                        900,     // max position (tenths)
                                         90,     // what servo angle is zero angle for the joint
                                       -900));   // min position (tenths)

  Serial.println(F("Initializing Axis 3 (wrist bend)..."));
  wrist_bend = (ArmAxis *)(new ServoAxis(0,     // default position
                                         3,     // output pin
                                        900,     // max position (tenths)
                                         90,     // what servo angle is zero angle for the joint
                                       -900));   // min position (tenths)

  Serial.println(F("Initializing Axis 4 (claw)..."));
  claw = (ArmAxis *)(new ServoAxis(  0,     // default position
                                     5,     // output pin
                                   700,     // max position (tenths) (700=closed)
                                    90,     // what servo angle is zero angle for the joint
                                  -800));   // min position (tenths)(-800 = wide open
                                       
  Serial.println(F("Initialization Comlete!"));
}

#define PRINT_EVERY 10
uint16_t print_iterator = 0;
void loop(){
  print_iterator++;

  shoulder->run();
  elbow->run();
  wrist_rot->run();
  wrist_bend->run();
  claw->run();

 if( print_iterator == PRINT_EVERY){
    Serial.print(F("Shoulder: ")); shoulder->print();
    Serial.print(F("Elbow:    ")); elbow->print();
    print_iterator = 0;
 }


 if(millis() >= 3000 && millis() < 4000) {
    shoulder->set_desired_position(-500);  //50 degrees from center
    elbow->set_desired_position(1200);      // 30 degrees from center
 }

 if(millis() >= 6000 && millis() < 8000){
  wrist_rot->set_desired_position(-450);
  wrist_bend->set_desired_position(-200);
 }

if(millis() >= 8000 && millis() < 10000){
  wrist_rot->set_desired_position(450);
  wrist_bend->set_desired_position(200);
 }

if(millis() >= 10000 && millis() < 13000){
  claw->set_desired_position(700);
}

if(millis() >= 13000){
  claw->set_desired_position(-750);
}

 
  delay(100);
}
