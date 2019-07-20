#include "Arduino.h"
#include "ServoAxis.h"
#include "GearMotorAxis.h"
#include "AxisProtocol.h"

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

AxisProtocol serial_protocol_data;

char serial_buffer[SERIAL_BUFFER_LENGTH];  //todo: SUPER not-threadsafe!!!!

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


ArmAxis * shoulder;
ArmAxis * elbow;
ArmAxis * wrist_bend;
ArmAxis * wrist_rot;
ArmAxis * claw;


// ISR for serial events
void serialEvent() {
  // todo: does this disable interrupts?
  Serial.readBytesUntil(SERIAL_MESSAGE_TERMINATION_CHAR, 
                        serial_buffer, 
                        SERIAL_BUFFER_LENGTH);
  serial_protocol_data.update(serial_buffer);
  serial_protocol_data.print();
}



//------------------------------------------------------------------
struct pose{
  int16_t axis_pos[NUM_AXES];
  int16_t axis_rate[NUM_AXES];
  uint16_t duration_ms;
};


// todo: save these to EEPROM
// todo: make programming utility
// todo: make teaching utility

#define NUM_TESTPOSES 7
//  shoulder,     elbow,     wrist_bend,       wrist_rot,    claw
pose testposes[NUM_TESTPOSES] = {
  {.axis_pos={  0 ,   0,   0,   0,   0},.axis_rate={0,0,0,0,0},.duration_ms=3000},
  {.axis_pos={-500,1200,   0,   0,   0},.axis_rate={0,0,0,0,0},.duration_ms=3000},
  {.axis_pos={-500,1200,-450, 200,   0},.axis_rate={0,0,0,0,0},.duration_ms=3000},
  {.axis_pos={-500,1200, 450, 200,   0},.axis_rate={0,0,0,0,0},.duration_ms=3000},
  {.axis_pos={-500,1200, 450, 200, 700},.axis_rate={0,0,0,0,0},.duration_ms=3000},
  {.axis_pos={-500,1200, 450, 200,-750},.axis_rate={0,0,0,0,0},.duration_ms=3000},
  {.axis_pos={-500,1200, 450, 200,   0},.axis_rate={0,0,0,0,0},.duration_ms=3000},
};
uint8_t current_auto_pose = 0;
//------------------------------------------------------------------

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




void run_axes(){
  shoulder->run();
  elbow->run();
  wrist_rot->run();
  wrist_bend->run();
  claw->run();
}

#define LOOP_DELAY_MS   10
#define PRINT_EVERY 100
uint16_t loop_iterator = 0;
uint16_t auto_cycles_remaining = 1000 / LOOP_DELAY_MS;  // iterations
uint16_t last_command_serial = 0;

void loop(){
  loop_iterator++;

  if(serial_protocol_data.get_last_serial() == 0){  // no commands received
    // take commands from auto routine
    if((auto_cycles_remaining-- == 0) &&
        (current_auto_pose < NUM_TESTPOSES)) {  //used up time, move to next pose
      shoulder->set_desired_position(   testposes[current_auto_pose].axis_pos[0]);
      elbow->set_desired_position(      testposes[current_auto_pose].axis_pos[1]);
      wrist_bend->set_desired_position( testposes[current_auto_pose].axis_pos[2]);
      wrist_rot->set_desired_position(  testposes[current_auto_pose].axis_pos[3]);
      claw->set_desired_position(       testposes[current_auto_pose].axis_pos[4]);
      Serial.print(F("Going to pose number "));Serial.println(current_auto_pose);
      current_auto_pose++;
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

  if(loop_iterator % 10 == 0)
    run_axes();

 if( loop_iterator == PRINT_EVERY){
    Serial.print(F("Shoulder: ")); shoulder->print();
    Serial.print(F("Elbow:    ")); elbow->print();
    loop_iterator = 0;
 }

  delay(LOOP_DELAY_MS);
}
