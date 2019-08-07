#include "Servo.h"
#include "Wire.h"   // ~120 bytes of dynamic memory

#define SERIAL_READ_TIMEOUT         10   // ms, should take no more than 5ms

#define I2C_SCL_PIN    A5  // clock pin
#define I2C_SDA_PIN    A4  // data pin
#define I2C_SLAVE_ADDRESS 8

#define SERVO_DEFAULT_ANGLE 90

#define BASE_SENSOR     4 
#define BASE_STEP_PIN   13   // todo: fixme
#define BASE_DIR_PIN     12   // todo: fixme
//#define BASE_ENA_PIN    XX


// Shoulder Axis Constants
#define SHOULDER_AXIS A3
#define SHOULDER_POSITIVE_PIN 8
#define SHOULDER_NEGATIVE_PIN 9
#define SHOULDER_ENA_PIN     11

#define VERBOSE false

// Elbow Axis Constants
#define ELBOW_AXIS A2
#define ELBOW_NEGATIVE_PIN 7
#define ELBOW_POSITIVE_PIN 6
#define ELBOW_ENA_PIN      2

// Wrist bend axis constants
#define WRIST_BEND_AXIS_PIN     3                // digital pin 3 (PWM pin)

// Wrist rotation axis constants
#define WRIST_ROT_AXIS_PIN    10                // digital pin 10 (PWM pin)

// Wrist rotation axis constants
#define CLAW_AXIS_PIN     5                // digital pin 5 (PWM pin)

Servo wrist_bend_servo;
Servo wrist_rot_servo;
Servo claw_servo;

void printhelp(){
  Serial.println(F("HELP Command Received.  Listing available commands:"));
  Serial.println(F("  H =>  Help, prints this command"));
  Serial.println(F("  B =>  Calibrate Base"));
  Serial.println(F("  S =>  Calibrate Shoulder"));
  Serial.println(F("  E =>  Calibrate Elbow"));
  Serial.println(F("  W =>  Calibrate Wrist bend"));
  Serial.println(F("  R =>  Calibrate wrist Rotation"));
  Serial.println(F("  C =>  Calibrate Claw"));
}

void printok(){
  Serial.println(F(" [OK]!")); // stored here to save space
}

void run_base_calibration(){
  unsigned long position_counts = 0;
  Serial.print(F("Waiting for the base limit switch to be released..."));
  while(digitalRead(BASE_SENSOR) == LOW){
    delay(10);
  }
  printok();
  Serial.print(F("Press the base limit switch manually..."));
    while(digitalRead(BASE_SENSOR) == HIGH){
    delay(10);
  }
  printok();
  Serial.print(F("Waiting for the base limit switch to be released..."));
  while(digitalRead(BASE_SENSOR) == LOW){
    delay(10);
  }
  printok();
  Serial.println(F("Stepping NEGATIVE until the limit switch is hit..."));
  unsigned char stepStatus = LOW; 
  digitalWrite(BASE_DIR_PIN, LOW);// low = negative direction
  while(digitalRead(BASE_SENSOR) == HIGH){
    if(stepStatus==LOW){
      digitalWrite(BASE_STEP_PIN, HIGH);
      stepStatus=HIGH;
    } else {
      digitalWrite(BASE_STEP_PIN, LOW);
      stepStatus=LOW;
    }
    delay(5);
  }
  printok();
  Serial.println(F("Limit switch hit position is Now ZERO."));
  delay(2000);
  Serial.println(F("Stepping POSITIVE."));
  Serial.println(F("Press the limit switch when you're near center..."));
  stepStatus = LOW;
  digitalWrite(BASE_DIR_PIN, HIGH);
  while(digitalRead(BASE_SENSOR) == HIGH || position_counts < 100){
    if(stepStatus==LOW){
      digitalWrite(BASE_STEP_PIN, HIGH);
      stepStatus=HIGH;
    } else {
      digitalWrite(BASE_STEP_PIN, LOW);
      stepStatus=LOW;
      position_counts++;
    }
    delay(5);
  }
  printok();
  Serial.println(F("Now, press and release the limit switch to step forward to the calibration line."));
  Serial.print(F("Press and hold when you have reached the center mark..."));
  bool run_step = false;
  uint16_t pressed_cycles = 0;
  while(true){
    if(digitalRead(BASE_SENSOR) == LOW){
      // button pressed, count how long and break if timed out
      if(++pressed_cycles >= 200){
        Serial.print(F(" Done! "));
        break;
      } else {
        run_step = true;  // signal that we should step next cycle
      }
    } else {
      pressed_cycles = 0;
      if(run_step){
        // do a step, if it's signaled when the button is released
        for(int i=0; i<1; i++){
          digitalWrite(BASE_STEP_PIN, HIGH);
          delay(5);
          digitalWrite(BASE_STEP_PIN, LOW);
          position_counts++;
        }
        run_step = false;
      } //if(run_step)
    }
    delay(10);
  }
  printok();
  Serial.print(F("The calibration point position is ")); Serial.print(position_counts); Serial.println(F(" counts"));
  Serial.println(F("\n\n exiting calibration\n"));
  printhelp();
}

void run_gearmotor_calibration(uint8_t sensor_pin, uint8_t positive_pin, uint8_t negative_pin){
  unsigned long position_counts = 0;
  Serial.println(F("Starting Gearmotor calibration"));
  Serial.print(F("Waiting for the base limit switch to be released..."));
  while(digitalRead(BASE_SENSOR) == LOW){
    delay(10);
  }
  printok();
  
  Serial.println(F("Press and hold the base limit switch to move the shoulder in the POSITIVE direction."));
  while(digitalRead(BASE_SENSOR) == HIGH){
    delay(10);
  }
  Serial.print(F("Moving positive>>>>>>>"));  
  while(digitalRead(BASE_SENSOR) == LOW){
    digitalWrite(positive_pin, HIGH);
    digitalWrite(negative_pin, LOW);
    delay(10);
  }
  digitalWrite(positive_pin, LOW);
  digitalWrite(negative_pin, LOW);
  Serial.print(analogRead(sensor_pin));
  printok();
    Serial.println(F("Press and hold the base limit switch to move the shoulder in the NEGATIVE direction."));
  while(digitalRead(BASE_SENSOR) == HIGH){
    delay(10);
  }
  Serial.print(F("Moving negative<<<<<<<"));
  while(digitalRead(BASE_SENSOR) == LOW){
    digitalWrite(positive_pin, LOW);
    digitalWrite(negative_pin, HIGH);
    delay(10);
  }
  digitalWrite(positive_pin, LOW);
  digitalWrite(negative_pin, LOW);
  Serial.print(analogRead(sensor_pin));
  printok();
  Serial.println(F("Done!"));
  printhelp();
}


void run_shoulder_calibration(){
 run_gearmotor_calibration(SHOULDER_AXIS, SHOULDER_POSITIVE_PIN, SHOULDER_NEGATIVE_PIN);
}

void run_elbow_calibration(){
  run_gearmotor_calibration(ELBOW_AXIS, ELBOW_POSITIVE_PIN, ELBOW_NEGATIVE_PIN);
}


void calibrate_servo(Servo servo){
  uint8_t servo_position = 90;
  Serial.println(F("Starting Servo calibration"));
  Serial.print(F("Waiting for the base limit switch to be released..."));
  while(digitalRead(BASE_SENSOR) == LOW){
    delay(10);
  }
  printok();
  servo.write(servo_position);  // always start at middle
  
  Serial.println(F("Press and hold the base limit switch to move the shoulder in the POSITIVE direction."));
  while(digitalRead(BASE_SENSOR) == HIGH){
    delay(10);
  }
  Serial.print(F("Moving positive>>>>>>>"));  
  while(digitalRead(BASE_SENSOR) == LOW){
    if(servo_position+1 <= 180)
      servo.write(++servo_position);
    delay(500);
  }
  Serial.print(servo_position);
  printok();
  
  Serial.println(F("Press and hold the base limit switch to move the shoulder in the NEGATIVE direction."));
  while(digitalRead(BASE_SENSOR) == LOW){
    if(servo_position+1 <= 180)
      servo.write(++servo_position);
    delay(500);
  }
  Serial.print(F("Moving negative<<<<<<<"));
  while(digitalRead(BASE_SENSOR) == LOW){
    if(servo_position-1 >= 0)
      servo.write(--servo_position);
    delay(500);
  }
  Serial.print(servo_position);
  printok();
  Serial.println(F("Done!"));
  printhelp();
}

void run_wrist_bend_calibration(){
  calibrate_servo(wrist_bend_servo);
}

void run_wrist_rot_calibration(){
  calibrate_servo(wrist_rot_servo);
}

void run_claw_calibration(){
  calibrate_servo(claw_servo);
}

#define NULL_COMMAND 0
char command_to_process = NULL_COMMAND;
void process_command(char command){
  if(command == NULL_COMMAND)
    return;
    
  Serial.print(F("Checking Command...")); Serial.println(command);
  switch(command){
    case 'B':
    case 'b':
      Serial.println(F("Base Calibration command received"));
      run_base_calibration();
      break;
    case 'S':
    case 's':
      Serial.println(F("Shoulder Calibration command received"));
      run_shoulder_calibration();
      break;
    case 'E':
    case 'e':
      Serial.println(F("Elbow Calibration command received"));
      run_elbow_calibration();
      break;
    case 'W':
    case 'w':
      Serial.println(F("Wrist bend Calibration command received"));
      run_wrist_bend_calibration();
      break;
    case 'R':
    case 'r':
      Serial.println(F("Wrist rotation Calibration command received"));
      run_wrist_rot_calibration();
      break;
    case 'C':
    case 'c':
      Serial.println(F("Claw Calibration command received"));
      run_claw_calibration();
      break;
    case 'H':
    case 'h':
    default :
      printhelp();
      break;
  }
}



//-----------------------SERIALEVENT-----------------------//
// ISR for serial events
void serialEvent() {
  char lastval = 0;
  char val = 0;
  while(Serial.available() > 0){
    val = Serial.read();
    if(val == '\n')
      command_to_process = lastval;
    Serial.print(val);
    lastval = val;
  }
}

//-----------------------I2C RECEIVEEVENT-----------------------//
// function that executes whenever data is received from master
void receiveEvent(int howMany) {
  char val = 0;
  while (Wire.available() > 0) { // loop through all bytes
    Serial.print(Wire.read()); // read bytes into the serial buffer
  }
  Serial.print("\n");
}




void setup() {
 // Config Serial Port and signify life
  Serial.begin(115200);
  Serial.println(F("Starting..."));
  
  Serial.println(F("Initializing I2C...(base)"));

  Wire.begin(I2C_SLAVE_ADDRESS);           // join i2c bus with address #8
  Wire.onReceive(receiveEvent);            // register event

  Serial.println(F("Initializing Base..."));
  pinMode(BASE_SENSOR, INPUT_PULLUP);  // digital in limit switch
  pinMode(BASE_STEP_PIN, OUTPUT); digitalWrite(BASE_STEP_PIN, LOW);
  pinMode(BASE_DIR_PIN, OUTPUT);  digitalWrite(BASE_DIR_PIN, LOW);
  
  Serial.println(F("Initializing Shoulder..."));
  pinMode(SHOULDER_AXIS, INPUT);  // analog input potentiometer
  pinMode(SHOULDER_POSITIVE_PIN, OUTPUT), digitalWrite(SHOULDER_POSITIVE_PIN, LOW);
  pinMode(SHOULDER_NEGATIVE_PIN, OUTPUT), digitalWrite(SHOULDER_NEGATIVE_PIN, LOW);
  pinMode(SHOULDER_ENA_PIN, OUTPUT), digitalWrite(SHOULDER_ENA_PIN, HIGH);

  Serial.println(F("Initializing Elbow..."));
  pinMode(ELBOW_AXIS, INPUT);  // analog input potentiometer
  pinMode(ELBOW_POSITIVE_PIN, OUTPUT), digitalWrite(ELBOW_POSITIVE_PIN, LOW);
  pinMode(ELBOW_NEGATIVE_PIN, OUTPUT), digitalWrite(ELBOW_NEGATIVE_PIN, LOW);
  pinMode(ELBOW_ENA_PIN, OUTPUT), digitalWrite(ELBOW_ENA_PIN, HIGH);

  Serial.println(F("Initializing Wrist Bend..."));
  wrist_bend_servo.attach(WRIST_BEND_AXIS_PIN); wrist_bend_servo.write(SERVO_DEFAULT_ANGLE);

  Serial.println(F("Initializing Wrist Rotation..."));
  wrist_rot_servo.attach(WRIST_ROT_AXIS_PIN); wrist_rot_servo.write(SERVO_DEFAULT_ANGLE);

  Serial.println(F("Initializing Claw ..."));
  claw_servo.attach(CLAW_AXIS_PIN); claw_servo.write(SERVO_DEFAULT_ANGLE);
                                         
  Serial.println(F("Initialization Comlete!"));
  printhelp();
}

void loop() {
  // check for command, run if available
  process_command(command_to_process);
  command_to_process = NULL_COMMAND;
  delay(10);
  
}
