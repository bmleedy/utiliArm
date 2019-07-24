#include "ServoAxis.h"
#include "Arduino.h"

ServoAxis::ServoAxis(int16_t default_position, 
                     uint8_t output_pin,
                     int16_t max_position,
                     int16_t zero_angle,
                     int16_t min_position):ArmAxis(max_position, 
                                                   min_position, 
                                                   default_position){
  // todo: sanity check that these pins are on the PWM pin list
  this->control_pin = output_pin;
  pinMode(this->control_pin, OUTPUT);
  servo_control.attach(this->control_pin);
  this->zero_angle = zero_angle;
}

bool ServoAxis::zero_energy() {
  // todo flatline the output pin, should make the servo go limp
  servo_control.detach();
  digitalWrite(this->control_pin, LOW);
  return true;
}

int16_t ServoAxis::read_rate(){return 0;} // not implemented

int16_t ServoAxis::read_position(){
  //Servo's don't give feedback, so just return current desired
  return this->desired_position;
  //todo: improve this with a guess based on time since last delta
}

bool ServoAxis::run_axis(){

  //todo: check max and min angles in base class run_axis
  int16_t servo_desired_angle =  (this->zero_angle*10) - this->desired_position;
  servo_control.write(servo_desired_angle/10);
  return true;
}

void ServoAxis::print(){
  Serial.print(millis());                 Serial.print(F(" , dest: "));
  Serial.print(this->desired_position);   Serial.print(F(" , pos: "));
  Serial.print(this->current_position);   Serial.print(F(" , raw: "));
  Serial.println(" ");
}
