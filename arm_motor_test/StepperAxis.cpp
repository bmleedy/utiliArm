#include "StepperAxis.h"
#include "Arduino.h"


// 60r/1min * 1min/60s * 200 counts/1r = 200 counts/s = 0.005 s/count = 5 msec/ct = 5000 usec/ct
// We want to time half-steps, so ... halfstep_period is 2500 usec/halfstep
#define STEPPER_MICROS_PER_HALFSTEP   2500     //60 rpm (can change eventually)
#define STEPPER_ANALOG_COUNTS_PER_DEGREE  4    //todo: measure real number
#define STEPPER_STEPS_PER_DEGREE         20    //measured = 1750 steps per 90 deg = 19.4444...

StepperAxis::StepperAxis(int16_t default_position,  // default position
                        uint8_t sensor_pin,         // potentiometer(analog) or limit switch digital for analog_position = false
                        uint8_t step_pin    ,       // digital output for stepper steps
                        uint8_t dir_pin     ,       // digital output for step direction
                        int16_t max_position,       // max position (tenths)
                        int16_t zero_counts,       // what counts equals zero position for the axis
                        int16_t min_position, 
                        bool analog_position):ArmAxis( max_position, 
                                                       min_position, 
                                                       default_position){
  // todo: sanity check and init this pin, maybe in a separate method
  this->sensor_pin = sensor_pin;
  this->analog_position = analog_position;
  if(analog_position){
    pinMode(this->sensor_pin, INPUT);  //sense position based on potentiometer
  } else {
    pinMode(this->sensor_pin, INPUT_PULLUP); // sense position based on limit switch and steps
  }
  this->read_position();

  // todo: sanity check and init this pin, maybe in a separate method
  this->step_pin = step_pin;
  pinMode(this->step_pin, OUTPUT);
  digitalWrite(this->step_pin, LOW);

  // todo: sanity check and init this pin, maybe in a separate method
  this->dir_pin = dir_pin;
  pinMode(this->dir_pin, OUTPUT);
  digitalWrite(this->dir_pin, LOW);
  
  this->zero_counts = zero_counts;

  this->init_position();
  this->desired_position = default_position;
}

bool StepperAxis::zero_energy() {
  Serial.println("zeroing");
  digitalWrite(this->step_pin, LOW);
  digitalWrite(this->dir_pin, LOW);
  return true;
}

int16_t StepperAxis::read_rate(){return 0;} // todo: not implemented

int16_t StepperAxis::read_position(){
  int16_t raw_counts;
  int16_t counts_per_degree;
  
  if(analog_position){
    raw_counts = analogRead(this->sensor_pin);
    counts_per_degree = STEPPER_ANALOG_COUNTS_PER_DEGREE;
  } else {
    raw_counts = this->step_counts;
    counts_per_degree = STEPPER_STEPS_PER_DEGREE;
  }

  this->current_position = (raw_counts - this->zero_counts) * 10 / counts_per_degree;  //todo: create angle calibration routine
  status.raw_counts = raw_counts;
  return this->current_position;
  //todo: in the future, update my rate every time I read the position
}

void StepperAxis::init_position(){
  if(this->analog_position)  // not relevant for potentiometer mode
    return;
  // move backward till I hit limit switch
  do{
    this->move_constant_velocity(-1);  
  } while(digitalRead(this->sensor_pin)==HIGH);
  this->step_counts = 0;
  return;
}

// 0=stop; 1=forward; -1=back
// Note, to make the motor move at 60rpm, this must be called at least every 2.5ms (400Hz)
void StepperAxis::move_constant_velocity(int8_t direction){

  if(direction == 0)
    return;  // no motion needed, nothing to do
    
  // Find out time since last step
  uint32_t this_us = micros();
  uint32_t delta_us = this_us - last_step_us;
  if(delta_us > 100000) // just skip on micros overflow
    return;

  // Set direction appropriately
  if(direction > 0)
    digitalWrite(this->dir_pin, HIGH);
  else
    digitalWrite(this->dir_pin, LOW);


  // Take a step, if it's time
  if(delta_us > STEPPER_MICROS_PER_HALFSTEP){
    if(last_step_pin_state == LOW){
      digitalWrite(this->step_pin, HIGH);
      last_step_pin_state = HIGH;
    } else {
      digitalWrite(this->step_pin, LOW);
      last_step_pin_state = LOW;
    }
    last_step_us = this_us;
    this->step_counts += direction;  // add or subtract one based on direction I'm moving
  }
  
}

bool StepperAxis::run_axis(){

  // Measure Rate
  this->read_rate();
  Serial.println("running");
  // Calculate delta-to-goal
  this->read_position();
  int16_t delta_pos = this->desired_position - this->current_position;
  Serial.println(delta_pos);

  // Check deadband and set desired velocity
  if(abs(delta_pos) <= this->position_deadband){
    status.direction = 0;  // status stopped
  } else if(delta_pos > 0) {                 // Move forward
    status.direction = 1; // status forward
  } else {                                   // Move back
    status.direction = -1; // status reverse
  }

  // Execute on that velocity
  move_constant_velocity(status.direction);
  
  return true;

  // only reach here in case of critical failure
  this->disable_axis(REASON_RUN_FAILED);
  return false;  
}

void StepperAxis::print(){
  Serial.print(millis());                 Serial.print(F(" - dir: "));
  Serial.print(status.direction);         Serial.print(F(" , dest: "));
  Serial.print(this->desired_position);   Serial.print(F(" , pos: "));
  Serial.print(this->current_position);   Serial.print(F(" , step: "));
  Serial.print(this->step_counts);        Serial.print(F(" , raw: "));
  Serial.print(status.raw_counts);
  Serial.println(" ");
}
