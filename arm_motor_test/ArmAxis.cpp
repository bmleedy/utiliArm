#include "ArmAxis.h"
#include "Arduino.h"

bool ArmAxis::set_max_position(int16_t pos){
  if((pos >= -1800) && (pos <= 1800)){
    this->max_position = pos;
  } else {
    this->disable_axis(REASON_MAX_LIM_OUT_OF_RANGE);
  }
}

bool ArmAxis::set_min_position(int16_t pos){
  if((pos >= -1800) && (pos <= 1800)){
    this->min_position = pos;
  } else {
    this->disable_axis(REASON_MIN_LIM_OUT_OF_RANGE);
  }
}

ArmAxis::ArmAxis(int16_t max_position, int16_t min_position, int16_t default_position, int16_t max_rate){
  this->max_rate = max_rate;
  bool success = true;
  success &= this->set_desired_position(default_position);
  success &= this->set_max_position(max_position);
  success &= this->set_min_position(min_position);
  if(success)
    this->enabled = true;
  else
    Serial.println(F("ArmAxis constructor failed!!"));
}

// Setters  False = failure / illegal operation
bool ArmAxis::set_desired_position(int16_t position_deg_tenths){
  // Sanity check input
  if(position_deg_tenths > this->max_position) {
    this->disable_axis(REASON_POS_TOO_HIGH);
    Serial.println(F("ArmAxis desired pos high!!"));
  } else if(position_deg_tenths < this->min_position) {
    this->disable_axis(REASON_POS_TOO_LOW);
    Serial.println(F("ArmAxis desired pos low!!"));
  } else {
    desired_position = position_deg_tenths;
    return true;
  }
  return false;
}

bool ArmAxis::set_desired_position(int16_t position_deg_tenths, 
                                  int16_t rate_tenths_per_sec) {
  return set_max_rate(rate_tenths_per_sec) && 
         set_desired_position(rate_tenths_per_sec);
}

bool ArmAxis::set_max_rate(int16_t max_rate){
  // Sanity check input
  if(max_rate < 0) {
    this->disable_axis(REASON_NEGATIVE_RATE);
  } else if(!this->rate_enabled) {
    this->disable_axis(REASON_RATE_DISABLED);
  } else {
    this->max_rate = max_rate;
    return true;
  }
}

bool ArmAxis::set_position_deadband(int16_t deadband_deg_tenths){
  if(deadband_deg_tenths > POSITION_DEADBAND_MAX) {
    this->disable_axis(REASON_DEADBAND_TOO_HIGH);
  } else if(deadband_deg_tenths < POSITION_DEADBAND_MIN) {
    this->disable_axis(REASON_DEADBAND_TOO_LOW);
  } else {
    this->position_deadband = deadband_deg_tenths;
    return true;
  }
}

bool ArmAxis::pid_init(int16_t pgain, int16_t igain, int16_t dgain,
              int16_t pmax, int16_t imax, int16_t dmax) {
  // not impl yet
  return false;
}

bool ArmAxis::disable_axis(uint16_t reason){
  this->enabled = false; 
  this->disabled_reason |= reason;
  return this->zero_energy();
}  // go to the safest possible state

uint16_t ArmAxis::get_disabled_reason(){
  return this->disabled_reason;
}

bool ArmAxis::stop_axis() {
  if(this->set_desired_position(this->read_position())){ // desire be here
    return true;  //set bits immediately
  } else {
    this->disable_axis(REASON_STOP_FAILED);
    return false;
  }
  // NOTE: we expect run_axis() to continue to run after this call to hold axis in place.
}

bool ArmAxis::run() {
  if(this->enabled){
    return this->run_axis();  // close all loops here
  } else {
    this->zero_energy();      // zero energy state if disabled
  }
}
