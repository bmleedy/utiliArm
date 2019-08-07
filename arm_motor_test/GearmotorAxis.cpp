#include "GearmotorAxis.h"
#include "Arduino.h"

#define GEARMOTOR_COUNTS_PER_DEGREE 4  // todo: measure real number

GearmotorAxis::GearmotorAxis(int16_t default_position,
                     uint8_t feedback_pin,    // analog input pin
                     uint8_t forward_pin,     // moves feedback in the positive
                     uint8_t reverse_pin,     // moves feedback in the negative
                     int16_t max_position,    // tenths of degrees from center
                     int16_t center_counts,   // raw feedback position zero deg
                     int16_t min_position):ArmAxis(max_position,
                                                   min_position,
                                                   default_position) {
  // todo: sanity check and init this pin, maybe in a separate method
  this->feedback_pin = feedback_pin;
  pinMode(this->feedback_pin, INPUT);
  this->read_position();

  // todo: sanity check and init this pin, maybe in a separate method
  this->forward_pin = forward_pin;
  pinMode(this->forward_pin, OUTPUT);
  digitalWrite(this->forward_pin, LOW);

  // todo: sanity check and init this pin, maybe in a separate method
  this->reverse_pin = reverse_pin;
  pinMode(this->reverse_pin, OUTPUT);
  digitalWrite(this->reverse_pin, LOW);

  this->center_counts = center_counts;
}

bool GearmotorAxis::zero_energy() {
  digitalWrite(this->forward_pin, LOW);
  digitalWrite(this->reverse_pin, LOW);
  return true;
}

int16_t GearmotorAxis::read_rate() {return 0;}  // not implemented

int16_t GearmotorAxis::read_position() {
  int16_t raw_counts = analogRead(this->feedback_pin);
  status.raw_counts = raw_counts;
  this->current_position = (raw_counts - this->center_counts) * 10
                           / GEARMOTOR_COUNTS_PER_DEGREE;
  return this->current_position;
  // todo: in the future, update my rate every time I read the position
}

bool GearmotorAxis::run_axis() {
  // Measure Rate
  this->read_rate();

  // Calculate delta-to-goal
  this->read_position();
  int16_t delta_pos = this->desired_position - this->current_position;


  // Check deadband
  if (abs(delta_pos) <= this->position_deadband) {
    digitalWrite(this->forward_pin, LOW);
    digitalWrite(this->reverse_pin, LOW);
    status.direction = 0;  // status stopped
  } else if (delta_pos > 0) {                 // Move forward
    digitalWrite(this->forward_pin, HIGH);
    digitalWrite(this->reverse_pin, LOW);
    status.direction = 1;  // status forward
  } else {                                   // Move back
    digitalWrite(this->forward_pin, LOW);
    digitalWrite(this->reverse_pin, HIGH);
    status.direction = -1;  // status reverse
  }
  return true;

  // only reach here in case of critical failure
  this->disable_axis(REASON_RUN_FAILED);
  return false;
}

void GearmotorAxis::print() {
  Serial.print(millis());                 Serial.print(F(" - dir: "));
  Serial.print(status.direction);         Serial.print(F(" , dest: "));
  Serial.print(this->desired_position);   Serial.print(F(" , pos: "));
  Serial.print(this->current_position);   Serial.print(F(" , raw: "));
  Serial.print(status.raw_counts);
}
