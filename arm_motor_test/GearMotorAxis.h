#ifndef INC_GEARMOTORAXIS
#define INC_FEARMOTORAXIS

#include "ArmAxis.h"

struct gearmotorStatus{
  int8_t direction = 0;     // -1 negative, 0 stop, 1 positive
  int16_t raw_counts = 0;   // raw A to D counts
};

class GearmotorAxis : public ArmAxis
{
  public:
  GearmotorAxis(int16_t default_position, 
                     uint8_t feedback_pin,    // analog input pin
                     uint8_t forward_pin,     // moves feedback in the positive direction
                     uint8_t reverse_pin,     // moves feedback in the negative direction
                     int16_t max_position,    // tenths of degrees from center
                     int16_t center_counts,   //raw feedback position at center (zero angle)
                     int16_t min_position);
  bool run_axis();

  private: 
  uint8_t forward_pin;
  uint8_t reverse_pin;
  uint8_t feedback_pin;
  int16_t center_counts;
  int16_t zero_angle;

  
  bool zero_energy();
  int16_t read_rate();  
  int16_t read_position();

  void print();
  gearmotorStatus status;
};

#endif
