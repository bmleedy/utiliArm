#ifndef INC_ARMAXIS
#define INC_ARMAXIS

#include "Stdint.h"

#define POSITION_DEADBAND_MAX 100     //tenths of degrees
#define POSITION_DEADBAND_DEFAULT 50  //tenths of degrees
#define POSITION_DEADBAND_MIN   1     //tenths of degrees

#define REASON_POS_TOO_HIGH       0x0001
#define REASON_POS_TOO_LOW        0x0002
#define REASON_NEGATIVE_RATE      0x0004
#define REASON_RATE_DISABLED      0x0008

#define REASON_DEADBAND_TOO_LOW        0x0010
#define REASON_DEADBAND_TOO_HIGH       0x0020
#define REASON_STOP_FAILED             0x0040
#define REASON_RUN_FAILED              0x0080

#define REASON_MAX_LIM_OUT_OF_RANGE    0x0100
#define REASON_MIN_LIM_OUT_OF_RANGE    0x0200

class ArmAxis{
  protected:  //todo: make some of these private
  bool enabled = false;               // axis will go to safest, lowest-power state if false
  uint16_t disabled_reason = 0x0;// 0 = uninitialized; see #defines above for decoding
  int16_t desired_position = 0;  // tenth degrees, +/-, zero is centered, straight joint
  int16_t max_position = 450;    // max allowed command position
  int16_t min_position = -450;   // min allowed command position
  int16_t current_position = 0;  // last measured position
  unsigned long position_time_millis = 0; // when was last position measured?
  int16_t position_deadband = POSITION_DEADBAND_DEFAULT; // tenth degrees, unsigned.  Value is "within ____ of desired"

  
  bool pid_implemented = false;  // pid controller
  bool pid_enabled = false;      // false = bang-bang controller
  int16_t p_gain, i_gain, d_gain;
  int16_t p_max, i_max, d_max;
  int16_t i_accum;

  bool rate_enabled = false;     // ability to set rate of motion
  uint16_t max_rate = 0;         // tenths of rpm, unsigned
  int16_t desired_rate = 0;      // tenth degrees per second, +/-
  int16_t current_rate = 0;      // last measured rate

  bool set_max_position(int16_t pos);
  bool set_min_position(int16_t pos);
  bool set_max_rate(int16_t max_rate);
  
  virtual int16_t read_rate() = 0;
  virtual int16_t read_position() = 0;
  virtual bool zero_energy() = 0;
  virtual bool run_axis() = 0;
  
  public:

  // Constructor
  ArmAxis(int16_t max_position, int16_t min_position, int16_t default_position=0, int16_t max_rate=0);
  
  // Setters  False = failure / illegal operation
  bool set_desired_position(int16_t position_deg_tenths);
  bool set_desired_position(int16_t position_deg_tenths, int16_t rate_tenths_per_sec);
  bool set_position_deadband(int16_t deadband_deg_tenths);
  
  bool pid_init(int16_t pgain, int16_t igain, int16_t dgain,
                int16_t pmax, int16_t imax, int16_t dmax);  // not impl yet
  bool stop_axis();                            // stop and hold axis where it is
  bool disable_axis(uint16_t reason);  // go to the safest possible state

  // Getters
  uint16_t get_position(){return this->read_position();}
  uint16_t get_rate(){return this->read_rate();}
  bool is_enabled(){return this->enabled;}
  uint16_t get_disabled_reason();
  
  // Loop  False = error, the axis has disabled itself for safety
  bool run();
  virtual void print() = 0;

};

#endif
