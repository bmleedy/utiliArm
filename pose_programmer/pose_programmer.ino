#include "EEPROM.h"
#define NUM_TESTPOSES 12
#define NUM_AXES 5

// with 512 bytes space, I can store ~40 12-byte records

// Poses for the automated pose routine
struct pose{
  int16_t axis_pos[NUM_AXES];
  uint16_t duration_ms;
};

//  shoulder,     elbow,     wrist_bend,       wrist_rot,    claw
pose testposes[NUM_TESTPOSES] = {
  {.axis_pos={   0 ,    0,    0,    0,    0},.duration_ms= 3000},  //1
  {.axis_pos={ -500, 1200,    0,    0,    0},.duration_ms= 3000},  //2
  {.axis_pos={ -500, 1200, -450,  200,    0},.duration_ms= 3000},  //3
  {.axis_pos={ -500, 1200,  450,  200,    0},.duration_ms= 3000},  //4
  {.axis_pos={ -500, 1200,  450,  200,  700},.duration_ms= 3000},  //5
  {.axis_pos={ -500, 1200,  450,  200, -750},.duration_ms= 3000},  //6
  {.axis_pos={ -500, 1200,  450,  200,    0},.duration_ms= 3000},  //7
  {.axis_pos={  450,    0,    0,  200, -750},.duration_ms= 3000},  //8
  {.axis_pos={  450,    0,    0, -500, -750},.duration_ms= 1000}, //9
  {.axis_pos={  450,    0,    0,  500,  750},.duration_ms= 1000}, //10
  {.axis_pos={  450,    0,    0, -500, -750},.duration_ms= 1000}, //11
  {.axis_pos={  450,    0,    0,  500,  750},.duration_ms= 1000}  //12
};

// Class to read poses off of storage: todo: make shared instead of copied over
class RoutineStorage{
  uint16_t record_storage_start = 1;
  uint16_t current_pointer = 0;
  uint16_t current_record = 0;
  uint8_t num_records = 0;
  bool wrap = false;

  public:
  RoutineStorage(){
    EEPROM.get(current_pointer, num_records);
    current_pointer += sizeof(num_records);
    record_storage_start = current_pointer;
  }

  uint8_t get_next_record(pose * read_position){
    if(current_record++ < num_records){
      EEPROM.get(current_pointer, *read_position);
      current_pointer += sizeof(*read_position);
    } else if(wrap) {  //this will studder one iteration at the end
      current_pointer = record_storage_start;
      current_record = 0;
    } else {
      //do nothing, just leave it as it is
    }
    return current_record;
  }
  
};

RoutineStorage * storage;

void setup() {
  Serial.begin(115200);
  Serial.print(F("Starting... \n Writing ")); Serial.print(NUM_TESTPOSES); Serial.println(" records");

  int ee_write_addr = 0;
  uint8_t num_records = NUM_TESTPOSES;
  EEPROM.put(ee_write_addr, num_records);
  ee_write_addr+=sizeof(num_records);
  
  Serial.println(F("Writing Records..."));
  for(int i=0; i<NUM_TESTPOSES; i++){
    EEPROM.put(ee_write_addr, testposes[i]);
    ee_write_addr+=sizeof(testposes[i]);
    Serial.print(F("Record written : ")); Serial.println(i);
  }

  pose read_position;
  int ee_read_addr = 0;
  uint8_t number_of_records;

  Serial.println(F("Reading back records..."));
  storage = new RoutineStorage();

  int i = 0;
  while(storage->get_next_record(&read_position) <= NUM_TESTPOSES){

    Serial.print(F("  {.axis_pos={ ")); 
    for(int pos_i=0; pos_i < NUM_AXES; pos_i++){
      Serial.print(read_position.axis_pos[pos_i]); Serial.print(",");
    }
    Serial.print(F("},.duration_ms= "));Serial.print(read_position.duration_ms);
    Serial.print(F("},  // ")); Serial.println(++i);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
