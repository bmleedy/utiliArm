#ifndef INC_AXISPROTOCOL
#define INC_AXISPROTOCOL

#define SERIAL_BUFFER_LENGTH        100   // bytes
#define SERIAL_READ_TIMEOUT  10   // ms, should take no more than 5ms
#define SERIAL_MESSAGE_START_CHAR        '#'
#define SERIAL_MESSAGE_TERMINATION_CHAR  '!'
#define SERIAL_MESSAGE_SPECIAL_CHECKSUM  '_'
#define SERIAL_MESSAGE_AXIS_SEPARATOR    '|'
#define SERIAL_MESSAGE_FIELD_SEPARATOR   ','
#define NUM_AXES 5

// #___,___|___,___|___,___|___,___|___,___|<8bitchecksum>!
// #rot_1,rate2|rot_2,rate2|.......|<8bitchecksum>!
// #111,222|333,444|555,666|777,888|999,000|_!

struct axis_field{
  int16_t position = 0;   // 1/10 deg
  int16_t rate = 0;       // 1/10 deg/s
};

void dump_serial_buffer(char * serial_buffer){
  Serial.print(F("buffer: ["));
  for(int i=0; i<50; i++){
    Serial.print(serial_buffer[i]);
  }
  Serial.println("]");
}

void parse_serial_fields(int16_t * output_array, 
                         uint16_t output_array_len, 
                         char * buffer, 
                         uint16_t start_index=1, 
                         uint16_t width = 4){
  for(int i=0; i<output_array_len; i++){
    buffer[start_index + i*width + width -1] = '\0';
  output_array[i] = atoi(buffer + start_index + i*width);
    //Serial.println(output_array[i]);
  }
}

bool is_bad_checksum(char * buffer, uint8_t checksum_index){
  if( buffer[checksum_index] == SERIAL_MESSAGE_SPECIAL_CHECKSUM)
    return false; // bypass check for this special sum. (yes, opens another collision)
  
  uint8_t sum = 0;
  for(int i = 0; i < checksum_index; i++){
    sum += buffer[i];
  }
  
  if(sum != buffer[checksum_index]){
    Serial.print(F("WARNING: Checksum expected [")); Serial.print(buffer[checksum_index]);
    Serial.print(F("] But received: [")); Serial.print(sum);
    Serial.println(F("] !"));
  }
}

class AxisProtocol{
  private:
  axis_field protocol_data[NUM_AXES];
  uint16_t received_serial = 0;
  
  public:
  void store_fields(int16_t source_data[]){
    for(int i = 0; i<NUM_AXES; i++){
      this->protocol_data[i].position = source_data[2*i];
      this->protocol_data[i].rate = source_data[2*i+1];
    }
    this->received_serial++;
  }
  
  void print(){
    Serial.println(F("Axis Protocol Data:"));
    for(int i=0; i<NUM_AXES; i++){
      Serial.print(F("  Axis["));Serial.print(i);Serial.print(F("] pos["));
      Serial.print(this->protocol_data[i].position);Serial.print(F("] rate["));
      Serial.print(this->protocol_data[i].rate);Serial.println(F("]"));
    }
    Serial.print(F("  Serial: ")); Serial.println(this->get_last_serial());
  }
  
  uint16_t get_last_serial(){return this->received_serial;}
  axis_field get_axis(uint8_t axis_num){return protocol_data[axis_num];}

  void update(char * serial_buffer){
    int16_t axis_field_protocol_data[NUM_AXES*2];
  
    // fill the 
    parse_serial_fields(axis_field_protocol_data, 
                        NUM_AXES*2, 
                        serial_buffer);
    
    this->store_fields(axis_field_protocol_data);
    // Check the data (starter, terminator, checksum)  todo: put all of these checks into the protocol class.
    if( !(serial_buffer[0] == '#') ){
      Serial.print(F("serialEvent() WARNING - no # found at start of serial message. Found: "));
      Serial.println(serial_buffer[0]);
      dump_serial_buffer(serial_buffer);
      return;
    } else if( is_bad_checksum(serial_buffer, 41 )){
      Serial.println(F("serialEvent() WARNING - bad checksum."));
      return;
    } else if(serial_buffer[ 4] != SERIAL_MESSAGE_FIELD_SEPARATOR &&
              serial_buffer[ 8] != SERIAL_MESSAGE_AXIS_SEPARATOR &&
              serial_buffer[12] != SERIAL_MESSAGE_FIELD_SEPARATOR &&
              serial_buffer[16] != SERIAL_MESSAGE_AXIS_SEPARATOR &&
              serial_buffer[20] != SERIAL_MESSAGE_FIELD_SEPARATOR &&
              serial_buffer[24] != SERIAL_MESSAGE_AXIS_SEPARATOR &&
              serial_buffer[28] != SERIAL_MESSAGE_FIELD_SEPARATOR &&
              serial_buffer[32] != SERIAL_MESSAGE_AXIS_SEPARATOR &&
              serial_buffer[36] != SERIAL_MESSAGE_FIELD_SEPARATOR &&
              serial_buffer[40] != SERIAL_MESSAGE_AXIS_SEPARATOR) {
      Serial.println(F("serialEvent() WARNING - separator format mismatch."));
      serial_buffer[42] = '\0';
      Serial.println(serial_buffer);
      return;
    }
  }
} ;

#endif
