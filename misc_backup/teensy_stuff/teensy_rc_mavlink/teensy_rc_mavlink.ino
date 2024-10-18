#include <checksum.h>
#include <ardupilotmega/mavlink.h>

int rc[8];

void request_datastream() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(1, 100, &msg, 1, 0, 511, 0, MAVLINK_MSG_ID_RC_CHANNELS, 100, 1, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); 
  Serial1.write(buf, len); 
}

void MavLink_receive()
{

  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial1.available())
  { 
    
    uint8_t c= Serial1.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    { 
     
      switch(msg.msgid)
        {
          case  MAVLINK_MSG_ID_RC_CHANNELS:
          {
            mavlink_rc_channels_t rc_raw;
            mavlink_msg_rc_channels_decode(&msg, &rc_raw);
            rc[0] = rc_raw.chan1_raw;
            rc[1] = rc_raw.chan2_raw;
            rc[2] = rc_raw.chan3_raw;
            rc[3] = rc_raw.chan4_raw;
            rc[4] = rc_raw.chan5_raw;
            rc[5] = rc_raw.chan6_raw;
            rc[6] = rc_raw.chan7_raw;
            rc[7] = rc_raw.chan8_raw;
            }
            break;
        }
        
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600); 
}

void loop() {
 request_datastream();
 MavLink_receive();

 for(int nik = 0; nik < 8; nik++)
 {
  Serial.print(rc[nik]);
  Serial.print(" ");
 }

 Serial.println("");

 delay(10);
  
}
