int rc_byte_[16];
int rc[8];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

}

void loop() {
  if(Serial.available()>0)
  {
    for(int nik = 0; nik < 18; nik++)
    {
      rc_byte_[nik] = Serial.read();
      //Serial.print(rc_byte_[nik]);
      //Serial.print(" ");
    }
    //Serial.println("");
  }

  if(rc_byte_[0] == 14 && rc_byte_[17] == 22)
  {
    for(int nik = 0; nik < 8; nik++)
    {
      rc[nik] = (rc_byte_[2*nik + 1] << 8 | rc_byte_[2*nik + 2]) & 0xFFFF;
      Serial.print(rc[nik]);
      Serial.print(" ");
    }
    
    Serial.println("");
  }

  
  

}
