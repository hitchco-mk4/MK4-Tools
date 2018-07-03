// data block tester, all this does is maintain a data block and pass it to the serial port

#include "dataBlock.h"

unsigned long previous_time = 0;
unsigned long previous_count_time = 0;
bool camera_state = false;

int count = 0; 

dataBlock db; // data block for talking with the Raspberry Pi

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Serial3.begin(9600);    // Communicates with the Raspberry Pi, should NOT be changed
  
  pinMode(A10, INPUT);
  pinMode(13, OUTPUT);
}

void loop() {

  bool reverse = true;

  if (digitalRead(A10)) {
    reverse = false; 
  }
  
  unsigned long now = millis();

  if (now - previous_time > 2000) { // every 5 seconds
    camera_state = !camera_state;
    // digitalWrite(13, camera_state);
    previous_time = now;
  }

  if (now - previous_count_time > 200) {
    count++;
    if (count > 100) {
      count = 0;
    }
    previous_count_time = now;
  }

  count = map(analogRead(8), 0, 1000, 0, 100);


  int ldr = analogRead(0);
  // Serial.println(ldr);
  
  // data block mapping
  db.db_f0 = (float)map(count, 0, 100, 0, 8000); // rpm
  db.db_f1 = (float)map(count, 0, 100, 0, 200); // mph
  db.db_f2 = (float)map(count, 0, 100, 0, 20); // vbat
  db.db_f3 = (float)map(count, 0, 100, 0, 100);; // oilp
  db.db_f4 = (float)map(count, 0, 100, 0, 100); // fuel
  db.db_f5 = (float)map(count, 0, 100, 0, 2); // egol
  db.db_f6 = (float)map(count, 0, 100, 0, 2); // egor
  db.db_f7 = 0; // miles this trip
  db.db_f8 = (float)map(count, 0, 100, 0, 8); // batc
  db.db_f9 = (float)map(count, 0, 100, 0, 100); // oill
  db.db_f10 = 0; // unused
  db.db_f11 = 0; // unused
  
  db.db_i0 = (int)map(count, 0, 100, 0, 300); // ect
  db.db_i1 = (int)map(count, 0, 100, 0, 300); // act
  db.db_i2 = 2; // state  
  db.db_i3 = ldr; // ldr

  db.db_b0 = (byte)map(count, 0, 100, 0, 1); // ebrk
  // db.db_b1 = camera_state; // rvrs
  db.db_b2 = false; // shutoff
  db.db_b3 = (byte)map(count, 0, 100, 0, 1); // left
  db.db_b4 = (byte)map(count, 0, 100, 0, 1); // right
  db.db_b5 = (byte)map(count, 0, 100, 0, 1); // css
  db.db_b6 = (byte)map(count, 0, 100, 0, 1); // lite

  process_message(Serial3);

}

void process_message(Stream &port) {
  // process messages from the Raspberry Pi. Incoming signals are NOT checksumed, this should be implemented if anything of value is to happen
  
  if (port.available() > 0) {
    digitalWrite(13, HIGH);
    
    char b = port.read();

    Serial.println(atoi(b));
    Serial.println(b);

    switch (b) {
    case '0':
      break;

    case '1':
      break;
    }

    db.send_data_block(port);
    
    /*
    for (int i = 0; i < 64; i++) {
      Serial.print(db.data_block[i]);
      Serial.print(",");
    }

    Serial.println("");
    */
    
    digitalWrite(13, LOW);
  }
}
