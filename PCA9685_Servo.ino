#include <Wire.h>
#include "PCA9685_Servo.h"

float xyz[3]{0.,0.,Z90};

PCA9685_Servo servo;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Servo Test");
  servo.begin();
}

void loop() {
  servo.loop();
  if (Serial.available()){
    uint8_t ch = Serial.peek();
    if (ch == '\r'){
      while (Serial.available()) {Serial.read();}
      return;
    }
    char cmd{'-'};

    if (ch=='i'){
      Serial.print(millis()/1000.);
      Serial.println(" running!");
      cmd=Serial.read();
      servo.run();
      return;
    }

    if (ch=='r'){
      Serial.print(millis()/1000.);
      Serial.println("switching off!");
      cmd=Serial.read();
      servo.off();
      return;
    }

    if (ch=='f'){
      cmd=Serial.read();
      Serial.print("feed: ");
      Serial.print(cmd);
      Serial.print("=");
      float num = Serial.parseFloat();
      servo.setFeed(num);
      Serial.println(num,DEC);
      return;
    }

     if (ch=='h'){
      cmd=Serial.read();
      Serial.println("home!");
      xyz[0]=0.;
      xyz[1]=0.;
      xyz[2]=-170.;
      servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      servo.run();
      return;
    }

    if (ch=='m'){
      cmd=Serial.read();
      Serial.print("magnet: ");
      Serial.print(cmd);
      Serial.print("=");
      uint8_t num = Serial.parseInt();
      if (num>1) { return; }
      servo.setMagnet(num);
      Serial.println(num,DEC);
      return;
    }

    if (ch=='a' || ch=='b' || ch=='c'){
      cmd=Serial.read();
      Serial.print("correction: ");
      Serial.print(cmd);
      Serial.print("=");
      float num = Serial.parseFloat();
      servo.setCorr(ch-'a',num);
      Serial.println(num,DEC);
      return;
    }

    if (ch=='x' || ch=='y' || ch=='z'){
      cmd=Serial.read();
      Serial.print("coordinate: ");
      Serial.print(cmd);
      Serial.print("=");
      float num = Serial.parseFloat();
      xyz[cmd-'x']=num;
      Serial.println(num,DEC);
      servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      return;
    }

    if (Serial.available()){
      Serial.print("unknown command: ");
      Serial.println(static_cast<char>(Serial.read()));
      while (Serial.available()) {Serial.read();}
    }
  }
}
