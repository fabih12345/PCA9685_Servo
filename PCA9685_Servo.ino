#include <Wire.h>
#include "PCA9685_Servo.h"

float xyz[3]{0.,0.,Z90};


constexpr float p[15][3]={
{110.0, -55.0, -178.0},
{56.3, -58.8, -177.0},
{2.5, -62.5, -176.0},
{-51.3, -66.3, -175.0},
{-105.0, -70.0, -174.0},
{109.0, 5.0, -179.0},
{56.3, 1.3, -178.8},
{3.5, -2.5, -178.5},
{-49.3, -6.3, -178.3},
{-102.0, -10.0, -178.0},
{110.0, 68.0, -178.0},
{56.8, 64.5, -178.5},
{3.5, 61.0, -179.0},
{-49.8, 57.5, -179.5},
{-103.0, 54.0, -180.0}
};


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
  //if (servo.isRunning()){ return; }

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

    if (ch=='p'){
      uint8_t num = Serial.parseInt();
      cmd=Serial.read();
      Serial.print("position: p");
      Serial.println(num);
      if (num>15) { return; }
      xyz[0]=p[num][0];
      xyz[1]=p[num][1];
      xyz[2]=p[num][2];
      servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      return;
    }

    if (ch=='q'){
      uint8_t num = Serial.parseInt();
      cmd=Serial.read();
      Serial.print("position: q");
      Serial.println(num);
      if(num==0){
        xyz[0]=110.;
        xyz[1]=-60.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==1){
        xyz[0]=55.;
        xyz[1]=-60.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==2){
        xyz[0]=0.;
        xyz[1]=-65.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==3){
        xyz[0]=-53.;
        xyz[1]=-65.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==4){
        xyz[0]=-110.;
        xyz[1]=-75.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==5){
        xyz[0]=110.;
        xyz[1]=0.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==6){
        xyz[0]=55.;
        xyz[1]=0.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==7){
        xyz[0]=0.;
        xyz[1]=-5.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==8){
        xyz[0]=-53.;
        xyz[1]=-5.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==9){
        xyz[0]=-110.;
        xyz[1]=-15.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==10){
        xyz[0]=110.;
        xyz[1]=60.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==11){
        xyz[0]=55.;
        xyz[1]=60.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==12){
        xyz[0]=0.;
        xyz[1]=55.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==13){
        xyz[0]=-53.;
        xyz[1]=55.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
      if(num==14){
        xyz[0]=-110.;
        xyz[1]=45.;
        xyz[2]=-140.;
        servo.setXYZ(xyz[0],xyz[1],xyz[2]);
      }
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

    if (ch=='s'){
      cmd=Serial.read();
      Serial.println("stop!");
      servo.stop();
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
