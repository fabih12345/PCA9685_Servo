#include "PCA9685_Servo.h"
#include <Arduino.h>

PCA9685_Servo::PCA9685_Servo(){}

void PCA9685_Servo::loop(){
    if (!running){ return; }

    float s0{0};
    float s1{0};
    float s2{0};

    if ( millis()<=endtime ){
        uint32_t now = millis();
        if (now>endtime){ now=endtime; }
        const double time = (endtime-now)/1000.;
        actual_xyz[0] = xyz[0]-speed_xyz[0]*time;
        actual_xyz[1] = xyz[1]-speed_xyz[1]*time;
        actual_xyz[2] = xyz[2]-speed_xyz[2]*time;
        calculateAngle(actual_xyz[0], actual_xyz[1],actual_xyz[2], s0, s1, s2);
        setAngle(s0, s1, s2);
    } else {
        actual_xyz[0] = xyz[0];
        actual_xyz[1] = xyz[1];
        actual_xyz[2] = xyz[2];

        calculateAngle(actual_xyz[0], actual_xyz[1],actual_xyz[2], s0, s1, s2);
        setAngle(s0, s1, s2);
        Serial.print(millis()/1000.);
        Serial.println(" arrived!");
        running=false;
    }
}

void PCA9685_Servo::begin(){
    Wire.begin();
    Wire.setClock(400000);
    write8(PCA9685_MODE1, MODE1_RESTART);
    float prescaleval = ((FREQUENCY_OSCILLATOR / (FREQ * 4096.0)) + 0.5) - 1;
    uint8_t prescale = static_cast<uint8_t>(prescaleval);
    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    write8(PCA9685_MODE1, newmode);
    write8(PCA9685_PRESCALE, prescale);
    write8(PCA9685_MODE1, oldmode);
    delay(5);
    write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
    running=false;
    pinMode(2, OUTPUT);
    digitalWrite(2,LOW);
}

void PCA9685_Servo::write8(uint8_t reg, uint8_t dta){
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(dta);
    Wire.endTransmission();
    delay(10);
}

uint8_t PCA9685_Servo::read8(uint8_t reg){
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(PCA9685_ADDR, static_cast<uint8_t>(1));
    return Wire.read();
}

void PCA9685_Servo::setPWM(uint8_t num, uint16_t on, uint16_t off) {
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(LED0_ADDR + 4 * num);
    Wire.write(on);
    Wire.write(on >> 8);
    Wire.write(off);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

uint16_t PCA9685_Servo::angleToTicks(float angle){
    constexpr float MIN {NULL_GRAD -5 * TICK_PER_GRAD};
    constexpr float MAX {NULL_GRAD + 185 * TICK_PER_GRAD};
    float tmp_ticks = NULL_GRAD + angle * TICK_PER_GRAD;
    if (tmp_ticks>MAX) { tmp_ticks=MAX;}
    if (tmp_ticks<MIN) { tmp_ticks=MIN;}
    return static_cast<uint16_t>(tmp_ticks);
}

void PCA9685_Servo::setFeed(const float feed_){
    feed=feed_;
}

void PCA9685_Servo::setCorr(const uint8_t s, const float val){
    if (s>2) {return;}
    angleCorr[s]=val;
}

void PCA9685_Servo::run(){
    running=true;
    Serial.print("moving to x=");
    Serial.print(xyz[0]);
    Serial.print(" y=");
    Serial.print(xyz[1]);
    Serial.print(" z=");
    Serial.println(xyz[2]);

    const double dx= xyz[0]-actual_xyz[0];
    const double dy= xyz[1]-actual_xyz[1];
    const double dz= xyz[2]-actual_xyz[2];
    const double dist = sqrt(dx*dx+dy*dy+dz*dz);
    const double time = dist / feed;
    if (time==0) {endtime=millis()-1; return;}
    speed_xyz[0] = dx/time;
    speed_xyz[1] = dy/time;
    speed_xyz[2] = dz/time;
    endtime=millis()+time*1000;
}

void PCA9685_Servo::off(){
    setPWM(0,0,4096);
    setPWM(1,0,4096);
    setPWM(2,0,4096);
    running=false;
}

void PCA9685_Servo::setAngle(float s0, float s1, float s2){
    setPWM(0,0,angleToTicks(s0+angleCorr[0]));
    setPWM(1,0,angleToTicks(s1+angleCorr[1]));
    setPWM(2,0,angleToTicks(s2+angleCorr[2]));
}

void PCA9685_Servo::setXYZ(float x, float y, float z){
    xyz[0]=x;
    xyz[1]=y;
    xyz[2]=z;
}


void PCA9685_Servo::calculateAngle(float x, float y, float z, float& s0, float& s1, float& s2){
    const double E0=2.*RL*(RA+y);
    const double E1=-RL*(1.73205080757*(x+RB)+RC+y);
    const double E2=RL*(1.73205080757*(x-RB)-RC-y);
    const double F=2.*RL*z;
    const double G0=x*x+y*y+z*z+RA*RA+RL*RL+2.*RA*y-RI*RI;
    const double G1=x*x+y*y+z*z+RB*RB+RC*RC+RL*RL+2.*(x*RB+y*RC)-RI*RI;
    const double G2=x*x+y*y+z*z+RB*RB+RC*RC+RL*RL+2.*(-x*RB+y*RC)-RI*RI;

    const double T0=(-F-sqrt(E0*E0+F*F-G0*G0))/(G0-E0);
    const double T1=(-F-sqrt(E1*E1+F*F-G1*G1))/(G1-E1);
    const double T2=(-F-sqrt(E2*E2+F*F-G2*G2))/(G2-E2);

    s0=(2*atan(T0)*4068.)/71.+90.;
    s1=(2*atan(T1)*4068.)/71.+90.;
    s2=(2*atan(T2)*4068.)/71.+90.;

    /* Serial.print("s0=");
    Serial.print(s0);
    Serial.print(" s1=");
    Serial.print(s1);
    Serial.print(" s2=");
    Serial.println(s2); */
}

void PCA9685_Servo::setMagnet(uint8_t num){
    if (num==0) {
        digitalWrite(2, LOW);
    } else {
        digitalWrite(2, HIGH);
    }
}