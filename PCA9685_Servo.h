#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H
#include <Wire.h>

// SG90 cont. -100%=150Ticks; 0%=299..318Ticks; 100%=480
// SG90 rot. 0°=150; 90°=340; 180°=540
// Futaba rot. min=60; max=480; 0°=92; 90°=280; 180°=477  ==> 2,14Ticks/° ^= 0,47°/Tick

constexpr float NULL_GRAD {92};
constexpr float TICK_PER_GRAD {2.14f};

constexpr uint8_t PCA9685_ADDR {0x40};
constexpr uint8_t PCA9685_SW_RESET {0x06};
constexpr uint8_t LED0_ADDR {0x06};
constexpr uint8_t LED_TOGGLE {0b00010000};
constexpr uint8_t PCA9685_PRESCALE {0xFE};
constexpr uint8_t PCA9685_MODE1 {0x00};
constexpr uint8_t MODE1_RESTART {0x80};
constexpr uint8_t MODE1_SLEEP {0x10}; // sleep, osc off
constexpr uint8_t MODE1_AI {0x20}; // autoincrement     
constexpr uint32_t FREQUENCY_OSCILLATOR {25000000};
constexpr float FREQ {50.};
constexpr uint16_t INTERVAL {50};

// robot parameters
constexpr float RL {87.};
constexpr float RI {214.};
constexpr float RA {27.3};
constexpr float RB {-23.6};
constexpr float RC {-13.7};
constexpr float Z90 {-180.9};
constexpr float FEED {20.};
constexpr float C0 {-7.};
constexpr float C1 {-7.};
constexpr float C2 {-4.5};


class PCA9685_Servo
{
private:
    float angleCorr[3]{C0,C1,C2}; // °
    float xyz[3]{0.,0.,Z90}; //mm
    float actual_xyz[3]{0.,0.,Z90}; //mm
    float speed_xyz[3]{0.,0.,0.}; //mm

    float feed{FEED}; // mm/s
    bool running;
    uint32_t lasttime{0};
    uint32_t endtime;

    void write8(uint8_t reg, uint8_t dta);
    uint8_t read8(uint8_t reg);
    uint16_t angleToTicks(float angle);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void setAngle(float s0, float s1, float s2);
    void calculateAngle(float x, float y, float z, float& s0, float& s1, float& s2);

public:
    PCA9685_Servo();
    void begin();
    void setFeed(float feed);
    void setCorr(uint8_t s, float val);
    void setXYZ(float x, float y, float z);
    void setMagnet(uint8_t num);
    void run();
    void off();
    void loop();
    void stop();

    bool isRunning();

};

#endif