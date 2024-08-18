#include <Arduino.h>
#include "Wire.h"

class LowPassFilter;

class AngleSensor
{
  public:
    AngleSensor(uint8_t id, int dir, uint8_t poles);
    // void Sensor_init();

    float getElectricAngle(bool is_mock = false);
    float getVelocity();
    float getFullRadian();

    void sensorUpdate();
    void init();

  private:
    float normalizeAngle(const float angle);
    uint16_t getRawAngle();
    float getAbsRadian();

  private:
    uint8_t id;
    int direction;
    uint8_t pole_pairs;

    TwoWire* wire;
    LowPassFilter *velocity; // rad/s
    LowPassFilter *position;
    // int16_t degree_angle;
    float radian_cur;
    float radian_prev;

    float full_radian_cur;
    float full_radian_prev;
    float velocity_rad;

    int16_t initial_degree;

    unsigned long timestamp_prev;
    unsigned long timestamp_now;
    int32_t round;
    int32_t round_prev;
};
