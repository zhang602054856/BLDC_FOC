#include <Arduino.h>

#include "Wire.h"
#include "AS5600.h"
#include "lowpass_filter.h"


#define _PI 3.14159265359f
#define _2PI 6.28318530718f
#define _1_6_PI 5.026548245743f

#define AS5600_ADDR 0x36
#define AS5600_ANGLE_REG 0x0C
#define AS5600_I2C_CLK 400000UL

static int count = 0;
constexpr int I2C_PIN[] = {19, 18, 23, 5};

AngleSensor::AngleSensor(uint8_t id, int dir, uint8_t poles)
    : id(id),
      direction(dir),
      pole_pairs(poles)
{
    timestamp_now = timestamp_prev = micros();
    round = round_prev = 0;
    full_radian_cur = full_radian_prev = 0;
    radian_cur = radian_prev = 0;

    velocity_rad = 0;
    initial_degree = 0;
    velocity = new LowPassFilter(0.05); //Tf = 50ms
    position = new LowPassFilter(0.05); //Tf = 50ms

    wire = new TwoWire(id);
}

void AngleSensor::init()
{
    wire->begin(I2C_PIN[0 + id * 2], I2C_PIN[1 + id * 2], AS5600_I2C_CLK);
    wire->beginTransmission(AS5600_ADDR);
    wire->write(AS5600_ANGLE_REG);
    wire->endTransmission(false);

    float _raw = 0.0f;
    for (int i = 0; i < 100; i++) {
         _raw += getRawAngle();
    }
    initial_degree = _raw *0.01;/// 100;

    printf("as5600 initial degree = %d\n", initial_degree);
}

uint16_t AngleSensor::getRawAngle()
{
    byte readArray[2];
    wire->requestFrom(AS5600_ADDR, (uint8_t)2);

    for (int i = 0; i < 2; i++) {
        readArray[i] = wire->read();
    }
    uint16_t raw_angle = ((readArray[0] << 8) | readArray[1]);
    uint16_t degree = (raw_angle * 360) >> 12;

    return degree;
}
/*
* bref: convert the angle to radian
* input angle: [0, 360]
* return: range [0, 2_PI]
*/
float AngleSensor::normalizeAngle(const float angle)
{
    float a = fmod(angle, _2PI);
    if (a < 0.0) {
        a += _2PI;
    }
    return a;
}
/*
* bref: return the physical_rad
* return: range [0, 2_PI]
*/
float AngleSensor::getAbsRadian()
{
    return direction * radian_cur;
}

float AngleSensor::getElectricAngle(bool is_mock)
{
    float physical_rad = 0;
    if (is_mock) {
        static int physical_ang = 0;
        physical_ang += 2;
        if (physical_ang >= 360) physical_ang = 0;

        physical_rad = physical_ang * _PI / 180.0f;
    }
    else {
        physical_rad = getAbsRadian();
    }

    float ele_angle = normalizeAngle(physical_rad * pole_pairs);

    // printf("physical_rad = %f, ele_angle=%f\n", physical_rad, ele_angle);

    return ele_angle;
}

float AngleSensor::getFullRadian()
{
    return direction * position->process(full_radian_cur);
}

float AngleSensor::getVelocity()
{
    return direction * velocity->process(velocity_rad);
}

void AngleSensor::sensorUpdate()
{
    timestamp_now = micros();

    uint16_t degree = getRawAngle();
    // uint16_t deg_aligned = (degree >= initial_degree) ?
    //             (degree - initial_degree) : (360 - initial_degree + degree);

    uint16_t deg_aligned = (degree >= initial_degree) ?
                (360 - degree + initial_degree) : (initial_degree - degree);

    radian_cur = deg_aligned * 0.017453292519;//_PI / 180;

    float delta_radian = radian_cur - radian_prev;
    if(fabs(delta_radian) > _1_6_PI ) {
        round += ( delta_radian > 0 ) ? -1 : 1;
    }

    full_radian_cur = round * _2PI + radian_cur;

    float Ts = (timestamp_now - timestamp_prev) * 1e-6;
    if(Ts <= 0) {
        Ts = 1e-3f;
        printf("system timer overflowed\n");
        while(1);
    }

    velocity_rad = (full_radian_cur - full_radian_prev) / Ts;

    // if (count++ > 200 ) {
    //     count = 0;
    //     printf("angle: %d, %f, %d, %f\n", deg_aligned, radian_cur, round, velocity_rad);
    // }

    radian_prev = radian_cur;
    full_radian_prev = full_radian_cur;
    timestamp_prev = timestamp_now;
}

