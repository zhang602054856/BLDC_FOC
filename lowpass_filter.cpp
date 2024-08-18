 #include <Arduino.h>

#include "lowpass_filter.h"


LowPassFilter::LowPassFilter(float time_constant)
    :   Tf(time_constant),
        value(0.0f)
{
    timestamp_prev = micros();
}

float LowPassFilter::process(float x)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        value = x;
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = Tf / (Tf + dt);
    value = alpha * value + (1.0f - alpha) * x;
    timestamp_prev = timestamp;

    return value;
}
// velocity::velocity()
//     : fliter(0.02),  // Tf = 10ms
//       radian_vel(0),
//       rpm(0)
// {
// }

// void velocity::update(float x)
// {
//     radian_vel = fliter.process(x);
//     rpm = 9.5493f * radian_vel;
// }

//   _velocity.update(direction * _vel);

