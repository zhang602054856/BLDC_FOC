// #include <stdlib.h>
#include <Arduino.h>

#include "pid.h"

pid::pid(const char *name, float P, float I, float D,
         float i_factor, float max_in, float max_out)
    : id(name),
      state(true),
      kp(P),
      ki(I),
      kd(D),
      ki_factor(i_factor),
      in_max(max_in),
      out_max(max_out)
{
    deviation = 0;
    deviation_sum = 0;
    deviation_prev = 0;
    pid_out = 0;

    printf("%s, P=%f, I=%f, D=%f, max=[%.4f=>%.4f]\n", name, P, I, D, max_in, max_out);
}

void pid::setup(float P, float I, float D)
{
    kp = P;
    ki = I;
    kd = D;
    deviation_sum = 0;
}

static int count = 0;
float pid::process(float target, float current)
{
    if (state == false) {
        deviation_sum = 0;
        deviation = 0;
        deviation_prev = 0;
        pid_out = 0;
    }
    else {
        float _target = _constrain(target, -in_max, in_max);

        deviation = _target - current;

        if (fabs(deviation) <= in_max * ki_factor) {
            deviation_sum += ki * deviation;
            deviation_sum = _constrain(deviation_sum, -out_max, out_max);
        }
        else {
            deviation_sum = 0;
        }

        float P = kp * deviation;
        float I = deviation_sum;
        float D = kd * (deviation - deviation_prev);

        pid_out = _constrain((P + I + D), -out_max, out_max);

        deviation_prev = deviation;
        // if ((strcmp(id, "position") == 0) && count++ > 100 ) {
        //     count = 0;
        //     printf("pid: %f, %f, %f, %f\n", P, I, D, pid_out);
        // }
    }

    return pid_out;
}

void pid::setDebug(int cmd, float set)
{
    switch (cmd) {
        case FOC_PID_STATE:
            state = (set == 0) ? false : true;
            break;

        case FOC_PID_KP:
            kp = set;
            break;

        case FOC_PID_KI:
            ki = set;
            if (set == 0) {
                deviation_sum = 0;
            }
            break;

        case FOC_PID_KD:
            kd = set;
            if (set == 0) {
                deviation_prev = 0;
            }
            break;
        case FOC_PID_KI_FACTOR:
            ki_factor = set;
            break;
    }
}