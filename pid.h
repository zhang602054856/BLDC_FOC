#ifndef PID_H
#define PID_H

#define _constrain(val, low, high) ((val)<(low)?(low):((val)>(high)?(high):(val)))

enum {
    FOC_PID_MODE_ID = 0,
    FOC_PID_MODE_VELOCITY,
    FOC_PID_MODE_POSITION,
    FOC_PID_MODE_MAX,
} FOC_DEBUG_MODE;

enum {
    FOC_PID_STATE = 0,
    FOC_PID_KP,
    FOC_PID_KI,
    FOC_PID_KD,
    FOC_PID_KI_FACTOR,
} FOC_DEBUG_CMD;


class pid {
    public:
        pid(const char *name,
            float P, float I, float D,
            float i_factor, float max_in, float max_out);
        ~pid() = default;

        float process(float target, float current);
        void setup(float out_max);
        void setDebug(int cmd, float set);
    private:
        const char *id;
        bool state;
        float kp;
        float ki;
        float kd;
        float ki_factor;
        float in_max;
        float out_max;

        float deviation;
        float deviation_sum;
        float deviation_prev;
        float pid_out;
};

#endif
