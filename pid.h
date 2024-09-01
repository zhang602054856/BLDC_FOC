#ifndef PID_H
#define PID_H

#define _constrain(val, low, high) ((val)<(low)?(low):((val)>(high)?(high):(val)))

enum {
    // PID_MODE_ID = 0,
    FOC_MODE_VEL,
    FOC_MODE_POS,
    FOC_MODE_POS_FEED,
    FOC_MODE_POS_RATCHET,
    FOC_MODE_NUM,
} FOC_MODE;

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

    void setup(float P, float I, float D);
    void setDebug(int cmd, float set);
    void setMaxOutput(float max) { out_max = max; }
    float process(float target, float current);

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
