#include <Arduino.h>

class LowPassFilter;

class CurrSense
{
  public:
    CurrSense(int id);
    void init();
    void getPhaseCurrents();
    float getCurentA();
    float getCurentB();

private:
    float readADCVoltageInline(const int pin);
    void configureADCInline(const int pinA, const int pinB);
    void calibrateOffsets();

private:
    // int _Mot_Num;
    float current_a,current_b;//,current_c;
    LowPassFilter *phase_a, *phase_b;
    int pinA;
    int pinB;
    // int pinC;
    float offset_ia;
    float offset_ib;
    // float offset_ic;
    // float _shunt_resistor;
    // float amp_gain;

    // float volts_to_amps_ratio;

    float gain_a;
    float gain_b;
    // float gain_c;

};
