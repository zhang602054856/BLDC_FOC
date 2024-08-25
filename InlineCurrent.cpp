#include <Arduino.h>

#include "InlineCurrent.h"
#include "lowpass_filter.h"

#define ADC_VOLTAGE         3.3f         // ADC 电压
#define ADC_RESOLUTION      4095.0f      // ADC width
#define SHUNT_RESISTOR      0.01f        // 分流电阻值
#define AMP_GAIN            50           // 电流检测运算放大器增益
#define VOLT_TO_AMP_RATIO   (1.0f / SHUNT_RESISTOR / AMP_GAIN)

// ADC 计数到电压转换比率求解
#define _ADC_CONV ( (ADC_VOLTAGE) / (ADC_RESOLUTION) )

CurrSense::CurrSense(int id)
{
    constexpr uint8_t GPIO_ADC[] = {39, 36, 35, 34};

    pinA = GPIO_ADC[0 + id * 2];
    pinB = GPIO_ADC[1 + id * 2];

    // gains for each phase
    gain_a = VOLT_TO_AMP_RATIO;
    gain_b = VOLT_TO_AMP_RATIO;
    // gain_c = VOLT_TO_AMP_RATIO * -1;

    phase_a = new LowPassFilter(0.005);
    phase_b = new LowPassFilter(0.005);
}

float CurrSense::readADCVoltageInline(const int pin)
{
    uint32_t raw_adc = analogRead(pin);
    return raw_adc * _ADC_CONV;
}

void CurrSense::configureADCInline(const int pinA,const int pinB)
{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    // if( _isset(pinC) ) pinMode(pinC, INPUT);
}

// 查找 ADC 零偏移量的函数
void CurrSense::calibrateOffsets()
{
    const int calibration_rounds = 1000;

    offset_ia = 0;
    offset_ib = 0;
    // offset_ic = 0;
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        // if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        delay(1);
    }

    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    // if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

void CurrSense::init()
{
    configureADCInline(pinA,pinB);

    calibrateOffsets();
}

void CurrSense::getPhaseCurrents()
{
    current_a = (readADCVoltageInline(pinA) - offset_ia)*gain_a;// amps
    current_b = (readADCVoltageInline(pinB) - offset_ib)*gain_b;// amps
    // current_c = (!_isset(pinC)) ? 0 : (readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
}

float CurrSense::getCurentA()
{
    // float _a = phase_a->process(current_a);
    // printf("current A: %f, %f\n", current_a, _a);
    return current_a;
}

float CurrSense::getCurentB()
{
    // float _b = phase_b->process(current_b);
    return current_b;
}