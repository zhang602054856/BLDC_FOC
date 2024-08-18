#include <Arduino.h>
#include "InlineCurrent.h"
#include "lowpass_filter.h"

//  - shunt_resistor  - 分流电阻值
//  - gain  - 电流检测运算放大器增益
//  - phA   - A 相 adc 引脚
//  - phB   - B 相 adc 引脚
//  - phC   - C 相 adc 引脚（可选）

#define _ADC_VOLTAGE 3.3f            //ADC 电压
#define _ADC_RESOLUTION 4095.0f      //ADC 分辨率
#define _shunt_resistor  0.01f
#define _amp_gain  50
#define volts_to_amps_ratio  (1.0f /_shunt_resistor / _amp_gain) // volts to amps

// ADC 计数到电压转换比率求解
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

CurrSense::CurrSense(int id)
{
    constexpr uint8_t GPIO_ADC[] = {39, 36, 35, 34};

    pinA = GPIO_ADC[0 + id * 2];
    pinB = GPIO_ADC[1 + id * 2];

    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    // gain_c = volts_to_amps_ratio * -1;

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

    // 查找0电流时候的电压
    offset_ia = 0;
    offset_ib = 0;
    // offset_ic = 0;
    // 读数1000次
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        // if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        delay(1);
    }
    // 求平均，得到误差
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    // if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

void CurrSense::init()
{
    // 配置函数
    configureADCInline(pinA,pinB);
    // 校准
    calibrateOffsets();
}

// 读取全部三相电流
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