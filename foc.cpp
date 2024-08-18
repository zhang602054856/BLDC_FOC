#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "foc.h"
#include "InlineCurrent.h"

static int count = 0;

constexpr float _PI = 3.14159265359f;
constexpr float _2PI = 6.28318530718f;
constexpr float _PI_2 = 1.57079632679f;
constexpr float _3PI_2 = 4.71238898038f;

constexpr float _SQRT3 = 1.73205080757f;
constexpr float _1_SQRT3 = 0.57735026919f;  // 1/√3 = √3/3
constexpr float _2_SQRT3 = 1.15470053838f;  // 2√3/3

constexpr float _RPM_FACTOR = 9.54929658551f;

constexpr float max_current = 0.5f;
constexpr float max_velocity = 100.0f;

//#define __unused __attribute__((unused))

bldc::bldc(int id, float power/*, uint8_t poles*/)
    : id(id),
      pole_pairs(7),
      supply_power(power)
{
    printf("bldc id = %d, U_max = %f\n", id, power);
}

int bldc::setup()
{
    constexpr int GPIO_PWM_TABLE[] = {32, 33, 25, 26, 27, 14};

    if (id >= MOTOR_NUMB) {
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        int index = i + id * 3;
        int gpio_pwm = GPIO_PWM_TABLE[index];

        pinMode(gpio_pwm, OUTPUT);
        ledcSetup(index, 30000, 11);  //pwm channel, freq, solution 2^11
        ledcAttachPin(gpio_pwm, index);

        printf("gpio(%d) => pwm(%d)\n", index, gpio_pwm);
        gpio_index[i] = index;
    }

    Serial.println("pwm gpio initialized");
    return 0;
 }


void bldc::setPwm(float Ta, float Tb, float Tc)
{
    // 限制占空比从0到1
    float d_a = _constrain(Ta, 0.0f , 1.0f );
    float d_b = _constrain(Tb, 0.0f , 1.0f );
    float d_c = _constrain(Tc, 0.0f , 1.0f );

    // 写入PWM到PWM 0 1 2 通道
    ledcWrite(gpio_index[0], d_a * 2048);
    ledcWrite(gpio_index[1], d_b * 2048);
    ledcWrite(gpio_index[2], d_c * 2048);
}


foc::foc(bldc *m, CurrSense* cur, AngleSensor* ang)
  : section(0),
    rpm(0),
    motor(m),
    current(cur),
    angle(ang)
{
    Uq = 0;
    Ud = 0;
    Udc = m->getMaxPower();
    Uq_max =  _1_SQRT3 * Udc;

    // kp=0.2, i=0.0001 , d=150
    // spring affect: p=0.01, i=0, d=0.9
    // position_pid = new pid("position", 0.4, 0, 160, 1, 200, max_current);
    position_pid = new pid("position", 4, 0, 0, 1, 200, 50);

    // velocity: p=0.2,i=0.0005,d=0.05,input Max=100rad/s
    // speed_pid = new pid("velocity", 0.01, 0.0001, 0.1, 0.06, max_velocity, max_current);
    speed_pid = new pid("velocity", 0.01, 0, 0, 1, max_velocity, max_current);

    // torque:inputMax=1A,outputMax=(√3/3)*Udc
    torque_d_pid = new pid("torque_d", 3, 0.04, 0, 1, max_current, Uq_max);
    torque_q_pid = new pid("torque_q", 3, 0.04, 0, 1, max_current, Uq_max);

    Id_fliter = new LowPassFilter(0.02);
    Iq_fliter = new LowPassFilter(0.02);

    printf("Uq_max = %f\n", Uq_max);
}

void foc::alignAngle()
{
    float _elec_angle = 0;

    motor->setup();
    current->init();

    setTorque(3, 0, _3PI_2);   //start
    delay(500);

    angle->init();

    setTorque(0, 0, 0);        //end
    // delay(500);
}

void foc::updateSensors()
{
    angle->sensorUpdate();
    current->getPhaseCurrents();
}

void foc::setTorque(float target_uq, float target_ud, float elec_angle)
{
    Uq = _constrain(target_uq, -Uq_max, Uq_max);
    Ud = _constrain(target_ud, -Uq_max, Uq_max);

    invParkSvpwm(Uq, Ud, elec_angle);
}

/*
*  brief: current loop pid control
*         calculate _ref_uq according to input target.
*         _ref_ud generated by target ud that always 0.
*  input: target current: [-0.7, 0.7] unit: A
*/
void foc::setTargetCurrent(float target_iq, float target_id)
{
    float _angle = angle->getElectricAngle();

    clarkPark(_angle);

    float _ref_ud = torque_d_pid->process(target_id, Id);
    float _ref_uq = torque_q_pid->process(target_iq, Iq);
    // if (count++ > 100 ) {
    //     count = 0;
    //     printf("torque: %f, %f, %f\n", target_id, Id, _ref_ud);
    // }
    setTorque(_ref_uq, _ref_ud, _angle);
}

/*
*  brief: velocity loop pid control
*         calculate _ref_Iq according to target velocity.
*         the range is [-0.6 , 0.6]
*  input: target velocity: [-100, 100] unit: rad/s
*/
void foc::setTargetVelocity(float target)
{
    float _velocity = angle->getVelocity();
    float _ref_Iq = speed_pid->process(target, _velocity);

    // if (count++ > 100 ) {
    //     count = 0;
    //     printf("radain: %f, %f\n", _velocity, _ref_Iq);
    // }
    setTargetCurrent(_ref_Iq, 0.0f);
}

/*
*  brief: position loop pid control
*         calculate _ref_vel according to input target.
*         the output range is [-86 , 86]
*  input: target velocity: [-86, 86] unit: rad/s
*/
void foc::setTargetPosition(float target)
{
    float _cur_pos = angle->getFullRadian();    // * 180 / _PI;
    float _ref_vel = position_pid->process(target, _cur_pos);
    setTargetVelocity(_ref_vel);

    // float _ref_Iq
    // setTargetCurrent(_ref_Iq, 0.0f);

    if (count++ > 100) {
        count = 0;
        printf("position: %f, %f, %f\n", _cur_pos, _ref_vel, Uq);
    }
}


void foc::setDebug(int mode, int id, float set)
{
    static pid* PID_CTL_TABLE[] = {
        /*0*/ torque_d_pid,
        /*1*/ speed_pid,
        /*2*/ position_pid,
    };

    if (mode < FOC_PID_MODE_MAX) {
        PID_CTL_TABLE[mode]->setDebug(id, set);
    }
}

/**************************************************************
*            basic foc algorithms implementation.             *
**************************************************************/

void foc::clarkPark(float elec_angle)
{
    Ia = current->getCurentA();
    Ib = current->getCurentB();

    // clark
    I_alpha = Ia;
    I_beta = _1_SQRT3 * ((Ib * 2) + Ia);
    // park
    Id = I_alpha * cosf(elec_angle) + I_beta * sinf(elec_angle);
    Iq = I_beta * cosf(elec_angle) - I_alpha * sinf(elec_angle);
    //printf("U_alpha: %f,%f\n",  measured.U_alpha, measured.U_beta);
    // low pass filter
    Id = Id_fliter->process(Id);
    Iq = Iq_fliter->process(Iq);
}

// void foc::invClark()
// {
//     Ua = U_alpha;
//     Ub = (_SQRT3 * U_beta - U_alpha) / 2;
//     Uc = -(U_alpha + _SQRT3 * U_beta) / 2;
//     // printf("Ua=%f\t Ub=%f\t Uc=%f\n", reference.Ua, reference.Ub, reference.Uc);
// }

uint8_t foc::identifySection(float A, float B, float C)
{
    constexpr uint8_t N_2_SECTION[] = {0, 2, 6, 1, 4, 3, 5};
    uint8_t a = 0, b = 0, c = 0;
    uint8_t sec = 0;

    if (A > 0) a = 1;
    if (B > 0) b = 1;
    if (C > 0) c = 1;

    uint8_t N = (c << 2) + (b << 1) + a;  // N=4C+2B+A

    if (N > 0 && N < 7) {
        sec = N_2_SECTION[N];
    }
    return sec;
}

void foc::invParkSvpwm(float ref_Uq, float ref_Ud, float elec_angle)
{
    constexpr float Ts = 1.0f;

    float Ta = 0, Tb = 0, Tc = 0;

    // invpark
    U_alpha = ref_Ud * cosf(elec_angle) - ref_Uq * sinf(elec_angle);
    U_beta = ref_Uq * cosf(elec_angle) + ref_Ud * sinf(elec_angle);

    // svpwm: section identify
    float A = U_beta;
    float B = (_SQRT3 * U_alpha - U_beta) * 0.5;
    float C = -(_SQRT3 * U_alpha + U_beta) * 0.5;

    section = identifySection(A, B, C);

    // svpwm: Ta, Tb, Tc time caculate
    float x = _SQRT3 * A / Udc;
    float y = _SQRT3 * (-C) / Udc;
    float z = _SQRT3 * (-B) / Udc;

    switch (section) {
      case 1:
        //T_fri = -z;
        //T_sec = x;
        //T_0 = (Ts - T_fri - T_sec) / 2;
        //Ta = T_fri + T_sec + T_0;
        //Tb = T_sec + T_0;
        //Tc = T_0;
        Ta = (x - z + Ts) * 0.5;
        Tb = (Ts + z + x) * 0.5;
        Tc = (Ts + z - x) * 0.5;
        break;
      case 2:
        // T_fri = z;
        // T_sec = y;
        // T_0 = (Ts - T_fri- T_sec) / 2;
        // Ta = ( T_sec + T_0);
        // Tb = (T_fri + T_sec + T_0);
        // Tc = T_0;
        Ta = (Ts - z + y) * 0.5;
        Tb = (Ts + z + y) * 0.5;
        Tc = (Ts - z - y) * 0.5;
        break;
      case 6:
        // T_fri = y;
        // T_sec = -x;
        // T_0 = (Ts - T_fri- T_sec) / 2;
        // Ta = (T_fri + T_sec + T_0);
        // Tb = T_0;
        // Tc = ( T_sec + T_0);
        Ta = (Ts + y - x) * 0.5;
        Tb = (Ts - y + x) * 0.5;
        Tc = (Ts - y - x) * 0.5;
        break;
      case 4:
        // T_fri = -x;
        // T_sec = z;
        // T_0 = (Ts - T_fri- T_sec) / 2;
        // Ta = T_0;
        // Tb = ( T_sec + T_0);
        // Tc = (T_fri + T_sec + T_0);
        Ta = (Ts + x - z) * 0.5;
        Tb = (Ts + x + z) * 0.5;
        Tc = (Ts - x + z) * 0.5;
        break;
      case 3:
        // T_fri = x;
        // T_sec = -y;
        // T_0 = (Ts - T_fri- T_sec) / 2;
        // Ta = T_0;
        // Tb = (T_fri + T_sec + T_0);
        // Tc = ( T_sec + T_0);

        Ta = (Ts - x + y) * 0.5;
        Tb = (Ts + x - y) * 0.5;
        Tc = (Ts - x - y) * 0.5;
        break;
      case 5:
        // T_fri = -y;
        // T_sec = -z;
        // T_0 = (Ts - T_fri- T_sec) / 2;
        // Ta = ( T_sec + T_0) * Udc;
        // Tb = T_0 * Udc;
        // Tc = (T_fri + T_sec + T_0) * Udc;
        Ta = (Ts + y - z) * 0.5;
        Tb = (Ts + y + z) * 0.5;
        Tc = (Ts - y - z) * 0.5;
        break;
      default:
        //printf("svpwm encounter excetion, section [%d] is not right\n", section);
        break;
    }

    motor->setPwm(Ta, Tb, Tc);
}