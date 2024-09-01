//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了
//函数声明
#ifndef FOC_ALGO_H
#define FOC_ALGO_H

class LowPassFilter;
class pid;
class AngleSensor;
class CurrSense;

enum {
    MOTOR_ID_0,
    MOTOR_ID_1,
    MOTOR_NUMB,
} MOTOR_ENUM;

class bldc {
    public:
        bldc(int id, int power/*, uint8_t poles*/);
        int init();
        // uint8_t getPolePairs() {return pole_pairs; }
        int getMaxPower() {return supply_power; }
        void setPwm(float Ta, float Tb, float Tc);

    private:
        int id;
        int pole_pairs;
        int supply_power;
        int gpio_index[3];
};

class foc {
    public:
        foc(bldc *m, CurrSense* cur, AngleSensor* ang);
        ~foc() = default;

        void initAndCalibrateSensor();
        void setTargetPosition(float target);
        void setTargetVelocity(float target);
        void setTargetCurrent(float target_iq, float target_id);
        void updateSensors();
        void setMaxTorqueForPosition(float cur) { position_pid->setMaxOutput(cur); }

        void setMode(int mode);
        void setDebug(int mode, int id, float set);

        float getTorqueMaxCurrent() { return Iq_max; }

    private:
        uint8_t identifySection(float A, float B, float C);

        void clarkPark(float elec_angle);
        void invParkSvpwm(float Uq, float Ud, float elec_angle);
        void setTorque(float Uq, float Ud, float elec_angle);

    private:
        uint8_t section; // identify stator section number
        float rpm; //1rpm = 9.5493 * 1 rad/s; 2000rpm = 209.44 rad/s
        // float torque_max;

        bldc *motor;
        CurrSense *current;
        AngleSensor* angle;

        LowPassFilter *Id_fliter;
        LowPassFilter *Iq_fliter;

        pid* position_pid;
        pid* speed_pid;
        pid* torque_d_pid;
        pid* torque_q_pid;

        // caculate data
        // float Ua;
        // float Ub;
        // float Uc;
        int Udc; // supply voltage. for example: 12.6v ==> 126

        float U_alpha;
        float U_beta;
        float Ud; // expect as 0
        float Uq; // decide the torque
        float Uq_max; // 6.9v => 69

        // measure data
        float Ia;
        float Ib;
        float I_alpha;
        float I_beta;
        float Id;
        float Iq;
        float Iq_max;

};


#endif