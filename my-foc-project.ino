//DengFOC V0.2
//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了

#include "pid.h"
#include "InlineCurrent.h"
#include "AS5600.h"
#include "foc.h"

// static foc* _foc[MOTOR_NUMB];

//==============串口接收==============
static float motor_target[MOTOR_NUMB] = {0};
static int pid_mode[MOTOR_NUMB];// = {FOC_PID_MODE_POSITION, FOC_PID_MODE_POSITION};
static int motor_id = MOTOR_ID_0;

// static bldc *motor[MOTOR_NUMB];
// static CurrSense *current[MOTOR_NUMB];
// static AngleSensor *angel[MOTOR_NUMB];

struct motor_ctl_t {
    bldc *motor;
    CurrSense *current;
    AngleSensor *angel;
    foc *mfoc;
    int pid_mode;
    int pid_target;
} motor_ctl[MOTOR_NUMB];

void serialReceiveUserCommand()
{
    // a string to hold incoming data
    static String received_chars;

    int commaPosition = 0;
    String command = "";

    float val = 0;

    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        received_chars += inChar;
        // end of user input
        if (inChar == '\n') {

            commaPosition = received_chars.indexOf('='); // cmd = value
            int size = received_chars.indexOf('\n') - 1;
            // printf("commaPosition= %d, size=%d\n", commaPosition, size);
            if(commaPosition != -1) {
                command = received_chars.substring(0, commaPosition);
                val = received_chars.substring(commaPosition + 1, size).toDouble();

                foc *_foc = motor_ctl[motor_id].mfoc;

                printf("%s = %f\n", command.c_str(), val);

                if (command == "motor") {
                    motor_id = val;
                }
                else if (command == "mode") {
                    motor_ctl[motor_id].pid_mode = val;
                }
                else if (command == "pid") {
                    _foc->setDebug(motor_ctl[motor_id].pid_mode, FOC_PID_STATE, val);
                }
                else if (command == "target") {
                    motor_target[motor_id] = val;
                }
                else if (command == "p") {
                    _foc->setDebug(motor_ctl[motor_id].pid_mode, FOC_PID_KP, val);
                }
                else if (command == "i") {
                    _foc->setDebug(motor_ctl[motor_id].pid_mode, FOC_PID_KI, val);
                }
                else if (command == "d") {
                    _foc->setDebug(motor_ctl[motor_id].pid_mode, FOC_PID_KD, val);
                }
                else if (command == "if") {
                    _foc->setDebug(motor_ctl[motor_id].pid_mode, FOC_PID_KI_FACTOR, val);
                }
            }
            // reset the command buffer
            received_chars = "";
        }
    }
}


static void create_motor_controller(int id)
{
    bldc *_motor = new bldc(id, 12);
    CurrSense *_current = new CurrSense(id);
    AngleSensor *_angel = new AngleSensor(id, 1, 7);
    foc *_foc = new foc(_motor, _current, _angel);

    _foc->alignAngle();

    motor_ctl[id].motor = _motor;
    motor_ctl[id].current = _current;
    motor_ctl[id].angel = _angel;
    motor_ctl[id].mfoc = _foc;

    pid_mode[id] = FOC_PID_MODE_POSITION;
}


void setup()
{
    Serial.begin(230400);
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);  //V4电机使能

    create_motor_controller(MOTOR_ID_0);
    // create_motor_controller(MOTOR_ID_1);
}


void loop()
{
    // unsigned long timestamp_prev = micros();
    serialReceiveUserCommand();

    for (int i = 0; i < MOTOR_NUMB; i++) {
        foc *_foc = motor_ctl[i].mfoc;
        _foc->updateSensors();

        switch (pid_mode[0]) {
            case FOC_PID_MODE_ID:
                _foc->setTargetCurrent(0, motor_target[i]);
                break;
            case FOC_PID_MODE_VELOCITY:
                _foc->setTargetVelocity(motor_target[0]);
                break;
            case FOC_PID_MODE_POSITION:
                _foc->setTargetPosition(motor_target[0]);
                // float pos = angel[1]->getFullRadian();
                // _foc[0]->setTargetPosition(pos);
                break;
        }
    }

    // unsigned long timestamp_now = micros();
    // printf("the loop consume(us): %d\n", timestamp_now - timestamp_prev);

    // single bldc motor: 460us
    // double bldc motor: 880
}
