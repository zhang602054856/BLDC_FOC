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

static bldc         *motor_list[MOTOR_NUMB];
static AngleSensor  *angle_list[MOTOR_NUMB];
static CurrSense    *current_list[MOTOR_NUMB];
static foc          *foc_list[MOTOR_NUMB];
static float ref_param[MOTOR_NUMB];

static int motor_id;
static int run_mode;
static float motor_target = 0.0f;
static int count = 0;

static float radian_prev = 0;
static float ratchet_target = 0;

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

            if(commaPosition != -1) {
                command = received_chars.substring(0, commaPosition);
                val = received_chars.substring(commaPosition + 1, size).toDouble();

                printf("%s = %f\n", command.c_str(), val);

                if (command == "motor") {
                    motor_id = val;
                }
                else if (command == "mode") {
                    run_mode = val;
                    motor_target = 0;
                    foc_list[MOTOR_ID_0]->setMode(run_mode);
                    foc_list[MOTOR_ID_1]->setMode(run_mode);
                }
                else if (command == "target") {
                    motor_target = val;
                }
                else if (command == "p") {

                    foc_list[motor_id]->setDebug(run_mode, FOC_PID_KP, val);
                }
                else if (command == "i") {

                    foc_list[motor_id]->setDebug(run_mode, FOC_PID_KI, val);
                }
                else if (command == "d") {

                    foc_list[motor_id]->setDebug(run_mode, FOC_PID_KD, val);
                }
                else if (command == "if") {
                    foc_list[motor_id]->setDebug(run_mode, FOC_PID_KI_FACTOR, val);
                }
            }
            // reset the command buffer
            received_chars = "";
        }
    }
}

// static AngleSensor * angle_list[2];
static void create_motor_controller(int id)
{
    bldc *_motor        = new bldc(id, 12);
    AngleSensor *_angle = new AngleSensor(id, 1, 7);
    CurrSense *_current = new CurrSense(id);
    foc *_foc           = new foc(_motor, _current, _angle);

    motor_list[id] = _motor;
    current_list[id] = _current;
    angle_list[id] = _angle;
    foc_list[id] = _foc;

    // motor_list[id] = new bldc(id, 12);
    // current_list[id] = new CurrSense(id);
    // angle_list[id] = new AngleSensor(id, 1, 7);

    // foc_list[id] = new foc(_motor, _current, _angel);

    _foc->initAndCalibrateSensor();
    _foc->updateSensors();
}

void setup()
{
    Serial.begin(230400);
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);  //V4电机使能

    create_motor_controller(MOTOR_ID_0);
    create_motor_controller(MOTOR_ID_1);

    run_mode = FOC_MODE_VEL;
    motor_id = MOTOR_ID_0;
}

void loop()
{
    // unsigned long timestamp_prev = micros();
    // float _pos0, _pos1;
    serialReceiveUserCommand();

    for (int i = 0; i < MOTOR_NUMB; i++) {
        foc_list[i]->updateSensors();
        ref_param[i] = angle_list[i]->getFullRadian();
    }

    // if (i == MOTOR_ID_0) continue;
    switch (run_mode) {
        // case FOC_PID_MODE_ID:
        //     _foc->setTargetCurrent(0, motor_target[i]);
        //     break;

        case FOC_MODE_VEL:
            for (int i = 0; i < MOTOR_NUMB; i++) {
                foc_list[i]->setTargetVelocity(motor_target);
                ref_param[i] = angle_list[i]->getVelocity();
            }
            break;

        case FOC_MODE_POS:
            for (int i = 0; i < MOTOR_NUMB; i++) {
                foc_list[i]->setTargetPosition(motor_target);
            }
            break;

        case FOC_MODE_POS_FEED:
            foc_list[MOTOR_ID_0]->setTargetPosition(ref_param[MOTOR_ID_1]);
            foc_list[MOTOR_ID_1]->setTargetPosition(ref_param[MOTOR_ID_0]);
            break;

        case FOC_MODE_POS_RATCHET: {
            float delat_angle = ref_param[MOTOR_ID_0] - radian_prev;
            if (delat_angle > _PI / 3) {
                ratchet_target += delat_angle;
                radian_prev = ref_param[MOTOR_ID_0];
            }
            else if (ref_param[MOTOR_ID_0] < radian_prev) {
                radian_prev = ref_param[MOTOR_ID_0];
            }
            foc_list[MOTOR_ID_1]->setTargetPosition(ratchet_target);
            break;
        }
        default :
            printf("unsupport run mode : %d\n", run_mode);
            break;
    }

    // if (count++ > 100) {
    //     count = 0;
    //     printf("pos: %f, %f\n", ref_param[0], ref_param[1]);
    // }
    // unsigned long timestamp_now = micros();
    // printf("the loop consume(us): %d\n", timestamp_now - timestamp_prev);

    // single bldc motor: 460us
    // double bldc motor: 880
}
