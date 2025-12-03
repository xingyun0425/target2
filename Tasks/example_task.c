#include "example_task.h"
#include "controller.h"
#include "motor.h"
#include "pid.h"

static PID speed_pid;
static int init_done = 0;
float speed_target = 0;
float speed_real = 0;
float speed_error = 0;
float out = 0;

void example_task(void* arg)
{
    if (!init_done)
    {
        PID_init(&speed_pid, 4.5f, 0.05f, 0.05f, 3000, 700);
        init_done = 1;
    }

    while (1)
    {
        if (!control.online)  
        {
            CAN1_MotorCurrents[3] = 0;
            CanMotor_SendCurrent(CAN1_MOTOR_1_TO_4);
            continue;
        }
        speed_target = control.channel[1] * 60.0f;  
        speed_real = CAN1_Motors[3].speed;
        speed_error = speed_target - speed_real;
        out = PID_calc(&speed_pid, speed_real, speed_target);
        

        CAN1_MotorCurrents[3] = (int16_t)out;
        CanMotor_SendCurrent(CAN1_MOTOR_1_TO_4);
        delay(1);
    }
}
