/************************************************************
 *  sync_motor.c
 *  角度同步控制：CAN1 主动轮 ← CAN2 从动轮
 *  示例配置：两个电机都使用 ID = 0x201（数组下标 = 1）
 ************************************************************/

#include "sync_motor.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

/********************** 电机 ID 配置（示例） ************************/

// 从动轮 = CAN2 的 ID=201 → 数组下标为 1
#define FOLLOWER_ID   1

// 主动轮 = CAN1 的 ID=201 → 数组下标也为 1
#define MASTER_ID     1


/*********************** PID 结构体 *********************************/
typedef struct {
    float kp, ki;
    float integral;
} PID_t;


/*********************** PID 参数（你可以调） ************************/
static PID_t pid_angle = {
    .kp = 0.7f,     // 位置环比例
    .ki = 0.0001f,  // 位置环积分
    .integral = 0
};


/********************** PID 公式 **************************************/
static int16_t AnglePID(PID_t *pid, float ref, float fdb)
{
    float err = ref - fdb;
    pid->integral += err;

    float out = pid->kp * err + pid->ki * pid->integral;

    if (out > 16000) out = 16000;
    if (out < -16000) out = -16000;

    return (int16_t)out;
}


/****************** 主同步函数：1ms 调用一次 ************************/
/*
 * 主动轮（CAN1）跟随从动轮（CAN2）
 * 使用 totalAngle 做角度同步 + speed 前馈更丝滑
 */
void SyncMotor_Update()
{
    // ========== 1. 获取从动轮的角度和速度 ==========
    float follower_angle = CAN2_Motors[FOLLOWER_ID].totalAngle;
    float follower_speed = CAN2_Motors[FOLLOWER_ID].speed;

    // ========== 2. 主动轮的当前角度 ==========
    float master_angle = CAN1_Motors[MASTER_ID].totalAngle;

    // ========== 3. 位置 PID（主同步控制）==========
    int16_t pid_out = AnglePID(&pid_angle, follower_angle, master_angle);

    // ========== 4. 加速度前馈（减少延迟） ==========
    int16_t feedforward = follower_speed * 2;   // 可调（推荐 1~4）

    // ========== 5. 合成输出电流 ==========
    int16_t current = pid_out + feedforward;

    if (current > 16000) current = 16000;
    if (current < -16000) current = -16000;

    // ========== 6. 写入 CAN1 电流缓冲区 ==========
    CAN1_MotorCurrents[MASTER_ID] = current;

    // ========== 7. 发送到主电调（控制 201~204）==========
    CanMotor_SendCurrent(MOTOR1_4);
}