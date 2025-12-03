#ifndef _PID_H
#define _PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float last_err;
  float integral;
  float integral_limit;
  float max_out;
} PID;

static inline void PID_init(PID* pid, float kp, float ki, float kd, float integral_limit, float max_out) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->max_out = max_out;
}

static inline void PID_reset(PID* pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0;
}

float PID_calc(PID *pid, float measure, float target);

#endif
