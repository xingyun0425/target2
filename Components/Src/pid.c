#include "pid.h"
#include <math.h>

static inline void abs_limit(float *a, float max) {
  if (*a > max)
    *a = max;
  else if (*a < -max)
    *a = -max;
}

float PID_calc(PID* pid, float measure, float target) {
  float err = target - measure;

  pid->integral += pid->ki * err;
  abs_limit(&pid->integral, pid->integral_limit);
  float out = pid->kp * err \
            + pid->integral \
            + pid->kd * (err - pid->last_err);
  abs_limit(&out, pid->max_out);

  pid->last_err = err;

  return out;
}
