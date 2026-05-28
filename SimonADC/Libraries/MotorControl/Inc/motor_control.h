#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
} MotorControlPid_t;

void MotorControl_InitPid(MotorControlPid_t *controller,
                          float kp,
                          float ki,
                          float kd,
                          float output_min,
                          float output_max);

float MotorControl_UpdatePid(MotorControlPid_t *controller,
                             float setpoint,
                             float measurement,
                             float dt_seconds);

void MotorControl_ResetPid(MotorControlPid_t *controller);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
