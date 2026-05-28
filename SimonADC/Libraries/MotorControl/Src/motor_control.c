#include "motor_control.h"

static float MotorControl_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

void MotorControl_InitPid(MotorControlPid_t *controller,
                          float kp,
                          float ki,
                          float kd,
                          float output_min,
                          float output_max)
{
    if (controller == 0)
    {
        return;
    }

    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->integral = 0.0f;
    controller->previous_error = 0.0f;
    controller->output_min = output_min;
    controller->output_max = output_max;
}

float MotorControl_UpdatePid(MotorControlPid_t *controller,
                             float setpoint,
                             float measurement,
                             float dt_seconds)
{
    float error;
    float derivative;
    float output;

    if ((controller == 0) || (dt_seconds <= 0.0f))
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    controller->integral += error * dt_seconds;
    derivative = (error - controller->previous_error) / dt_seconds;

    output = (controller->kp * error)
           + (controller->ki * controller->integral)
           + (controller->kd * derivative);

    output = MotorControl_Clamp(output,
                                controller->output_min,
                                controller->output_max);

    controller->previous_error = error;

    return output;
}

void MotorControl_ResetPid(MotorControlPid_t *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->integral = 0.0f;
    controller->previous_error = 0.0f;
}
