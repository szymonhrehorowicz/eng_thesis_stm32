extern "C"
{
#include "control/pid.h"
}
#include <gtest/gtest.h>

// A helper to initialize PID with anti-windup enabled
static PID_t makePID(float Kp, float Ki, float Kd, float Kaw, int sample_time_ms)
{
    PID_t pid;
    PID_reset(&pid);
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.Kaw = Kaw;
    pid.sample_time = sample_time_ms;
    return pid;
}

TEST(PIDControllerTest, AntiWindupPreventsIntegralRunaway)
{
    // Setup PID with integral and anti-windup gain
    PID_t pid = makePID(0.0f, 1.0f, 0.0f, 1.0f, 100); // Ki=1, Kaw=1, Ts=100ms

    float error = 10.0f;              // large positive error
    float control_difference = -5.0f; // simulating actuator saturation feedback

    // Run several updates to simulate controller loop
    for (int i = 0; i < 50; ++i)
    {
        float u = PID_update(&pid, error, control_difference);
        (void)u; // ignore actual output, we care about internal sums
    }

    // Without anti-windup, integral_sum would grow unbounded
    // With anti-windup, aw_integral_sum counteracts runaway
    EXPECT_NEAR(pid.aw_integral_sum, control_difference * 50 * 0.1f, 1e-3);

    // The total integral contribution should be moderated
    EXPECT_LT(fabs(pid.u_i), fabs(pid.integral_sum * pid.Ki));

    // Ensure the controller output is finite and not exploding
    float final_output = PID_update(&pid, error, control_difference);
    EXPECT_TRUE(final_output < 1000.0f);
}

TEST(PIDControllerTest, AntiWindupReducesIntegratorWhenSaturated)
{
    PID_t pid = makePID(0.0f, 2.0f, 0.0f, 2.0f, 100);

    float error = 20.0f;
    float control_difference = -20.0f; // strong negative feedback due to saturation

    // Run updates
    for (int i = 0; i < 20; ++i)
    {
        PID_update(&pid, error, control_difference);
    }

    // The aw_integral_sum should be negative, counteracting positive integral_sum
    EXPECT_LT(pid.aw_integral_sum, 0.0f);

    // The net integral term should be significantly reduced
    EXPECT_LT(pid.u_i, pid.integral_sum * pid.Ki);
}
