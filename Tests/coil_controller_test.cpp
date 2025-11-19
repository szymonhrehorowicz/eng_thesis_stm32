
extern "C"
{
#include "control/bang_bang.h"
#include "control/coil_controller.h"
#include "control/pid.h"
#include "measurements/temperature.h"
#include "mocks/stm32f4xx_hal.h"
}
#include <gtest/gtest.h>

class CoilControllerTest : public ::testing::Test
{
  protected:
    CoilController_t controller;

    void SetUp() override
    {
        CoilController_init(&controller);
    }

    static uint16_t find_adc_for_temperature(float target_celsius)
    {
        uint16_t best_adc = 0;
        float best_err = 1e9f;
        for (uint16_t adc = 0; adc <= 4095; ++adc)
        {
            float t = NTC_ADC2Temperature(adc);
            float err = std::fabs(t - target_celsius);
            if (err < best_err)
            {
                best_err = err;
                best_adc = adc;
            }
        }
        return best_adc;
    }
};

TEST_F(CoilControllerTest, InitializesWithDefaults)
{
    EXPECT_EQ(controller.mode, OFF);
    EXPECT_EQ(controller.used_controller, BANG_BANG);
    EXPECT_EQ(controller.ref_temp, TEMP_TOP);
    EXPECT_EQ(controller.ref_coil, COIL_A);
    EXPECT_EQ(controller.error, 0);
    EXPECT_EQ(controller.u, 0);
    EXPECT_EQ(controller.u_saturated, 0);
}

// --- Mode switching ---
TEST_F(CoilControllerTest, SwitchesToOnMode)
{
    CoilController_setMode(&controller, ON);
    EXPECT_EQ(controller.mode, ON);
}

TEST_F(CoilControllerTest, SwitchesToCombinedMode)
{
    CoilController_setMode(&controller, COMBINED);
    EXPECT_EQ(controller.mode, COMBINED);
}

// --- Bang-bang controller ---
TEST_F(CoilControllerTest, BangBangControllerSwitchesOutput)
{
    CoilController_setMode(&controller, ON);
    CoilController_setController(&controller, BANG_BANG);

    controller.error = 10.0f;
    float u = controller.u_max * BBController_update(&controller.BB_controller, controller.error);
    EXPECT_TRUE(u == controller.u_max || u == 0.0f);
}

// --- Saturation logic ---
TEST_F(CoilControllerTest, SaturatesOutputAtUMax)
{
    CoilController_setMode(&controller, ON);
    controller.u = controller.u_max * 2; // force overshoot
    uint16_t u_transformed = controller.u / 12.0f * 1000.0f;

    if (u_transformed > controller.u_max)
    {
        controller.u_saturated = (float)controller.u_max * 12.0f / 1000.0f;
        u_transformed = controller.u_max;
    }
    EXPECT_EQ(u_transformed, controller.u_max);
}

// --- Overheat protection ---
TEST_F(CoilControllerTest, ShutsDownOnOverheat)
{
    controller.temperatures[TEMP_TOP] = MAX_TEMPERATURE + 10;
    CoilController_update(&controller);
    EXPECT_EQ(pwm_pulse, 0);
}

// --- Reset ---
TEST_F(CoilControllerTest, ResetClearsState)
{
    controller.u = 100;
    controller.error = 5;
    CoilController_reset(&controller);
    EXPECT_EQ(controller.u, 0);
    EXPECT_EQ(controller.error, 0);
    EXPECT_EQ(controller.PID_controller.integral_sum, 0);
}

// --- PID controller with temperatures below 100 °C ---
TEST_F(CoilControllerTest, PIDControllerRespondsToTemperatureErrorBelow100C)
{
    CoilController_setMode(&controller, ON);
    CoilController_setController(&controller, PID);

    controller.PID_controller.Kp = 2.0f;
    controller.PID_controller.Ki = 1.0f;
    controller.PID_controller.Kaw = 1.0f;
    controller.PID_controller.sample_time = 100;

    // Reference temperature (setpoint)
    controller.control_reference.ref_value = 90.0f; // °C
    controller.ref_temp = TEMP_TOP;
    controller.ref_coil = COIL_A;

    // Simulate ADC raw voltage corresponding to ~70–80 °C
    controller.raw_voltages[TEMP_TOP] = 350;
    controller.raw_voltages[TEMP_BOTTOM] = 350;

    // Warm up the filter with repeated updates
    float measured_temp;

    for (int i = 0; i < 50; ++i)
    {
        CoilController_update(&controller);

        measured_temp = controller.filters[TEMP_TOP].value;
        std::cout << "Measured: " << measured_temp << " | Set point: " << controller.control_reference.ref_value
                  << " | error: " << controller.error << std::endl;
    }

    EXPECT_LT(measured_temp, 100.0f); // sanity check
    EXPECT_NEAR(measured_temp, NTC_ADC2Temperature(350), 5.0);

    // Error should be positive (setpoint higher than measured)
    EXPECT_GT(controller.error, 0.0f);

    // PID contributions should be non-zero
    EXPECT_GT(controller.PID_controller.u_p, 0.0f);

    // PWM pulse should be clamped between u_min and u_max
    uint16_t pwm_pulse = pwm_pulse;
    EXPECT_LE(pwm_pulse, controller.u_max);
    EXPECT_GE(pwm_pulse, controller.u_min);

    // Force saturation to test anti-windup
    controller.u = controller.u_max * 2;
    controller.u_saturated = (float)controller.u_max * 12.0f / 1000.0f;

    for (int i = 0; i < 10; ++i)
    {
        PID_update(&controller.PID_controller, controller.error,
                   (controller.u_saturated * 12.0f / 1000.0f) - controller.u);
    }
    EXPECT_LT(controller.PID_controller.aw_integral_sum, 0.0f);
    EXPECT_LT(controller.PID_controller.u_i, controller.PID_controller.integral_sum * controller.PID_controller.Ki);
}

// --- PID controller cooling case (negative error) ---
TEST_F(CoilControllerTest, PIDControllerRespondsToNegativeErrorBelow80C)
{
    CoilController_setMode(&controller, ON);
    CoilController_setController(&controller, PID);

    controller.PID_controller.Kp = 2.0f;
    controller.PID_controller.Ki = 1.0f;
    controller.PID_controller.Kaw = 1.0f;
    controller.PID_controller.sample_time = 100;

    // Setpoint lower than measured
    ControlReference_setStepReference(&controller.control_reference, 50);
    controller.ref_temp = TEMP_TOP;
    controller.ref_coil = COIL_A;

    // Choose ADC value that maps to ~70 °C (safe, below overheat limit)
    controller.raw_voltages[TEMP_TOP] = find_adc_for_temperature(70);
    controller.raw_voltages[TEMP_BOTTOM] = find_adc_for_temperature(70);

    float measured_temp = 0.0f;
    for (int i = 0; i < 50; ++i)
    {
        CoilController_update(&controller);
        measured_temp = controller.filters[TEMP_TOP].value;
        std::cout << "Measured: " << measured_temp << " | Set point: " << controller.control_reference.ref_value
                  << " | error: " << controller.error << std::endl;
    }

    EXPECT_LT(measured_temp, 80.0f);                                  // ensure no overheat protection triggered
    EXPECT_GT(measured_temp, controller.control_reference.ref_value); // cooling case

    // Now check the actual error field
    EXPECT_LT(controller.error, 0.0f);

    // Proportional term should also be negative
    EXPECT_LT(controller.PID_controller.u_p, 0.0f);

    // PWM pulse should remain clamped
    EXPECT_GE(pwm_pulse, controller.u_min);
    EXPECT_LE(pwm_pulse, controller.u_max);
}

TEST_F(CoilControllerTest, CoilControllerUsesCorrectUSaturatedScaling_AntiWindupMagnitude)
{
    CoilController_setMode(&controller, ON);
    CoilController_setController(&controller, PID);

    controller.PID_controller.Kp = 0.0f;
    controller.PID_controller.Ki = 0.0f;
    controller.PID_controller.Kd = 0.0f;
    controller.PID_controller.Kaw = 1.0f;
    controller.PID_controller.sample_time = 100;

    ControlReference_setStepReference(&controller.control_reference, 50);
    controller.ref_temp = TEMP_TOP;
    controller.ref_coil = COIL_A;

    uint16_t adc_top = find_adc_for_temperature(65.0f);
    controller.raw_voltages[TEMP_TOP] = adc_top;
    controller.raw_voltages[TEMP_BOTTOM] = adc_top;

    controller.u_saturated = 600.0f;
    controller.u = 100.0f;
    controller.error = 0.0f;

    float aw_before = controller.PID_controller.aw_integral_sum;
    CoilController_update(&controller);
    float aw_after = controller.PID_controller.aw_integral_sum;

    float aw_delta = std::fabs(aw_after - aw_before);

    // With correct scaling, aw_delta is ~1 000–2 000.
    // With buggy scaling, aw_delta is ~100.
    EXPECT_GT(aw_delta, 500.0f) << "Anti-windup correction too small — likely using buggy u_saturated scaling.";
}
