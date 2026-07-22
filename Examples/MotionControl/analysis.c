#define _USE_MATH_DEFINES
#include <math.h>

#include "analysis.h"
#include <string.h>



double GenerateCommandSignal(const TestConfig* config, double time_sec)
{
    if (!config || config->frequency <= 0) {
        return config ? config->offset : 0.0;
    }

    double period = 1.0 / config->frequency;
    double phase = fmod(time_sec, period) / period;  // 0 to 1 within each cycle

    double signal_value;

    switch (config->signal_type) {
    case SIGNAL_STEP:
        // Step signal: bipolar square wave around offset (matches sine range)
        signal_value = (phase < 0.5) ? config->amplitude : -config->amplitude;
        break;

    case SIGNAL_SINE:
        // Sine signal: offset + amplitude * sin(2*pi*freq*t)
        signal_value = config->amplitude * sin(2.0 * M_PI * config->frequency * time_sec);
        break;

    default:
        signal_value = 0.0;
        break;
    }

    return config->offset + signal_value;
}

double GetTestDuration(const TestConfig* config)
{
    if (!config || config->frequency <= 0 || config->cycles <= 0) {
        return 0.0;
    }

    return (double)config->cycles / config->frequency;
}
#include <math.h>

static void calculate_rise_time(const double* response_positions, int num_samples,
                                double initial_value, double step_size, double dt,
                                double* rise_time)
{
    if (!response_positions || num_samples <= 0 || !rise_time) {
        if (rise_time) *rise_time = -1.0;
        return;
    }

    // Threshold is 90% of the step change from initial value
    double threshold = initial_value + 0.9 * step_size;

    for (int i = 0; i < num_samples; i++) {
        // Check if we've reached 90% of the step (works for both positive and negative steps)
        if (step_size > 0) {
            if (response_positions[i] >= threshold) {
                *rise_time = i * dt;
                return;
            }
        } else {
            if (response_positions[i] <= threshold) {
                *rise_time = i * dt;
                return;
            }
        }
    }

    // Never reached 90%
    *rise_time = -1.0;
}

static void calculate_sse(const double* response_positions,
                          const double* commanded_positions,
                          int num_samples, double* sse)
{
    if (!response_positions || !commanded_positions || num_samples <= 0 || !sse) {
        if (sse) *sse = 0.0;
        return;
    }

    // Detect step transitions and measure SSE in settling windows
    // A settling window is the last 20% of each constant-command region
    double sum_error = 0.0;
    int count = 0;

    // Find regions where commanded value is constant (within tolerance)
    const double cmd_tolerance = 0.01;  // Commands within 0.01 deg are "same"
    int region_start = 0;

    for (int i = 1; i <= num_samples; i++) {
        // Check if command changed or end of data
        int cmd_changed = (i == num_samples) ||
                         (fabs(commanded_positions[i] - commanded_positions[region_start]) > cmd_tolerance);

        if (cmd_changed) {
            // End of constant-command region [region_start, i)
            int region_len = i - region_start;

            if (region_len >= 5) {  // Need at least 5 samples for meaningful SSE
                // Use last 20% of this region as settling window
                int settle_start = region_start + (int)(0.8 * region_len);
                if (settle_start < region_start) settle_start = region_start;

                for (int j = settle_start; j < i; j++) {
                    sum_error += fabs(response_positions[j] - commanded_positions[j]);
                    count++;
                }
            }

            // Start new region
            if (i < num_samples) {
                region_start = i;
            }
        }
    }

    *sse = (count > 0) ? (sum_error / count) : 0.0;
}

// Calculate overshoot using per-region analysis (requires commanded_positions)
static void calculate_overshoot_per_region(const double* response_positions,
                                           const double* commanded_positions,
                                           int num_samples,
                                           double step_size,
                                           double* overshoot_pct)
{
    if (!response_positions || !commanded_positions || num_samples <= 0 || !overshoot_pct) {
        if (overshoot_pct) *overshoot_pct = 0.0;
        return;
    }

    if (fabs(step_size) < 1e-9) {
        *overshoot_pct = 0.0;
        return;
    }

    // Find regions where commanded value is constant and measure overshoot in each
    const double cmd_tolerance = 0.01;
    int region_start = 0;
    double max_overshoot_pct = 0.0;
    double prev_target = commanded_positions[0];

    for (int i = 1; i <= num_samples; i++) {
        int cmd_changed = (i == num_samples) ||
                         (fabs(commanded_positions[i] - commanded_positions[region_start]) > cmd_tolerance);

        if (cmd_changed) {
            int region_len = i - region_start;

            if (region_len >= 5) {
                double target = commanded_positions[region_start];
                // Determine step direction for this region
                double region_step = target - prev_target;

                if (fabs(region_step) > cmd_tolerance) {
                    // Find peak in this region
                    double peak = response_positions[region_start];
                    for (int j = region_start + 1; j < i; j++) {
                        if (region_step > 0) {
                            if (response_positions[j] > peak) peak = response_positions[j];
                        } else {
                            if (response_positions[j] < peak) peak = response_positions[j];
                        }
                    }

                    // Calculate overshoot
                    double overshoot;
                    if (region_step > 0) {
                        overshoot = peak - target;
                    } else {
                        overshoot = target - peak;
                    }

                    if (overshoot > 0) {
                        double pct = 100.0 * overshoot / fabs(region_step);
                        if (pct > max_overshoot_pct) {
                            max_overshoot_pct = pct;
                        }
                    }
                }

                prev_target = target;
            }

            if (i < num_samples) {
                region_start = i;
            }
        }
    }

    *overshoot_pct = max_overshoot_pct;
}

void CalculateAllMetrics(const double* response_positions,
                          const double* commanded_positions,
                          int num_samples,
                          double commanded_target, double step_size, double dt,
                          ResponseMetrics* metrics)
{
    if (!metrics) {
        return;
    }

    metrics->valid = false;

    if (!response_positions || !commanded_positions || num_samples <= 0 || dt <= 0) {
        return;
    }

    // initial_value is target minus the step
    double initial_value = commanded_target - step_size;

    calculate_rise_time(response_positions, num_samples, initial_value, step_size, dt,
                       &metrics->rise_time_sec);

    // Use per-region overshoot calculation for multi-cycle tests
    calculate_overshoot_per_region(response_positions, commanded_positions, num_samples,
                                   step_size, &metrics->overshoot_pct);

    calculate_sse(response_positions, commanded_positions, num_samples,
                 &metrics->steady_state_error);

    metrics->valid = true;
}

