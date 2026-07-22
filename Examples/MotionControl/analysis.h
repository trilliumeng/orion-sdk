#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>

// Signal types
typedef enum {
    SIGNAL_STEP,
    SIGNAL_SINE
} SignalType;

// Control modes
typedef enum {
    MODE_POSITION,
    MODE_STAB_RATE
} ControlMode;

// Test axis
typedef enum {
    AXIS_PAN,
    AXIS_TILT
} TestAxis;

// Test configuration
typedef struct TestConfig {
    ControlMode mode;
    SignalType signal_type;
    TestAxis axis;
    double offset;
    double amplitude;
    double frequency;
    int cycles;
    double cmd_rate_hz;
} TestConfig;

// Step response metrics
typedef struct ResponseMetrics {
    double rise_time_sec;
    double overshoot_pct;
    double steady_state_error;
    bool valid;
} ResponseMetrics;

// Generate command signal value at given time (degrees)
double GenerateCommandSignal(const TestConfig* config, double time_sec);

// Get total test duration in seconds
double GetTestDuration(const TestConfig* config);

// Calculate step response metrics from response/commanded arrays
void CalculateAllMetrics(const double* response_positions,
                          const double* commanded_positions,
                          int num_samples,
                          double commanded_target, double step_size, double dt,
                          ResponseMetrics* metrics);

#endif // ANALYSIS_H
