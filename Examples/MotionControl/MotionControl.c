#include "OrionPublicPacket.h"
#include "OrionComm.h"
#include "analysis.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

//-----------------------------------------------------------------------------
// Test configuration -- edit these to change test behavior
//-----------------------------------------------------------------------------

#define CMD_RATE_HZ     80.0    // Command send rate (max 100 Hz)
#define AMPLITUDE_DEG    5.0    // Step amplitude in degrees
#define FREQUENCY_HZ     1.0    // Step frequency in Hz
#define CYCLES           5      // Number of cycles to run

#define TEST_MODE       MODE_POSITION   // MODE_POSITION or MODE_STAB_RATE
#define TEST_AXIS       AXIS_PAN        // AXIS_PAN or AXIS_TILT
#define TEST_SIGNAL     SIGNAL_STEP     // SIGNAL_STEP or SIGNAL_SINE

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

#define LATE_THRESHOLD_US       2000
#define SPIN_COARSE_MARGIN_US   20000
#define RTT_SANITY_MAX_US       1000000
#define SETTLE_DURATION_SEC     2.0
#define POST_TEST_DELAY_MS      100
#define CONNECTION_TIMEOUT_US   3000000
#define STAB_RATE_IMPULSE_TIME  0.25f
#define DEG_TO_RAD              (3.14159265358979323846 / 180.0)

#define MAX_CMD_RATE_HZ         100
#define MAX_TEST_DURATION_SEC   120
#define MAX_SAMPLES             (MAX_CMD_RATE_HZ * MAX_TEST_DURATION_SEC)

//-----------------------------------------------------------------------------
// Response capture
//-----------------------------------------------------------------------------

typedef struct Response {
    int64_t  rx_time_us;
    float    cmd_target[2];
    float    stab_gyro_rate[3];
    float    encoder_pos[2];
    float    encoder_rate[2];
    uint16_t cmd_sequence_num;
    uint32_t clevis_time_ms;
    uint8_t  drops_detected;
} Response;

static Response g_responses[MAX_SAMPLES];
static int64_t  g_tx_times[MAX_SAMPLES];
static int      g_resp_count = 0;
static int64_t  g_last_rx_us = 0;
static int      g_null_gyros = 0;  // set by --null-gyros; default skips the gyro-null step

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static int64_t GetTimestampUs(void);
static void    SpinWaitUntil(int64_t target_us);
static void    DrainResponses(void);
static void    NullStabGyros(void);
static void    ExportResponsesCsv(uint16_t test_start_seq, const TestConfig *config,
                                   const char *filename);
static void    CalculateAndPrintMetrics(const TestConfig *config);
static void    PrintTimingStats(int sent, double sum_err, double sum_err_sq,
                                double max_err, int late, uint16_t test_start_seq);
static void    ProcessArgs(int argc, char **argv);
static void    KillProcess(const char *pMessage, int Value);

//-----------------------------------------------------------------------------
// Platform timer
//-----------------------------------------------------------------------------

static int64_t GetTimestampUs(void)
{
#ifdef _WIN32
    static LARGE_INTEGER freq = {0};
    if (freq.QuadPart == 0)
        QueryPerformanceFrequency(&freq);
    LARGE_INTEGER count;
    QueryPerformanceCounter(&count);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
#endif
}

static void SpinWaitUntil(int64_t target_us)
{
    while (GetTimestampUs() < target_us - SPIN_COARSE_MARGIN_US)
        Sleep(1);
    while (GetTimestampUs() < target_us)
        ;
}

//-----------------------------------------------------------------------------
// Response draining -- poll OrionCommReceive until empty
//-----------------------------------------------------------------------------

static void DrainResponses(void)
{
    OrionPkt_t rx_pkt;

    while (OrionCommReceive(&rx_pkt)) {
        int64_t rx_time = GetTimestampUs();

        if (rx_pkt.ID == ORION_PKT_CMD) {
            if (rx_pkt.Length >= getOrionCmdExtendedResponseMinDataLength()) {
                OrionCmdExtendedResponse_t resp;
                if (decodeOrionCmdExtendedResponsePacket(&rx_pkt, &resp) &&
                    g_resp_count < MAX_SAMPLES) {
                    Response *r = &g_responses[g_resp_count];
                    r->rx_time_us        = rx_time;
                    r->cmd_target[0]     = resp.Target[0];
                    r->cmd_target[1]     = resp.Target[1];
                    r->stab_gyro_rate[0] = resp.StabGyroRate[0];
                    r->stab_gyro_rate[1] = resp.StabGyroRate[1];
                    r->stab_gyro_rate[2] = resp.StabGyroRate[2];
                    r->encoder_pos[0]    = resp.EncoderPos[0];
                    r->encoder_pos[1]    = resp.EncoderPos[1];
                    r->encoder_rate[0]   = resp.EncoderRate[0];
                    r->encoder_rate[1]   = resp.EncoderRate[1];
                    r->cmd_sequence_num  = resp.SequenceNum;
                    r->clevis_time_ms    = resp.ClevisTime;
                    r->drops_detected    = resp.DropsDetected;
                    g_resp_count++;
                }
            }
            g_last_rx_us = rx_time;
        }
    }
}

//-----------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------

int main(int argc, char **argv)
{
    char csv_filename[128];
    if (argc > 1) {
        const char *iface = argv[1];
        /* Strip the Win32 device-namespace prefix so a serial path like
           "\\.\COM4" produces "responses_COM4.csv" instead of a name that
           fopen rejects. */
        if (strncmp(iface, "\\\\.\\", 4) == 0)
            iface += 4;
        snprintf(csv_filename, sizeof(csv_filename), "responses_%s.csv", iface);
    } else {
        snprintf(csv_filename, sizeof(csv_filename), "responses.csv");
    }

    ProcessArgs(argc, argv);

    // Test configuration from compile-time constants
    TestConfig config;
    memset(&config, 0, sizeof(config));
    config.mode         = TEST_MODE;
    config.signal_type  = TEST_SIGNAL;
    config.axis         = TEST_AXIS;
    config.offset       = 0.0;
    config.amplitude    = AMPLITUDE_DEG;
    config.frequency    = FREQUENCY_HZ;
    config.cycles       = CYCLES;
    config.cmd_rate_hz  = CMD_RATE_HZ;

    printf("Mode:         %s\n", config.mode == MODE_POSITION ? "Position" : "Stabilized Rate");
    printf("Axis:         %s\n", config.axis == AXIS_PAN ? "Pan" : "Tilt");
    printf("Signal:       %s\n", config.signal_type == SIGNAL_STEP ? "Step" : "Sine");
    printf("Amplitude:    %.1f deg\n", config.amplitude);
    printf("Frequency:    %.2f Hz, Cycles: %d\n", config.frequency, config.cycles);
    printf("Command rate: %.0f Hz\n", config.cmd_rate_hz);
    printf("Duration:     %.2f sec\n", GetTestDuration(&config));

    // Verify firmware supports OrionCmdExtended before any gimbal commands
    printf("\nProtocol:  %s\n", getOrionPublicVersion());
    printf("Verifying firmware supports OrionCmdExtended...\n");
    {
        OrionCmdExtended_t probe = {0};
        OrionPkt_t tx_pkt, rx_pkt;
        probe.Mode = ORION_MODE_POSITION;
        encodeOrionCmdExtendedPacket(&tx_pkt, &probe);

        int elapsed_ms = 0, verified = 0;
        while (elapsed_ms < 3000) {
            if (elapsed_ms % 500 == 0)
                OrionCommSend(&tx_pkt);
            if (OrionCommReceive(&rx_pkt) && rx_pkt.ID == ORION_PKT_CMD) {
                if (rx_pkt.Length >= getOrionCmdExtendedResponseMinDataLength()) {
                    printf("  OK (%d ms)\n", elapsed_ms);
                    verified = 1;
                    break;
                } else {
                    KillProcess("  FAILED: firmware does not support OrionCmdExtended. Install tpkg 3.1+.", 1);
                }
            }
            Sleep(10);
            elapsed_ms += 10;
        }
        if (!verified)
            KillProcess("  No response (3 sec). Check power, connection, IP/COM port.", 1);
    }

    // Home gimbal
    printf("Homing gimbal to 0/0...\n");
    {
        OrionCmd_t home = {0};
        OrionPkt_t pkt;
        home.Mode = ORION_MODE_POSITION;
        encodeOrionCmdPacket(&pkt, &home);
        OrionCommSend(&pkt);
        Sleep(2000);
    }

    if (g_null_gyros)
        NullStabGyros();

    // Timing setup
    int64_t period_us = (int64_t)(1000000.0 / CMD_RATE_HZ);
    int target_idx = (config.axis == AXIS_PAN) ? 0 : 1;
    uint16_t seq = 0;
    OrionCmdExtended_t cmd;
    OrionPkt_t tx_pkt;

#ifdef _WIN32
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif

    // --- Settle: hold initial position ---
    {
        int settle_samples = (int)(SETTLE_DURATION_SEC * CMD_RATE_HZ);
        double initial_rad = GenerateCommandSignal(&config, 0.0) * DEG_TO_RAD;
        printf("\nSettling: %.1f sec...\n", SETTLE_DURATION_SEC);

        memset(&cmd, 0, sizeof(cmd));
        cmd.Target[target_idx] = (float)initial_rad;
        cmd.Mode = (config.mode == MODE_POSITION) ? ORION_MODE_POSITION : ORION_MODE_RATE;
        cmd.Stabilized = (config.mode == MODE_STAB_RATE) ? 1 : 0;
        cmd.ImpulseTime = (config.mode == MODE_STAB_RATE) ? STAB_RATE_IMPULSE_TIME : 0.0f;
        cmd.SequenceNum = seq;
        encodeOrionCmdExtendedPacket(&tx_pkt, &cmd);

        int64_t next_send = GetTimestampUs();
        for (int i = 0; i < settle_samples; i++) {
            SpinWaitUntil(next_send);
            OrionCommSend(&tx_pkt);
            if (seq < MAX_SAMPLES) g_tx_times[seq] = GetTimestampUs();
            seq++;
            DrainResponses();
            next_send += period_us;
            cmd.SequenceNum = seq;
            encodeOrionCmdExtendedPacket(&tx_pkt, &cmd);
        }
    }

    // --- Test: send command signal, capture responses ---
    {
        g_resp_count = 0;
        uint16_t test_start_seq = seq;
        double duration_sec = GetTestDuration(&config);
        int total_samples = (int)(duration_sec * CMD_RATE_HZ);

        double sum_err = 0, sum_err_sq = 0, max_err = 0;
        int late = 0, sent = 0;

        printf("Test: %.2f sec, %d samples at %.0f Hz\n", duration_sec, total_samples, CMD_RATE_HZ);

        // Pre-compute first command (signal at t=0)
        double target_deg = GenerateCommandSignal(&config, 0.0);
        memset(&cmd, 0, sizeof(cmd));
        cmd.Target[target_idx] = (float)(target_deg * DEG_TO_RAD);
        cmd.Mode = (config.mode == MODE_POSITION) ? ORION_MODE_POSITION : ORION_MODE_RATE;
        cmd.Stabilized = (config.mode == MODE_STAB_RATE) ? 1 : 0;
        cmd.ImpulseTime = (config.mode == MODE_STAB_RATE) ? STAB_RATE_IMPULSE_TIME : 0.0f;
        cmd.SequenceNum = seq;
        encodeOrionCmdExtendedPacket(&tx_pkt, &cmd);

        int64_t t0 = GetTimestampUs();
        int64_t next_send = t0;

        for (int i = 0; i < total_samples; i++) {
            // Time-critical: spin then send immediately
            SpinWaitUntil(next_send);
            OrionCommSend(&tx_pkt);
            int64_t send_us = GetTimestampUs();

            // --- Slack time ---
            if (seq < MAX_SAMPLES) g_tx_times[seq] = send_us;
            sent++;
            seq++;

            double err = (double)(send_us - next_send);
            sum_err += err;
            sum_err_sq += err * err;
            if (fabs(err) > max_err) max_err = fabs(err);
            if (err > LATE_THRESHOLD_US) late++;

            DrainResponses();

            if ((g_last_rx_us > 0) && ((send_us - g_last_rx_us) > CONNECTION_TIMEOUT_US)) {
                printf("\nERROR: Connection lost (no response for %.1f sec)\n",
                       CONNECTION_TIMEOUT_US / 1000000.0);
                break;
            }

            if ((i + 1) % 100 == 0) {
                printf("  Sent %d/%d\r", i + 1, total_samples);
                fflush(stdout);
            }

            // Pre-compute next command at scheduled time
            next_send += period_us;
            target_deg = GenerateCommandSignal(&config, (double)(next_send - t0) / 1000000.0);
            memset(&cmd, 0, sizeof(cmd));
            cmd.Target[target_idx] = (float)(target_deg * DEG_TO_RAD);
            cmd.Mode = (config.mode == MODE_POSITION) ? ORION_MODE_POSITION : ORION_MODE_RATE;
            cmd.Stabilized = (config.mode == MODE_STAB_RATE) ? 1 : 0;
            cmd.ImpulseTime = (config.mode == MODE_STAB_RATE) ? STAB_RATE_IMPULSE_TIME : 0.0f;
            cmd.SequenceNum = seq;
            encodeOrionCmdExtendedPacket(&tx_pkt, &cmd);
        }

        // Home gimbal, drain final responses
        printf("\n");
        memset(&cmd, 0, sizeof(cmd));
        cmd.Mode = ORION_MODE_POSITION;
        cmd.SequenceNum = seq;
        encodeOrionCmdExtendedPacket(&tx_pkt, &cmd);
        OrionCommSend(&tx_pkt);
        Sleep(POST_TEST_DELAY_MS);
        DrainResponses();

        printf("Test complete. Received %d responses.\n", g_resp_count);

#ifdef _WIN32
        SetPriorityClass(GetCurrentProcess(), NORMAL_PRIORITY_CLASS);
#endif

        if (g_resp_count > 0)
            ExportResponsesCsv(test_start_seq, &config, csv_filename);

        CalculateAndPrintMetrics(&config);
        PrintTimingStats(sent, sum_err, sum_err_sq, max_err, late, test_start_seq);
    }

    OrionCommClose();
    return 0;
}

//-----------------------------------------------------------------------------
// Gyro calibration
//-----------------------------------------------------------------------------

static void NullStabGyros(void)
{
    OrionCmd_t cmd;
    OrionPkt_t pkt;

    printf("Nulling stabilization gyros (keep gimbal still for 30 sec)...\n");

    memset(&cmd, 0, sizeof(cmd));
    cmd.Mode        = ORION_MODE_NULL_GYROS;
    cmd.ImpulseTime = 30.0f;
    encodeOrionCmdPacket(&pkt, &cmd);
    OrionCommSend(&pkt);

    /* Wait for the firmware NULL_GYROS state machine to complete: ~2 sec
       homing + ImpulseTime sec of nulling before Mode returns to POSITION. */
    Sleep(35000);
    printf("  Calibrating done.\n");
}

//-----------------------------------------------------------------------------
// CSV export
//-----------------------------------------------------------------------------

static void ExportResponsesCsv(uint16_t test_start_seq, const TestConfig *config,
                                const char *filename)
{
    FILE *csv = fopen(filename, "w");
    if (!csv) return;

    int pos_idx  = (config->axis == AXIS_PAN) ? 0 : 1;
    int gyro_idx = (config->axis == AXIS_PAN) ? 2 : 1;

    fprintf(csv, "sequence_num,time_sec,tx_time_us,rx_time_us,rtt_us,received,"
                 "cmd_target_deg,stab_gyro_rate_deg_s,encoder_pos_deg,"
                 "encoder_rate_deg_s,clevis_time_ms\n");

    int64_t first_tx = 0;
    if (test_start_seq < MAX_SAMPLES && g_tx_times[test_start_seq] != 0)
        first_tx = g_tx_times[test_start_seq];

    int received = 0, dropped = 0;

    for (uint16_t seq = test_start_seq; seq < MAX_SAMPLES; seq++) {
        if (g_tx_times[seq] == 0) break;

        uint16_t norm_seq = seq - test_start_seq;
        int64_t  tx_us    = g_tx_times[seq];
        double   time_sec = (double)(tx_us - first_tx) / 1000000.0;

        int     rx_found  = 0;
        int64_t rx_us = 0, rtt_us = 0;
        float   cmd_target = 0, gyro_rate = 0, enc_pos = 0, enc_rate = 0;
        uint32_t clevis_ms = 0;

        for (int i = 0; i < g_resp_count; i++) {
            if (g_responses[i].cmd_sequence_num == seq) {
                rx_found   = 1;
                rx_us      = g_responses[i].rx_time_us;
                rtt_us     = rx_us - tx_us;
                cmd_target = g_responses[i].cmd_target[pos_idx];
                gyro_rate  = g_responses[i].stab_gyro_rate[gyro_idx];
                enc_pos    = g_responses[i].encoder_pos[pos_idx];
                enc_rate   = g_responses[i].encoder_rate[pos_idx];
                clevis_ms  = g_responses[i].clevis_time_ms;
                received++;
                break;
            }
        }
        if (!rx_found) dropped++;

        fprintf(csv, "%u,%.6f,%lld,%lld,%lld,%d,%.6f,%.6f,%.6f,%.6f,%u\n",
                norm_seq, time_sec, tx_us, rx_us, rtt_us, rx_found,
                rad2degf(cmd_target), rad2degf(gyro_rate),
                rad2degf(enc_pos), rad2degf(enc_rate), clevis_ms);
    }

    fclose(csv);
    printf("Exported %d sequences to %s (%d received, %d dropped)\n",
           received + dropped, filename, received, dropped);
}

//-----------------------------------------------------------------------------
// Step response metrics
//-----------------------------------------------------------------------------

static void CalculateAndPrintMetrics(const TestConfig *config)
{
    if (g_resp_count <= 0 || config->signal_type != SIGNAL_STEP)
        return;

    int pos_idx  = (config->axis == AXIS_PAN) ? 0 : 1;
    int gyro_idx = (config->axis == AXIS_PAN) ? 2 : 1;

    double *response  = (double *)malloc(g_resp_count * sizeof(double));
    double *commanded = (double *)malloc(g_resp_count * sizeof(double));
    if (!response || !commanded) { free(response); free(commanded); return; }

    for (int i = 0; i < g_resp_count; i++) {
        if (config->mode == MODE_STAB_RATE)
            response[i] = rad2degf(g_responses[i].stab_gyro_rate[gyro_idx]);
        else
            response[i] = rad2degf(g_responses[i].encoder_pos[pos_idx]);
        commanded[i] = rad2degf(g_responses[i].cmd_target[pos_idx]);
    }

    ResponseMetrics m;
    CalculateAllMetrics(response, commanded, g_resp_count,
                          config->offset + config->amplitude,
                          2.0 * config->amplitude,
                          1.0 / config->cmd_rate_hz, &m);

    free(response);
    free(commanded);

    if (m.valid) {
        printf("\n=== Step Response ===\n");
        if (m.rise_time_sec >= 0)
            printf("  Rise time:     %.3f sec\n", m.rise_time_sec);
        else
            printf("  Rise time:     N/A (90%% threshold not reached)\n");
        printf("  Overshoot:     %.1f %%\n", m.overshoot_pct);
        printf("  Steady-state:  %.4f deg\n", m.steady_state_error);
    }
}

//-----------------------------------------------------------------------------
// Timing statistics
//-----------------------------------------------------------------------------

static void PrintTimingStats(int sent, double sum_err, double sum_err_sq,
                             double max_err, int late, uint16_t test_start_seq)
{
    double period_us = 1000000.0 / CMD_RATE_HZ;

    // Send timing
    double mean_err = (sent > 0) ? sum_err / sent : 0;
    double mean_period = period_us + mean_err;
    double variance = (sent > 0) ? (sum_err_sq / sent) - (mean_err * mean_err) : 0;
    double send_jitter = (variance > 0) ? sqrt(variance) : 0;

    // Execute jitter (ClevisTime deltas between consecutive responses)
    double exec_jitter = 0;
    if (g_resp_count > 1) {
        double expected_ms = period_us / 1000.0;
        double sum = 0, sum_sq = 0;
        for (int i = 1; i < g_resp_count; i++) {
            int32_t delta_ms = (int32_t)(g_responses[i].clevis_time_ms - g_responses[i-1].clevis_time_ms);
            double err_us = (delta_ms - expected_ms) * 1000.0;
            sum += err_us;
            sum_sq += err_us * err_us;
        }
        int n = g_resp_count - 1;
        double m = sum / n;
        double v = (sum_sq / n) - (m * m);
        exec_jitter = (v > 0) ? sqrt(v) : 0;
    }

    // Receive jitter (rx_time deltas)
    double recv_jitter = 0;
    if (g_resp_count > 1) {
        double sum = 0, sum_sq = 0;
        for (int i = 1; i < g_resp_count; i++) {
            double delta = (double)(g_responses[i].rx_time_us - g_responses[i-1].rx_time_us);
            double err = delta - period_us;
            sum += err;
            sum_sq += err * err;
        }
        int n = g_resp_count - 1;
        double m = sum / n;
        double v = (sum_sq / n) - (m * m);
        recv_jitter = (v > 0) ? sqrt(v) : 0;
    }

    // Round-trip latency (TX timestamp to RX timestamp via sequence correlation)
    double mean_rtt = 0, max_rtt = 0;
    int rtt_count = 0;
    double sum_rtt = 0;
    for (int i = 0; i < g_resp_count; i++) {
        uint16_t s = g_responses[i].cmd_sequence_num;
        if (s < MAX_SAMPLES && g_tx_times[s] != 0) {
            double rtt = (double)(g_responses[i].rx_time_us - g_tx_times[s]);
            if (rtt > 0 && rtt < RTT_SANITY_MAX_US) {
                sum_rtt += rtt;
                if (rtt > max_rtt) max_rtt = rtt;
                rtt_count++;
            }
        }
    }
    if (rtt_count > 0) mean_rtt = sum_rtt / rtt_count;

    // Uplink drops (reported by firmware)
    int uplink_drops = 0;
    for (int i = 1; i < g_resp_count; i++)
        uplink_drops += g_responses[i].drops_detected;

    // Downlink drops (sent but no response received)
    int downlink_drops = 0;
    for (uint16_t s = test_start_seq; s < MAX_SAMPLES; s++) {
        if (g_tx_times[s] == 0) break;
        int found = 0;
        for (int i = 0; i < g_resp_count; i++) {
            if (g_responses[i].cmd_sequence_num == s) { found = 1; break; }
        }
        if (!found) downlink_drops++;
    }

    // Print
    printf("\n=== Timing ===\n");
    printf("  Command rate:  %.2f ms mean period\n", mean_period / 1000.0);
    printf("  Late packets:  %d / %d (%.1f%%)\n",
           late, sent, sent > 0 ? 100.0 * late / sent : 0.0);

    printf("\nJitter (RMS):\n");
    printf("  Send:          %.2f msec\n", send_jitter / 1000.0);
    printf("  Execute:       %.2f msec\n", exec_jitter / 1000.0);
    printf("  Receive:       %.2f msec\n", recv_jitter / 1000.0);

    printf("\nRound-trip latency:\n");
    printf("  Mean:          %.1f msec\n", mean_rtt / 1000.0);
    printf("  Max:           %.1f msec\n", max_rtt / 1000.0);

    printf("\nPacket drops:\n");
    printf("  Uplink:        %d\n", uplink_drops);
    printf("  Downlink:      %d\n", downlink_drops);
    printf("  Received:      %d / %d\n", g_resp_count, sent);
}

//-----------------------------------------------------------------------------
// Argument handling
//-----------------------------------------------------------------------------

static void PrintUsage(const char *program_name)
{
    printf("MotionControl -- step response testing over Ethernet or front panel serial\n\n");
    printf("Usage:\n");
    printf("  %s <address> [--null-gyros]\n", program_name);
    printf("  %s -h | --help\n\n", program_name);
    printf("Arguments:\n");
    printf("  <address>      Gimbal IP (e.g. 169.254.87.45) or serial path\n");
    printf("                 (\\\\.\\COM4 on Windows, /dev/ttyUSB0 on Linux)\n");
    printf("  --null-gyros   Run the 30-sec gyro-null pre-step before the test.\n");
    printf("                 Default: skip.\n\n");
    printf("Test parameters (mode, axis, signal, amplitude, frequency, cycles, rate)\n");
    printf("are compile-time constants at the top of MotionControl.c. Edit and rebuild\n");
    printf("to change them.\n\n");
    printf("Current build:\n");
    printf("  Mode:      %s\n", TEST_MODE == MODE_POSITION ? "Position" : "Stabilized Rate");
    printf("  Axis:      %s\n", TEST_AXIS == AXIS_PAN ? "Pan" : "Tilt");
    printf("  Signal:    %s\n", TEST_SIGNAL == SIGNAL_STEP ? "Step" : "Sine");
    printf("  Amplitude: %.1f deg\n", (double)AMPLITUDE_DEG);
    printf("  Frequency: %.2f Hz\n", (double)FREQUENCY_HZ);
    printf("  Cycles:    %d\n", CYCLES);
    printf("  Rate:      %.0f Hz\n", (double)CMD_RATE_HZ);
}

static void ProcessArgs(int argc, char **argv)
{
    // Pull options out of argv, leaving only the gimbal address for OrionCommOpen.
    int write = 1;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            PrintUsage(argv[0]);
            exit(0);
        }
        if (strcmp(argv[i], "--null-gyros") == 0) {
            g_null_gyros = 1;
            continue;
        }
        argv[write++] = argv[i];
    }
    argc = write;

    if (argc != 2) {
        PrintUsage(argv[0]);
        exit(1);
    }

    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("Failed to connect to gimbal", 1);
}

static void KillProcess(const char *pMessage, int Value)
{
    printf("%s\n", pMessage);
    fflush(stdout);
    OrionCommClose();
    exit(Value);
}
