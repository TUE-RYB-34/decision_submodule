// Rock-Your-Baby - Algorithm Module (Decision Submodule)
// Created by domas on 12/01/26.
//
// Implements gradient descent pathfinding through 5x5 stress matrix (F1-F5, A1-A5)
// to find optimal cradle rocking parameters that minimize baby stress.
//
// Features:
// - Sensor validation for heart rate (60-240 BPM) and crying level
// - Stress calculation from heart rate and crying according to PDF specs
// - Intelligent pathfinding with unvisited cell exploration
// - Test mode with hardcoded matrix for algorithm validation
//
#include <libpynq.h>

#define DECISION_SM_ADDR               0x50U

#define MOTOR_DRIVER_SM_ADDR           0x51U
#define MOTOR_DRIVER_SM_AMPLITUDE_REG  0x00U
#define MOTOR_DRIVER_SM_FREQUENCY_REG  0x01U

#define HEARTBEAT_SM_ADDR              0x52U
#define HEARTBEAT_SM_HEARTRATE_REG     0x00U

#define CRYING_SM_ADDR                 0x53U
#define CRYING_SM_LEVEL_REG            0x00U

#define CRYING_MAX_VALUE               50000  // Maximum crying intensity value
#define HEART_RATE_MIN                 60     // Minimum valid heart rate (BPM)
#define HEART_RATE_MAX                 240    // Maximum valid heart rate (BPM)
#define UNVISITED_CELL                 255    // Marker for unvisited matrix cells
#define CONVERGENCE_SAMPLES            5      // Number of samples to check for convergence
#define CONVERGENCE_THRESHOLD          3      // Max stress change to consider converged
#define PANIC_THRESHOLD                10     // Stress increase indicating panic region

display_t display;
FontxFile fx;

typedef struct {
    uint8_t amp;
    uint8_t freq;
} pos;

typedef struct {
    uint32_t l;
    uint32_t u;
    uint32_t r;
    uint32_t d;
} around;

void write_to_screen(uint32_t crying_level, uint32_t heart_rate, uint32_t motor_amplitude, uint32_t motor_frequency, uint32_t stress_level)
{
    // Clear display
    displayFillScreen(&display, RGB_BLACK);

    int y = 20;
    int line_height = 20;

    // Crying level
    char crying_string[50];
    snprintf(crying_string, sizeof(crying_string), "crying level: %lu", (unsigned long)crying_level);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)crying_string, RGB_GREEN);

    // Heart rate
    y += line_height;
    char heart_string[50];
    snprintf(heart_string, sizeof(heart_string), "heart rate: %lu", (unsigned long)heart_rate);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)heart_string, RGB_GREEN);

    // Motor amplitude
    y += line_height;
    char amplitude_string[50];
    snprintf(amplitude_string, sizeof(amplitude_string), "motor amp: %lu", (unsigned long)motor_amplitude);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)amplitude_string, RGB_GREEN);

    // Motor frequency
    y += line_height;
    char frequency_string[50];
    snprintf(frequency_string, sizeof(frequency_string), "motor freq: %lu", (unsigned long)motor_frequency);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)frequency_string, RGB_GREEN);

    // Stress level
    y += line_height;
    char stress_string[50];
    snprintf(stress_string, sizeof(stress_string), "stress level: %lu", (unsigned long)stress_level);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)stress_string, RGB_GREEN);
}

int validate_sensors(uint32_t heart_rate, uint32_t crying_level)
{
    // Validate heart rate (60-240 BPM per PDF Figure 1)
    if(heart_rate < HEART_RATE_MIN || heart_rate > HEART_RATE_MAX)
    {
        printf("ERROR: Invalid heart rate %d BPM (valid range: %d-%d)\n",
               heart_rate, HEART_RATE_MIN, HEART_RATE_MAX);
        return 0;  // Invalid
    }

    // Validate crying level
    if(crying_level > CRYING_MAX_VALUE)
    {
        printf("ERROR: Invalid crying level %d (max: %d)\n",
               crying_level, CRYING_MAX_VALUE);
        return 0;  // Invalid
    }

    return 1;  // Valid
}

uint32_t calc_stress(uint32_t crying_level, uint32_t heart_rate, uint32_t stress_level)
{
    // Calculate stress from heart rate (PDF Figure 1: linear 60-240 BPM → 10-100% stress)
    // Formula: stress = (HR - 60) / 180 * 90 + 10 = HR * 0.5 - 20
    uint32_t hr_stress = heart_rate * 0.5 - 20;

    // Normalize crying to percentage (0.0 - 1.0)
    float crying_volume = (float)crying_level / CRYING_MAX_VALUE;

    // Calculate expected crying for this heart rate stress level (PDF Figure 2)
    float expected_crying = 0.0;
    if(hr_stress >= 50)
    {
        expected_crying = 1.0;  // 100% crying at high stress
    }
    else if(hr_stress > 10)
    {
        // Linear interpolation between 10% stress (0% cry) and 50% stress (100% cry)
        expected_crying = (hr_stress - 10.0) / 40.0;
    }

    // Use heart rate as primary indicator, but adjust if crying differs significantly
    stress_level = hr_stress;

    // If crying is much higher than expected, increase stress estimate
    if(crying_volume > expected_crying + 0.3)
    {
        stress_level += (uint32_t)((crying_volume - expected_crying) * 20.0);
    }

    // Clamp to valid range (10-100%)
    if(stress_level < 10) stress_level = 10;
    if(stress_level > 100) stress_level = 100;

    return stress_level;
}

around check_around(uint32_t stress_matrix[5][5], uint32_t motor_amplitude, uint32_t motor_frequency)
{
    around around_result;

    // Check left neighbor (frequency - 1)
    if(motor_frequency <= 1)
        around_result.l = 1000;  // Out of bounds
    else
        around_result.l = stress_matrix[motor_frequency-2][motor_amplitude-1];

    // Check right neighbor (frequency + 1)
    if(motor_frequency >= 5)
        around_result.r = 1000;  // Out of bounds
    else
        around_result.r = stress_matrix[motor_frequency][motor_amplitude-1];

    // Check up neighbor (amplitude - 1)
    if(motor_amplitude <= 1)
        around_result.u = 1000;  // Out of bounds
    else
        around_result.u = stress_matrix[motor_frequency-1][motor_amplitude-2];

    // Check down neighbor (amplitude + 1)
    if(motor_amplitude >= 5)
        around_result.d = 1000;  // Out of bounds
    else
        around_result.d = stress_matrix[motor_frequency-1][motor_amplitude];

    return around_result;
}

pos calc_next_pos(uint32_t stress_matrix[5][5], uint32_t stress_level, uint32_t motor_amplitude, uint32_t motor_frequency)
{
    pos next_pos;
    next_pos.freq = motor_frequency;
    next_pos.amp = motor_amplitude;

    // Goal reached: stay at F1, A1 (lowest stress position)
    if(motor_frequency == 1 && motor_amplitude == 1)
    {
        return next_pos;
    }

    // Get stress values of neighboring cells
    around neighbors = check_around(stress_matrix, motor_amplitude, motor_frequency);

    // Strategy 1: Move to visited neighbor with lowest stress
    uint32_t min_stress = stress_level;
    int found_better = 0;

    // Check left (frequency - 1)
    if(neighbors.l != 1000 && neighbors.l != UNVISITED_CELL && neighbors.l < min_stress)
    {
        min_stress = neighbors.l;
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        found_better = 1;
    }

    // Check right (frequency + 1)
    if(neighbors.r != 1000 && neighbors.r != UNVISITED_CELL && neighbors.r < min_stress)
    {
        min_stress = neighbors.r;
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        found_better = 1;
    }

    // Check up (amplitude - 1)
    if(neighbors.u != 1000 && neighbors.u != UNVISITED_CELL && neighbors.u < min_stress)
    {
        min_stress = neighbors.u;
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        found_better = 1;
    }

    // Check down (amplitude + 1)
    if(neighbors.d != 1000 && neighbors.d != UNVISITED_CELL && neighbors.d < min_stress)
    {
        min_stress = neighbors.d;
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude + 1;
        found_better = 1;
    }

    // If found better visited neighbor, return it
    if(found_better)
    {
        return next_pos;
    }

    // Strategy 2: No better visited neighbor - explore toward goal (F1, A1)
    // Prefer moving left (decrease frequency) or up (decrease amplitude)
    if(motor_frequency > 1 && neighbors.l != 1000 && neighbors.l == UNVISITED_CELL)
    {
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(motor_amplitude > 1 && neighbors.u != 1000 && neighbors.u == UNVISITED_CELL)
    {
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        return next_pos;
    }

    // Strategy 3: Stuck - explore any unvisited neighbor
    if(neighbors.r != 1000 && neighbors.r == UNVISITED_CELL)
    {
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(neighbors.d != 1000 && neighbors.d == UNVISITED_CELL)
    {
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude + 1;
        return next_pos;
    }

    // All neighbors explored and no improvement - stay at current position (Sopt reached)
    return next_pos;
}

int check_convergence(uint32_t stress_history[], int history_count)
{
    // Need at least CONVERGENCE_SAMPLES to check
    if(history_count < CONVERGENCE_SAMPLES)
        return 0;  // Not enough data yet

    // Check if stress has stabilized (variation < threshold)
    uint32_t min_stress = stress_history[0];
    uint32_t max_stress = stress_history[0];

    for(int i = 1; i < CONVERGENCE_SAMPLES; i++)
    {
        if(stress_history[i] < min_stress) min_stress = stress_history[i];
        if(stress_history[i] > max_stress) max_stress = stress_history[i];
    }

    uint32_t variation = max_stress - min_stress;
    return (variation <= CONVERGENCE_THRESHOLD);  // Converged if variation is small
}

int detect_panic(uint32_t current_stress, uint32_t previous_stress)
{
    // Panic if stress increased significantly (motion too aggressive or too soft)
    if(current_stress > previous_stress + PANIC_THRESHOLD)
    {
        printf("WARNING: Panic detected! Stress jumped from %d to %d\n",
               previous_stress, current_stress);
        return 1;  // Panic
    }
    return 0;  // Normal
}

int main(void) {
    // init
    pynq_init();
    switchbox_init();
    iic_init(IIC0);

    InitFontx(&fx, "../../fonts/ILGH24XB.FNT", "");
	display_init(&display);

    // pins
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    // vars
    uint32_t crying_level    = 0;
    uint32_t heart_rate      = 80;
    uint32_t motor_amplitude = 5;
    uint32_t motor_frequency = 5;
    uint32_t stress_level = 100;
    uint32_t previous_stress = 100;

    // Convergence tracking (per PDF Section 3.1: stress converges "in one or more steps")
    uint32_t stress_history[CONVERGENCE_SAMPLES] = {0};
    int history_index = 0;
    int samples_at_position = 0;
    int converged = 0;

    // TEST MODE: Set to 1 to test pathfinding with hardcoded matrix
    // Set to 0 for real operation with sensors
    int test_mode = 0;

    // Initialize stress matrix
    // Matrix layout: stress_matrix[frequency-1][amplitude-1]
    uint32_t stress_matrix[5][5];

    if(test_mode)
    {
        // TEST MODE: Hardcoded example from PDF Figure 4
        // K values: K1=10, K2=20, K3=30, K4=40, K5=50, K6=60, K7=70, K8=80, K9=90
        // Expected path: (F5,A5)->(F4,A5)->(F4,A4)->(F4,A3)->(F3,A3)->(F3,A2)->(F3,A1)->(F2,A1)->(F1,A1)
        uint32_t test_matrix[5][5] = {
            // F1   F2   F3   F4   F5
            {  10,  50,  60,  90,  90 },  // A1: K1  K5  K6  K9  K9
            {  20,  40,  50,  80,  90 },  // A2: K2  K4  K5  K8  K9
            {  30,  40,  50,  70,  80 },  // A3: K3  K4  K5  K7  K8
            {  70,  60,  60,  70,  80 },  // A4: K7  K6  K6  K7  K8
            {  90,  90,  90,  90,  90 }   // A5: K9  K9  K9  K9  K9
        };
        for(int i = 0; i < 5; i++)
            for(int j = 0; j < 5; j++)
                stress_matrix[i][j] = test_matrix[i][j];
    }
    else
    {
        // REAL MODE: Initialize all cells as unvisited
        for(int i = 0; i < 5; i++)
            for(int j = 0; j < 5; j++)
                stress_matrix[i][j] = UNVISITED_CELL;
    }

    // Initialize display
    displayFillScreen(&display, RGB_BLACK);

    // Main control loop
    for (;;) {
        // Read sensor data from I2C submodules
        iic_read_register(IIC0, CRYING_SM_ADDR, CRYING_SM_LEVEL_REG,
                          (void*)&crying_level, 4);
        sleep_msec(1);
        iic_read_register(IIC0, HEARTBEAT_SM_ADDR, HEARTBEAT_SM_HEARTRATE_REG,
                          (void*)&heart_rate, 4);

        // Write motor control parameters
        iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_AMPLITUDE_REG,
                           (void*)&motor_amplitude, 4);
        sleep_msec(1);
        iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_FREQUENCY_REG,
                           (void*)&motor_frequency, 4);

        if(test_mode)
        {
            // TEST MODE: Use hardcoded matrix values as stress
            stress_level = stress_matrix[motor_frequency-1][motor_amplitude-1];
        }
        else
        {
            // REAL MODE: Validate sensors and calculate stress
            if(!validate_sensors(heart_rate, crying_level))
            {
                printf("WARNING: Skipping iteration due to invalid sensor data\n");
                sleep_msec(100);
                continue;  // Skip this iteration
            }

            // Calculate stress from sensors
            previous_stress = stress_level;
            stress_level = calc_stress(crying_level, heart_rate, stress_level);

            printf("Measured stress at (F%d, A%d): HR=%d BPM, Cry=%d, Stress=%d%%\n",
                   motor_frequency, motor_amplitude, heart_rate, crying_level, stress_level);

            // Add to history for convergence checking
            stress_history[history_index] = stress_level;
            history_index = (history_index + 1) % CONVERGENCE_SAMPLES;
            samples_at_position++;

            // Check for panic (per PDF Section 3.1: panic jump or panic block)
            if(detect_panic(stress_level, previous_stress))
            {
                printf("Panic detected! Trying different position immediately.\n");
                converged = 1;  // Force position change
            }
            else
            {
                // Check if stress has converged to Sopt (per PDF: "converge in one or more steps")
                converged = check_convergence(stress_history, samples_at_position);
                if(converged)
                {
                    printf("Stress converged at Sopt=%d. Recording and moving to next position.\n", stress_level);
                }
            }

            // Update matrix with converged stress value
            stress_matrix[motor_frequency-1][motor_amplitude-1] = stress_level;
        }

        // Only move to next position if stress has converged at current position
        if(test_mode || converged)
        {
            // Calculate next position based on gradient descent
            pos next_pos = calc_next_pos(stress_matrix, stress_level, motor_amplitude, motor_frequency);

            printf("Current: (F%d, A%d) Stress=%d | Next: (F%d, A%d)\n",
                   motor_frequency, motor_amplitude, stress_level,
                   next_pos.freq, next_pos.amp);

            // Reset convergence tracking for new position
            if(next_pos.freq != motor_frequency || next_pos.amp != motor_amplitude)
            {
                motor_frequency = next_pos.freq;
                motor_amplitude = next_pos.amp;
                samples_at_position = 0;
                history_index = 0;
                converged = 0;
                for(int i = 0; i < CONVERGENCE_SAMPLES; i++)
                    stress_history[i] = 0;
            }
        }
        else
        {
            printf("Waiting for stress to converge... (sample %d/%d)\n",
                   samples_at_position, CONVERGENCE_SAMPLES);
        }

        // In test mode, stop when goal is reached
        if(test_mode && motor_frequency == 1 && motor_amplitude == 1)
        {
            printf("GOAL REACHED! Baby is calm at (F1, A1) with stress=%d\n", stress_level);
            sleep_msec(1000);  // Display for 1 second
            break;
        }

        write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
        sleep_msec(50);
    }

    // Cleanup (only reached if test_mode breaks loop)
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}
