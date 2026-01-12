// Rock-Your-Baby - Algorithm Module (Decision Submodule)
// Created by domas on 12/01/26.
//
// Implements gradient descent pathfinding through 5x5 stress matrix (F1-F5, A1-A5)
// to find optimal cradle rocking parameters that minimize baby stress.
//
// Features:
// - Discrete stress states (1-9): 1=calmest, 9=most stressed
// - Heartbeat averaging (5 samples) to handle BPM fluctuations (±10 BPM tolerance)
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
#define CONVERGENCE_SAMPLES            7      // Number of samples to check for convergence
#define CONVERGENCE_SAMPLES_START      7     // Number of samples at start position (5,5)
#define CONVERGENCE_THRESHOLD          1      // Max stress change to consider converged (discrete)
#define PANIC_THRESHOLD                2      // Stress increase indicating panic region (discrete)
#define HEARTBEAT_SAMPLES              5      // Number of heartbeat samples to average
#define HEARTBEAT_TOLERANCE            10     // Heartbeat fluctuation tolerance (BPM)

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

uint32_t average_heartbeat(uint32_t heartbeat_history[], int history_count)
{
    // Calculate average of heartbeat readings to smooth out fluctuations
    if(history_count == 0)
        return 0;

    uint32_t sum = 0;
    for(int i = 0; i < history_count; i++)
    {
        sum += heartbeat_history[i];
    }

    return sum / history_count;
}

uint32_t calc_stress(uint32_t crying_level, uint32_t heart_rate)
{
    // Calculate continuous stress from heart rate (PDF Figure 1: linear 60-240 BPM → 10-100% stress)
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
    uint32_t continuous_stress = hr_stress;

    // If crying is much higher than expected, increase stress estimate
    if(crying_volume > expected_crying + 0.3)
    {
        continuous_stress += (uint32_t)((crying_volume - expected_crying) * 20.0);
    }

    // Clamp to valid range (10-100%)
    if(continuous_stress < 10) continuous_stress = 10;
    if(continuous_stress > 100) continuous_stress = 100;

    // Map continuous stress (10-100%) to discrete stress states (1-9)
    // Add ±5 offset to round to nearest 10% boundary center before mapping
    // This prevents oscillation at boundaries (e.g., 79% and 80% both map to state 8)
    //
    // Examples with rounding:
    // 75% + 5 = 80 → ((80-10)/10)+1 = 8 (stable at state 8)
    // 79% + 5 = 84 → ((84-10)/10)+1 = 8 (stable at state 8)
    // 80% + 5 = 85 → ((85-10)/10)+1 = 8 (stable at state 8)
    // 85% + 5 = 90 → ((90-10)/10)+1 = 9 (transitions to state 9)
    //
    // This creates effective ranges:
    // State 1: 10-24%  | State 2: 25-34%  | State 3: 35-44%
    // State 4: 45-54%  | State 5: 55-64%  | State 6: 65-74%
    // State 7: 75-84%  | State 8: 85-94%  | State 9: 95-100%
    uint32_t rounded_stress = continuous_stress + 5;
    uint32_t discrete_stress = ((rounded_stress - 10) / 10) + 1;
    if(discrete_stress > 9) discrete_stress = 9;
    if(discrete_stress < 1) discrete_stress = 1;

    return discrete_stress;
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

pos calc_next_pos(uint32_t stress_matrix[5][5], uint32_t stress_level, uint32_t motor_amplitude, uint32_t motor_frequency, uint8_t failed_moves[5][5])
{
    pos next_pos;
    next_pos.freq = motor_frequency;
    next_pos.amp = motor_amplitude;

    // Goal reached: reset to starting position for continuous testing
    if(motor_frequency == 1 && motor_amplitude == 1)
    {
        printf("=== GOAL REACHED! Resetting to (F5, A5) for next iteration ===\n");
        next_pos.freq = 5;
        next_pos.amp = 5;
        return next_pos;
    }

    // Get stress values of neighboring cells
    around neighbors = check_around(stress_matrix, motor_amplitude, motor_frequency);

    // Get failed directions bitmap for current position (check early)
    uint8_t failed = failed_moves[motor_frequency-1][motor_amplitude-1];

    // Strategy 1: Move to visited neighbor with lowest stress
    // BUT avoid directions marked as failed (didn't decrease stress before)
    uint32_t min_stress = stress_level;
    int found_better = 0;

    // Check left (frequency - 1)
    if(neighbors.l != 1000 && neighbors.l != UNVISITED_CELL && neighbors.l < min_stress &&
       !(failed & 0x01))  // Skip if left direction marked as failed
    {
        min_stress = neighbors.l;
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        found_better = 1;
    }

    // Check right (frequency + 1)
    if(neighbors.r != 1000 && neighbors.r != UNVISITED_CELL && neighbors.r < min_stress &&
       !(failed & 0x04))  // Skip if right direction marked as failed
    {
        min_stress = neighbors.r;
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        found_better = 1;
    }

    // Check up (amplitude - 1)
    if(neighbors.u != 1000 && neighbors.u != UNVISITED_CELL && neighbors.u < min_stress &&
       !(failed & 0x02))  // Skip if up direction marked as failed
    {
        min_stress = neighbors.u;
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        found_better = 1;
    }

    // Check down (amplitude + 1)
    if(neighbors.d != 1000 && neighbors.d != UNVISITED_CELL && neighbors.d < min_stress &&
       !(failed & 0x08))  // Skip if down direction marked as failed
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
    // But skip directions that previously failed to decrease stress
    if(motor_frequency > 1 && neighbors.l != 1000 && neighbors.l == UNVISITED_CELL &&
       !(failed & 0x01))  // Check if left direction not marked as failed
    {
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(motor_amplitude > 1 && neighbors.u != 1000 && neighbors.u == UNVISITED_CELL &&
       !(failed & 0x02))  // Check if up direction not marked as failed
    {
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        return next_pos;
    }

    // Strategy 3: Stuck - explore any unvisited neighbor (avoiding failed directions)
    if(neighbors.r != 1000 && neighbors.r == UNVISITED_CELL &&
       !(failed & 0x04))  // Check if right direction not marked as failed
    {
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(neighbors.d != 1000 && neighbors.d == UNVISITED_CELL &&
       !(failed & 0x08))  // Check if down direction not marked as failed
    {
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude + 1;
        return next_pos;
    }

    // Strategy 4: Completely stuck - try ANY unvisited neighbor, even if direction was marked failed
    // This is a last resort to escape local plateaus where all good directions have same stress
    if(motor_frequency > 1 && neighbors.l == UNVISITED_CELL)
    {
        printf("ESCAPE: Trying left despite being marked failed (stuck at plateau)\n");
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(motor_amplitude > 1 && neighbors.u == UNVISITED_CELL)
    {
        printf("ESCAPE: Trying up despite being marked failed (stuck at plateau)\n");
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        return next_pos;
    }

    if(neighbors.r != 1000 && neighbors.r == UNVISITED_CELL)
    {
        printf("ESCAPE: Trying right despite being marked failed (stuck at plateau)\n");
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(neighbors.d != 1000 && neighbors.d == UNVISITED_CELL)
    {
        printf("ESCAPE: Trying down despite being marked failed (stuck at plateau)\n");
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude + 1;
        return next_pos;
    }

    // All neighbors explored and no improvement - stay at current position (Sopt reached)
    // This could mean we found the optimum, OR we're truly stuck with no unvisited neighbors
    printf("INFO: No better neighbor found from (F%d, A%d). Staying at current position.\n",
           motor_frequency, motor_amplitude);
    return next_pos;
}

int check_convergence(uint32_t stress_history[], int history_count, uint32_t motor_frequency, uint32_t motor_amplitude)
{
    // At starting position (F5, A5), require more samples for convergence
    int required_samples = CONVERGENCE_SAMPLES;
    if(motor_frequency == 5 && motor_amplitude == 5)
    {
        required_samples = CONVERGENCE_SAMPLES_START;
    }

    // Need at least required_samples to check
    if(history_count < required_samples)
        return 0;  // Not enough data yet

    // Check if stress has stabilized (variation < threshold)
    uint32_t min_stress = stress_history[0];
    uint32_t max_stress = stress_history[0];

    for(int i = 1; i < required_samples && i < history_count; i++)
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
    // With discrete stress (1-9), threshold is now 2 states instead of 10%
    if(current_stress > previous_stress + PANIC_THRESHOLD)
    {
        printf("WARNING: Panic detected! Stress jumped from state %d to %d\n",
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
    uint32_t stress_level = 9;  // Start with max discrete stress
    uint32_t previous_stress = 9;

    // Heartbeat averaging (to handle fluctuations up to 10 BPM)
    //uint32_t heartbeat_history[HEARTBEAT_SAMPLES] = {0};
    //int heartbeat_index = 0;
    //int heartbeat_samples_count = 0;

    // Convergence tracking (per PDF Section 3.1: stress converges "in one or more steps")
    // Use larger array to accommodate extended convergence at start position (5,5)
    uint32_t stress_history[CONVERGENCE_SAMPLES_START] = {0};
    int history_index = 0;
    int samples_at_position = 0;
    int converged = 0;

    // Backtracking: track previous position to return to if stress doesn't decrease
    uint32_t prev_motor_amplitude = 5;
    uint32_t prev_motor_frequency = 5;
    uint32_t prev_stress_level = 9;
    int should_backtrack = 0;

    // Failed directions matrix: tracks which neighbors from each position didn't decrease stress
    // failed_moves[freq-1][amp-1] is a bitmask: bit 0=left, bit 1=up, bit 2=right, bit 3=down
    uint8_t failed_moves[5][5] = {0};

    // Goal reached counter for continuous testing
    int goal_reached_count = 0;
    int just_left_goal = 0;  // Flag to track if we just left (1,1) - only reset when true

    // TEST MODE: Set to 1 to test pathfinding with hardcoded matrix
    // Set to 0 for real operation with sensors
    int test_mode = 0;

    // Initialize stress matrix
    // Matrix layout: stress_matrix[frequency-1][amplitude-1]
    uint32_t stress_matrix[5][5];

    if(test_mode)
    {
        // TEST MODE: Hardcoded example with discrete stress values (1-9)
        // Discrete stress mapping: 1=calmest, 9=most stressed
        // Expected path: (F5,A5)->(F4,A5)->(F4,A4)->(F4,A3)->(F3,A3)->(F3,A2)->(F3,A1)->(F2,A1)->(F1,A1)
        uint32_t test_matrix[5][5] = {
            // F1  F2  F3  F4  F5
            {   1,  5,  6,  9,  9 },  // A1: Stress 1, 5, 6, 9, 9
            {   2,  4,  5,  8,  9 },  // A2: Stress 2, 4, 5, 8, 9
            {   3,  4,  5,  7,  8 },  // A3: Stress 3, 4, 5, 7, 8
            {   7,  6,  6,  7,  8 },  // A4: Stress 7, 6, 6, 7, 8
            {   9,  9,  9,  9,  9 }   // A5: Stress 9, 9, 9, 9, 9
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

        uint32_t raw_heart_rate = 0;
        iic_read_register(IIC0, HEARTBEAT_SM_ADDR, HEARTBEAT_SM_HEARTRATE_REG,
                          (void*)&raw_heart_rate, 4);

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
            heart_rate = 80;  // Dummy value for display
        }
        else
        {
            // REAL MODE: Use raw heart rate directly (averaging disabled)
            // COMMENTED OUT: Heartbeat averaging buffer
            // heartbeat_history[heartbeat_index] = raw_heart_rate;
            // heartbeat_index = (heartbeat_index + 1) % HEARTBEAT_SAMPLES;
            // if(heartbeat_samples_count < HEARTBEAT_SAMPLES)
            //     heartbeat_samples_count++;
            // heart_rate = average_heartbeat(heartbeat_history, heartbeat_samples_count);

            // Use raw heart rate directly without averaging
            heart_rate = raw_heart_rate;

            // Validate sensors and calculate stress
            if(!validate_sensors(heart_rate, crying_level))
            {
                printf("WARNING: Skipping iteration due to invalid sensor data\n");
                sleep_msec(100);
                continue;  // Skip this iteration
            }

            // Calculate discrete stress from sensors (1-9)
            previous_stress = stress_level;
            stress_level = calc_stress(crying_level, heart_rate);

            // SPECIAL CASE: Starting position (F5, A5) always has stress level 9
            // This position is the most aggressive setting and always causes maximum stress
            // Also requires extended convergence time (20 samples instead of 5)
            if(motor_frequency == 5 && motor_amplitude == 5)
            {
                stress_level = 9;
            }

            printf("Measured stress at (F%d, A%d): HR=%d BPM, Cry=%d, Stress State=%d/9\n",
                   motor_frequency, motor_amplitude, heart_rate, crying_level, stress_level);

            // Add to history for convergence checking
            stress_history[history_index] = stress_level;
            history_index = (history_index + 1) % CONVERGENCE_SAMPLES_START;
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
                converged = check_convergence(stress_history, samples_at_position, motor_frequency, motor_amplitude);
                if(converged)
                {
                    printf("Stress converged at State=%d/9. Recording and moving to next position.\n", stress_level);

                    // Check if we should backtrack: if stress didn't decrease from previous position
                    // (Skip this check at starting position 5,5)
                    if((motor_frequency != 5 || motor_amplitude != 5) &&
                       (motor_frequency != prev_motor_frequency || motor_amplitude != prev_motor_amplitude))
                    {
                        // Mark this direction as failed in the previous position
                        // Determine which direction was taken from prev to current
                        uint8_t failed_direction = 0;
                        if(motor_frequency < prev_motor_frequency)  // Moved left
                            failed_direction = 0x01;
                        else if(motor_amplitude < prev_motor_amplitude)  // Moved up
                            failed_direction = 0x02;
                        else if(motor_frequency > prev_motor_frequency)  // Moved right
                            failed_direction = 0x04;
                        else if(motor_amplitude > prev_motor_amplitude)  // Moved down
                            failed_direction = 0x08;

                        if(stress_level > prev_stress_level)
                        {
                            // Stress INCREASED (but not panic) - backtrack to previous position
                            printf("WARNING: Stress increased (was %d, now %d). Backtracking to (F%d, A%d).\n",
                                   prev_stress_level, stress_level, prev_motor_frequency, prev_motor_amplitude);
                            should_backtrack = 1;
                            failed_moves[prev_motor_frequency-1][prev_motor_amplitude-1] |= failed_direction;
                        }
                        else if(stress_level == prev_stress_level)
                        {
                            // Stress STAYED SAME - return to previous position, wait 1 second, then move to next
                            // This avoids diagonal movements and ensures physical motors move sequentially
                            printf("INFO: Stress unchanged (stayed at %d). Returning to (F%d, A%d) then moving to next best position.\n",
                                   stress_level, prev_motor_frequency, prev_motor_amplitude);

                            // Mark direction as failed
                            failed_moves[prev_motor_frequency-1][prev_motor_amplitude-1] |= failed_direction;

                            // Step 1: Move motors back to previous position
                            motor_frequency = prev_motor_frequency;
                            motor_amplitude = prev_motor_amplitude;

                            // Write motor positions to hardware
                            iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_AMPLITUDE_REG,
                                             (void*)&motor_amplitude, 4);
                            sleep_msec(1);
                            iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_FREQUENCY_REG,
                                             (void*)&motor_frequency, 4);

                            printf("SKIP: Returned to (F%d, A%d). Waiting 1 second for motors to settle...\n",
                                   motor_frequency, motor_amplitude);

                            // Step 2: Wait 1 second for motors to physically return to previous position
                            write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
                            sleep_msec(1000);  // 1 second wait

                            // Step 3: Calculate next position from previous position (avoiding failed direction)
                            pos skip_pos = calc_next_pos(stress_matrix, prev_stress_level, prev_motor_amplitude, prev_motor_frequency, failed_moves);

                            // Step 4: Move to the new position
                            motor_frequency = skip_pos.freq;
                            motor_amplitude = skip_pos.amp;
                            samples_at_position = 0;
                            history_index = 0;
                            converged = 0;
                            for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                                stress_history[i] = 0;

                            printf("SKIP: Now moving to next best position (F%d, A%d).\n",
                                   skip_pos.freq, skip_pos.amp);

                            // Continue to next iteration to avoid duplicate position change below
                            write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
                            sleep_msec(50);
                            continue;
                        }
                        // else: stress decreased - continue normally, no backtrack needed
                    }
                }
            }

            // Update matrix with converged stress value
            stress_matrix[motor_frequency-1][motor_amplitude-1] = stress_level;
        }

        // Only move to next position if stress has converged at current position
        if(test_mode || converged)
        {
            pos next_pos;
            int is_backtracking = 0;  // Track if this move is a backtrack

            // If we should backtrack, return to previous position
            if(should_backtrack)
            {
                next_pos.freq = prev_motor_frequency;
                next_pos.amp = prev_motor_amplitude;
                should_backtrack = 0;  // Reset flag
                is_backtracking = 1;   // Mark as backtrack move
                printf("BACKTRACKING: (F%d, A%d) Stress State=%d/9 | Returning to: (F%d, A%d)\n",
                       motor_frequency, motor_amplitude, stress_level,
                       next_pos.freq, next_pos.amp);
            }
            else
            {
                // Check if we reached the goal (1,1)
                if(motor_frequency == 1 && motor_amplitude == 1)
                {
                    goal_reached_count++;
                    printf("\n*** GOAL REACHED (Iteration #%d) at (F1, A1) with Stress State=%d/9 ***\n",
                           goal_reached_count, stress_level);
                    printf("*** Total positions explored in this iteration ***\n\n");
                    just_left_goal = 1;  // Set flag - we're about to leave (1,1)
                }

                // Calculate next position based on gradient descent
                next_pos = calc_next_pos(stress_matrix, stress_level, motor_amplitude, motor_frequency, failed_moves);

                printf("Current: (F%d, A%d) Stress State=%d/9 | Next: (F%d, A%d)\n",
                       motor_frequency, motor_amplitude, stress_level,
                       next_pos.freq, next_pos.amp);
            }

            // Reset convergence tracking for new position
            if(next_pos.freq != motor_frequency || next_pos.amp != motor_amplitude)
            {
                // CRITICAL FIX: Only update previous position if NOT backtracking
                // When backtracking, we want to keep the original previous position
                // so we don't create an infinite ping-pong loop
                if(!is_backtracking)
                {
                    // Save current position as previous before moving forward
                    prev_motor_frequency = motor_frequency;
                    prev_motor_amplitude = motor_amplitude;
                    prev_stress_level = stress_level;
                }
                // If backtracking, prev_position stays pointing to the position
                // BEFORE the failed move, not the failed position itself

                // Move to next position
                motor_frequency = next_pos.freq;
                motor_amplitude = next_pos.amp;
                samples_at_position = 0;
                history_index = 0;
                converged = 0;
                for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                    stress_history[i] = 0;

                // Check if we just moved to starting position (5,5) AFTER reaching goal
                // Only reset if we came directly from (1,1), not from backtracking
                if(motor_frequency == 5 && motor_amplitude == 5 && just_left_goal)
                {
                    printf("\n*** STARTING NEW CYCLE #%d ***\n\n", goal_reached_count + 1);

                    // Clear the flag - reset only happens once per goal completion
                    just_left_goal = 0;

                    // Reset all tracking for fresh start
                    stress_level = 9;
                    previous_stress = 9;
                    prev_motor_amplitude = 5;
                    prev_motor_frequency = 5;
                    prev_stress_level = 9;
                    should_backtrack = 0;

                    // Clear failed moves matrix
                    for(int i = 0; i < 5; i++)
                        for(int j = 0; j < 5; j++)
                            failed_moves[i][j] = 0;

                    // Clear stress matrix (except in test mode)
                    if(!test_mode)
                    {
                        for(int i = 0; i < 5; i++)
                            for(int j = 0; j < 5; j++)
                                stress_matrix[i][j] = UNVISITED_CELL;
                    }
                }
            }
        }
        else
        {
            int required_samples = (motor_frequency == 5 && motor_amplitude == 5) ?
                                   CONVERGENCE_SAMPLES_START : CONVERGENCE_SAMPLES;
            printf("Waiting for stress to converge... (sample %d/%d)\n",
                   samples_at_position, required_samples);
        }

        // Removed: Test mode break - now continuously resets to (5,5) for testing
        // The system will automatically reset when reaching (1,1)

        write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
        sleep_msec(50);
    }

    // Cleanup (only reached if test_mode breaks loop)
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}
