#include <libpynq.h>

#define DECISION_SM_ADDR               0x50U

#define MOTOR_DRIVER_SM_ADDR           0x51U
#define MOTOR_DRIVER_SM_AMPLITUDE_REG  0x00U
#define MOTOR_DRIVER_SM_FREQUENCY_REG  0x01U

#define HEARTBEAT_SM_ADDR              0x52U
#define HEARTBEAT_SM_HEARTRATE_REG     0x00U

#define CRYING_SM_ADDR                 0x53U
#define CRYING_SM_LEVEL_REG            0x00U

#define CRYING_MAX_VALUE_DEFAULT       50000
#define HEART_RATE_MIN                 60
#define HEART_RATE_MAX                 240
#define UNVISITED_CELL                 255
#define CONVERGENCE_SAMPLES            7
#define CONVERGENCE_SAMPLES_START      7
#define CONVERGENCE_THRESHOLD          1
#define PANIC_THRESHOLD                2
#define HEARTBEAT_SAMPLES              5
#define HEARTBEAT_TOLERANCE            10

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
    displayFillScreen(&display, RGB_BLACK);

    int y = 20;
    int line_height = 20;

    char crying_string[50];
    snprintf(crying_string, sizeof(crying_string), "crying level: %lu", (unsigned long)crying_level);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)crying_string, RGB_GREEN);

    y += line_height;
    char heart_string[50];
    snprintf(heart_string, sizeof(heart_string), "heart rate: %lu", (unsigned long)heart_rate);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)heart_string, RGB_GREEN);

    y += line_height;
    char amplitude_string[50];
    snprintf(amplitude_string, sizeof(amplitude_string), "motor amp: %lu", (unsigned long)motor_amplitude);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)amplitude_string, RGB_GREEN);

    y += line_height;
    char frequency_string[50];
    snprintf(frequency_string, sizeof(frequency_string), "motor freq: %lu", (unsigned long)motor_frequency);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)frequency_string, RGB_GREEN);

    y += line_height;
    char stress_string[50];
    snprintf(stress_string, sizeof(stress_string), "stress level: %lu", (unsigned long)stress_level);
    displayDrawString(&display, &fx, 10, y, (uint8_t *)stress_string, RGB_GREEN);
}

int validate_sensors(uint32_t heart_rate, uint32_t crying_level)
{
    if(heart_rate < HEART_RATE_MIN || heart_rate > HEART_RATE_MAX)
    {
        printf("ERROR: Invalid heart rate %d BPM (valid range: %d-%d)\n",
               heart_rate, HEART_RATE_MIN, HEART_RATE_MAX);
        return 0;
    }

    if(crying_level > CRYING_MAX_VALUE_DEFAULT * 2)
    {
        printf("ERROR: Invalid crying level %d (max: %d)\n",
               crying_level, CRYING_MAX_VALUE_DEFAULT * 2);
        return 0;
    }

    return 1;
}

uint32_t average_heartbeat(uint32_t heartbeat_history[], int history_count)
{
    if(history_count == 0)
        return 0;

    uint32_t sum = 0;
    for(int i = 0; i < history_count; i++)
    {
        sum += heartbeat_history[i];
    }

    return sum / history_count;
}

uint32_t calc_stress(uint32_t crying_level, uint32_t heart_rate, uint32_t crying_at_k5, uint32_t crying_step_size, int use_crying)
{
    uint32_t hr_stress = heart_rate * 0.5 - 20;

    uint32_t continuous_stress = hr_stress;

    if(use_crying && crying_step_size > 0)
    {
        uint32_t expected_crying_value;

        if(hr_stress >= 50)
        {
            expected_crying_value = crying_at_k5;
        }
        else if(hr_stress >= 40)
        {
            expected_crying_value = crying_at_k5 - crying_step_size;
        }
        else if(hr_stress >= 30)
        {
            expected_crying_value = crying_at_k5 - (crying_step_size * 2);
        }
        else if(hr_stress >= 20)
        {
            expected_crying_value = crying_at_k5 - (crying_step_size * 3);
        }
        else
        {
            expected_crying_value = crying_at_k5 - (crying_step_size * 4);
        }

        if(crying_level > expected_crying_value + (crying_step_size / 2))
        {
            continuous_stress += 10;
        }
    }

    if(continuous_stress < 10) continuous_stress = 10;
    if(continuous_stress > 100) continuous_stress = 100;

    uint32_t rounded_stress = continuous_stress + 5;
    uint32_t discrete_stress = ((rounded_stress - 10) / 10) + 1;
    if(discrete_stress > 9) discrete_stress = 9;
    if(discrete_stress < 1) discrete_stress = 1;

    return discrete_stress;
}

around check_around(uint32_t stress_matrix[5][5], uint32_t motor_amplitude, uint32_t motor_frequency)
{
    around around_result;

    if(motor_frequency <= 1)
        around_result.l = 1000;
    else
        around_result.l = stress_matrix[motor_frequency-2][motor_amplitude-1];

    if(motor_frequency >= 5)
        around_result.r = 1000;
    else
        around_result.r = stress_matrix[motor_frequency][motor_amplitude-1];

    if(motor_amplitude <= 1)
        around_result.u = 1000;
    else
        around_result.u = stress_matrix[motor_frequency-1][motor_amplitude-2];

    if(motor_amplitude >= 5)
        around_result.d = 1000;
    else
        around_result.d = stress_matrix[motor_frequency-1][motor_amplitude];

    return around_result;
}

pos calc_next_pos(uint32_t stress_matrix[5][5], uint32_t stress_level, uint32_t motor_amplitude, uint32_t motor_frequency, uint8_t failed_moves[5][5])
{
    pos next_pos;
    next_pos.freq = motor_frequency;
    next_pos.amp = motor_amplitude;

    if(motor_frequency == 1 && motor_amplitude == 1)
    {
        printf("=== GOAL REACHED! Resetting to (F5, A5) for next iteration ===\n");
        next_pos.freq = 5;
        next_pos.amp = 5;
        return next_pos;
    }

    around neighbors = check_around(stress_matrix, motor_amplitude, motor_frequency);

    uint8_t failed = failed_moves[motor_frequency-1][motor_amplitude-1];

    uint32_t min_stress = stress_level;
    int found_better = 0;

    if(neighbors.l != 1000 && neighbors.l != UNVISITED_CELL && neighbors.l < min_stress &&
       !(failed & 0x01))
    {
        min_stress = neighbors.l;
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        found_better = 1;
    }

    if(neighbors.r != 1000 && neighbors.r != UNVISITED_CELL && neighbors.r < min_stress &&
       !(failed & 0x04))
    {
        min_stress = neighbors.r;
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        found_better = 1;
    }

    if(neighbors.u != 1000 && neighbors.u != UNVISITED_CELL && neighbors.u < min_stress &&
       !(failed & 0x02))
    {
        min_stress = neighbors.u;
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        found_better = 1;
    }

    if(neighbors.d != 1000 && neighbors.d != UNVISITED_CELL && neighbors.d < min_stress &&
       !(failed & 0x08))
    {
        min_stress = neighbors.d;
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude + 1;
        found_better = 1;
    }

    if(found_better)
    {
        return next_pos;
    }

    if(motor_frequency > 1 && neighbors.l != 1000 && neighbors.l == UNVISITED_CELL &&
       !(failed & 0x01))
    {
        next_pos.freq = motor_frequency - 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(motor_amplitude > 1 && neighbors.u != 1000 && neighbors.u == UNVISITED_CELL &&
       !(failed & 0x02))
    {
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude - 1;
        return next_pos;
    }

    if(neighbors.r != 1000 && neighbors.r == UNVISITED_CELL &&
       !(failed & 0x04))
    {
        next_pos.freq = motor_frequency + 1;
        next_pos.amp = motor_amplitude;
        return next_pos;
    }

    if(neighbors.d != 1000 && neighbors.d == UNVISITED_CELL &&
       !(failed & 0x08))
    {
        next_pos.freq = motor_frequency;
        next_pos.amp = motor_amplitude + 1;
        return next_pos;
    }

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

    printf("INFO: No better neighbor found from (F%d, A%d). Staying at current position.\n",
           motor_frequency, motor_amplitude);
    return next_pos;
}

int check_convergence(uint32_t stress_history[], int history_count, uint32_t motor_frequency, uint32_t motor_amplitude)
{
    int required_samples = CONVERGENCE_SAMPLES;
    if(motor_frequency == 5 && motor_amplitude == 5)
    {
        required_samples = CONVERGENCE_SAMPLES_START;
    }

    if(history_count < required_samples)
        return 0;

    uint32_t min_stress = stress_history[0];
    uint32_t max_stress = stress_history[0];

    for(int i = 1; i < required_samples && i < history_count; i++)
    {
        if(stress_history[i] < min_stress) min_stress = stress_history[i];
        if(stress_history[i] > max_stress) max_stress = stress_history[i];
    }

    uint32_t variation = max_stress - min_stress;
    return (variation <= CONVERGENCE_THRESHOLD);
}

int detect_panic(uint32_t current_stress, uint32_t previous_stress)
{
    if(current_stress > previous_stress + 1)
    {
        printf("WARNING: Panic detected! Stress jumped from state %d to %d\n",
               previous_stress, current_stress);
        return 1;
    }
    return 0;
}

int main(void) {
    pynq_init();
    switchbox_init();
    iic_init(IIC0);

    InitFontx(&fx, "../../fonts/ILGH24XB.FNT", "");
	display_init(&display);

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    uint32_t crying_level    = 0;
    uint32_t heart_rate      = 80;
    uint32_t motor_amplitude = 5;
    uint32_t motor_frequency = 5;
    uint32_t stress_level = 9;
    uint32_t previous_stress = 9;

    uint32_t stress_history[CONVERGENCE_SAMPLES_START] = {0};
    int history_index = 0;
    int samples_at_position = 0;
    int converged = 0;

    uint32_t prev_motor_amplitude = 5;
    uint32_t prev_motor_frequency = 5;
    uint32_t prev_stress_level = 9;
    int should_backtrack = 0;

    uint8_t failed_moves[5][5] = {0};

    int goal_reached_count = 0;
    int just_left_goal = 0;

    int test_mode = 0;
    int use_crying_for_stress = 1;
    int skip_convergence_at_goal = 1;
    int use_crying_accel = 1;

    pos path_history[25];
    int path_length = 0;
    int replaying_path = 0;
    int replay_index = 0;
    int waiting_at_start_after_panic = 0;

    uint32_t crying_samples[CONVERGENCE_SAMPLES_START] = {0};
    int crying_sample_index = 0;
    uint32_t crying_at_k5 = 0;
    uint32_t crying_at_k4 = 0;
    uint32_t crying_step_size = 0;
    int crying_step_calibrated = 0;
    int collecting_k4_samples = 0;

    uint32_t stress_matrix[5][5];

    if(test_mode)
    {
        uint32_t test_matrix[5][5] = {
            {   1,  5,  6,  9,  9 },
            {   2,  4,  5,  8,  9 },
            {   3,  4,  5,  7,  8 },
            {   7,  6,  6,  7,  8 },
            {   9,  9,  9,  9,  9 }
        };
        for(int i = 0; i < 5; i++)
            for(int j = 0; j < 5; j++)
                stress_matrix[i][j] = test_matrix[i][j];
    }
    else
    {
        for(int i = 0; i < 5; i++)
            for(int j = 0; j < 5; j++)
                stress_matrix[i][j] = UNVISITED_CELL;
    }

    displayFillScreen(&display, RGB_BLACK);

    printf("=== Rock-Your-Baby Algorithm Starting ===\n");
    printf("Test mode: %s\n", test_mode ? "ENABLED" : "DISABLED");
    printf("Crying for stress: %s\n", use_crying_for_stress ? "ENABLED" : "DISABLED");
    printf("Skip convergence at goal: %s\n", skip_convergence_at_goal ? "ENABLED" : "DISABLED");
    printf("\n");

    for (;;) {
        iic_read_register(IIC0, CRYING_SM_ADDR, CRYING_SM_LEVEL_REG,
                          (void*)&crying_level, 4);
        sleep_msec(1);

        uint32_t raw_heart_rate = 0;
        iic_read_register(IIC0, HEARTBEAT_SM_ADDR, HEARTBEAT_SM_HEARTRATE_REG,
                          (void*)&raw_heart_rate, 4);

        iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_AMPLITUDE_REG,
                           (void*)&motor_amplitude, 4);
        sleep_msec(1);
        iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_FREQUENCY_REG,
                           (void*)&motor_frequency, 4);

        if(test_mode)
        {
            stress_level = stress_matrix[motor_frequency-1][motor_amplitude-1];
            heart_rate = 80;
        }
        else
        {
            heart_rate = raw_heart_rate;

            if(!validate_sensors(heart_rate, crying_level))
            {
                printf("WARNING: Skipping iteration due to invalid sensor data\n");
                sleep_msec(100);
                continue;
            }

            if(stress_level == 5 && crying_at_k5 == 0 && use_crying_for_stress)
            {
                crying_samples[crying_sample_index] = crying_level;
                crying_sample_index++;
            }
            else if(stress_level == 4 && crying_at_k5 > 0 && !crying_step_calibrated && use_crying_for_stress)
            {
                if(!collecting_k4_samples)
                {
                    collecting_k4_samples = 1;
                    crying_sample_index = 0;
                    printf("INFO: Starting to collect crying samples at stress state 4 for step calibration.\n");
                }
                crying_samples[crying_sample_index] = crying_level;
                crying_sample_index++;
            }

            previous_stress = stress_level;
            stress_level = calc_stress(crying_level, heart_rate, crying_at_k5, crying_step_size, use_crying_for_stress);

            if(motor_frequency == 5 && motor_amplitude == 5)
            {
                stress_level = 9;
            }

            printf("Measured stress at (F%d, A%d): HR=%d BPM, Cry=%d, Stress State=%d/9\n",
                   motor_frequency, motor_amplitude, heart_rate, crying_level, stress_level);

            stress_history[history_index] = stress_level;
            history_index = (history_index + 1) % CONVERGENCE_SAMPLES_START;
            samples_at_position++;

            if(detect_panic(stress_level, previous_stress))
            {
                printf("Panic detected! Restarting to (5,5) and waiting for baby to calm down.\n");

                if(path_length > 0)
                {
                    uint8_t failed_direction = 0;
                    if(motor_frequency < prev_motor_frequency)
                        failed_direction = 0x01;
                    else if(motor_amplitude < prev_motor_amplitude)
                        failed_direction = 0x02;
                    else if(motor_frequency > prev_motor_frequency)
                        failed_direction = 0x04;
                    else if(motor_amplitude > prev_motor_amplitude)
                        failed_direction = 0x08;

                    failed_moves[prev_motor_frequency-1][prev_motor_amplitude-1] |= failed_direction;

                    path_length--;
                }

                motor_frequency = 5;
                motor_amplitude = 5;
                stress_level = 9;
                previous_stress = 9;
                samples_at_position = 0;
                history_index = 0;
                converged = 0;
                for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                    stress_history[i] = 0;

                crying_sample_index = 0;
                crying_step_calibrated = 0;
                collecting_k4_samples = 0;
                for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                    crying_samples[i] = 0;

                waiting_at_start_after_panic = 1;
                replaying_path = 0;
                replay_index = 0;

                printf("Path length before panic: %d positions. Waiting for convergence at (5,5) before replaying.\n", path_length);
                continue;
            }
            else
            {
                converged = check_convergence(stress_history, samples_at_position, motor_frequency, motor_amplitude);
                if(converged)
                {
                    printf("Stress converged at State=%d/9. Recording and moving to next position.\n", stress_level);

                    if(waiting_at_start_after_panic)
                    {
                        waiting_at_start_after_panic = 0;
                        replaying_path = 1;
                        replay_index = 0;
                        printf("Baby calmed at (5,5). Starting path replay to last good position.\n");
                    }

                    if(stress_level == 5 && crying_at_k5 == 0 && use_crying_for_stress)
                    {
                        if(crying_sample_index >= 2)
                        {
                            uint32_t sum = crying_samples[crying_sample_index - 2] + crying_samples[crying_sample_index - 1];
                            crying_at_k5 = sum / 2;
                            crying_sample_index = 0;
                            printf("CALIBRATION: Crying at K5 (stress state 5) = %d\n", crying_at_k5);
                        }
                    }
                    else if(stress_level == 4 && crying_at_k5 > 0 && !crying_step_calibrated && use_crying_for_stress)
                    {
                        if(crying_sample_index >= 2)
                        {
                            uint32_t sum = crying_samples[crying_sample_index - 2] + crying_samples[crying_sample_index - 1];
                            crying_at_k4 = sum / 2;
                            if(crying_at_k5 > crying_at_k4)
                            {
                                crying_step_size = crying_at_k5 - crying_at_k4;
                            }
                            else
                            {
                                crying_step_size = 1;
                            }
                            crying_step_calibrated = 1;
                            printf("CALIBRATION: Crying at K4 (stress state 4) = %d\n", crying_at_k4);
                            printf("CALIBRATION: Crying step size per stress state = %d\n", crying_step_size);
                            printf("CALIBRATION: Expected crying values:\n");
                            printf("  K5 (50-60%%): %d\n", crying_at_k5);
                            printf("  K4 (40-50%%): %d\n", crying_at_k5 - crying_step_size);
                            printf("  K3 (30-40%%): %d\n", crying_at_k5 - (crying_step_size * 2));
                            printf("  K2 (20-30%%): %d\n", crying_at_k5 - (crying_step_size * 3));
                            printf("  K1 (10-20%%): %d\n", crying_at_k5 - (crying_step_size * 4));
                        }
                    }

                    if((motor_frequency != 5 || motor_amplitude != 5) &&
                       (motor_frequency != prev_motor_frequency || motor_amplitude != prev_motor_amplitude))
                    {
                        uint8_t failed_direction = 0;
                        if(motor_frequency < prev_motor_frequency)
                            failed_direction = 0x01;
                        else if(motor_amplitude < prev_motor_amplitude)
                            failed_direction = 0x02;
                        else if(motor_frequency > prev_motor_frequency)
                            failed_direction = 0x04;
                        else if(motor_amplitude > prev_motor_amplitude)
                            failed_direction = 0x08;

                        if(stress_level > prev_stress_level)
                        {
                            printf("WARNING: Stress increased (was %d, now %d). Backtracking to (F%d, A%d).\n",
                                   prev_stress_level, stress_level, prev_motor_frequency, prev_motor_amplitude);
                            should_backtrack = 1;
                            failed_moves[prev_motor_frequency-1][prev_motor_amplitude-1] |= failed_direction;
                        }
                        else if(stress_level == prev_stress_level)
                        {
                            printf("INFO: Stress unchanged (stayed at %d). Returning to (F%d, A%d) then moving to next best position.\n",
                                   stress_level, prev_motor_frequency, prev_motor_amplitude);

                            failed_moves[prev_motor_frequency-1][prev_motor_amplitude-1] |= failed_direction;

                            motor_frequency = prev_motor_frequency;
                            motor_amplitude = prev_motor_amplitude;

                            iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_AMPLITUDE_REG,
                                             (void*)&motor_amplitude, 4);
                            sleep_msec(1);
                            iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_FREQUENCY_REG,
                                             (void*)&motor_frequency, 4);

                            printf("SKIP: Returned to (F%d, A%d). Waiting 1 second for motors to settle...\n",
                                   motor_frequency, motor_amplitude);

                            write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
                            sleep_msec(1000);

                            pos skip_pos = calc_next_pos(stress_matrix, prev_stress_level, prev_motor_amplitude, prev_motor_frequency, failed_moves);

                            motor_frequency = skip_pos.freq;
                            motor_amplitude = skip_pos.amp;
                            samples_at_position = 0;
                            history_index = 0;
                            converged = 0;
                            for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                                stress_history[i] = 0;

                            printf("SKIP: Now moving to next best position (F%d, A%d).\n",
                                   skip_pos.freq, skip_pos.amp);

                            write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
                            sleep_msec(50);
                            continue;
                        }
                    }
                }
            }

            stress_matrix[motor_frequency-1][motor_amplitude-1] = stress_level;
        }

        if((test_mode || converged || (replaying_path && motor_frequency == 5 && motor_amplitude == 5)) && !waiting_at_start_after_panic)
        {
            pos next_pos;
            int is_backtracking = 0;

            if(replaying_path)
            {
                if(replay_index < path_length)
                {
                    next_pos = path_history[replay_index];
                    replay_index++;
                    printf("REPLAY %d/%d: Moving to (F%d, A%d)\n",
                           replay_index, path_length, next_pos.freq, next_pos.amp);

                    if(replay_index >= path_length)
                    {
                        replaying_path = 0;
                        printf("REPLAY COMPLETE: Reached last good position (F%d, A%d). Now trying different path.\n",
                               next_pos.freq, next_pos.amp);
                    }
                }
                else
                {
                    replaying_path = 0;
                    next_pos.freq = motor_frequency;
                    next_pos.amp = motor_amplitude;
                }
            }
            else if(should_backtrack)
            {
                next_pos.freq = prev_motor_frequency;
                next_pos.amp = prev_motor_amplitude;
                should_backtrack = 0;
                is_backtracking = 1;
                printf("BACKTRACKING: (F%d, A%d) Stress State=%d/9 | Returning to: (F%d, A%d)\n",
                       motor_frequency, motor_amplitude, stress_level,
                       next_pos.freq, next_pos.amp);
            }
            else
            {
                if(motor_frequency == 1 && motor_amplitude == 1 && !just_left_goal)
                {
                    goal_reached_count++;
                    printf("\n*** GOAL REACHED (Iteration #%d) at (F1, A1) with Stress State=%d/9 ***\n",
                           goal_reached_count, stress_level);
                    printf("*** Total positions explored in this iteration ***\n\n");
                    just_left_goal = 1;
                }

                next_pos = calc_next_pos(stress_matrix, stress_level, motor_amplitude, motor_frequency, failed_moves);

                printf("Current: (F%d, A%d) Stress State=%d/9 | Next: (F%d, A%d)\n",
                       motor_frequency, motor_amplitude, stress_level,
                       next_pos.freq, next_pos.amp);
            }

            if(next_pos.freq != motor_frequency || next_pos.amp != motor_amplitude)
            {
                if(!is_backtracking && !replaying_path)
                {
                    prev_motor_frequency = motor_frequency;
                    prev_motor_amplitude = motor_amplitude;
                    prev_stress_level = stress_level;

                    if(motor_frequency != 5 || motor_amplitude != 5)
                    {
                        if(path_length < 25)
                        {
                            path_history[path_length].freq = motor_frequency;
                            path_history[path_length].amp = motor_amplitude;
                            path_length++;
                        }
                    }
                }

                motor_frequency = next_pos.freq;
                motor_amplitude = next_pos.amp;
                samples_at_position = 0;
                history_index = 0;
                converged = 0;
                for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                    stress_history[i] = 0;

                if(next_pos.freq == 1 && next_pos.amp == 1 && prev_stress_level == 2 && skip_convergence_at_goal)
                {
                    printf("INFO: Moving to (1,1) from stress state 2. Baby is calm, skipping convergence check.\n");

                    motor_frequency = 1;
                    motor_amplitude = 1;

                    iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_AMPLITUDE_REG,
                                     (void*)&motor_amplitude, 4);
                    sleep_msec(1);
                    iic_write_register(IIC0, MOTOR_DRIVER_SM_ADDR, MOTOR_DRIVER_SM_FREQUENCY_REG,
                                     (void*)&motor_frequency, 4);

                    goal_reached_count++;
                    printf("\n*** GOAL REACHED (Iteration #%d) at (F1, A1) - Baby already calm ***\n", goal_reached_count);
                    printf("*** Staying at (1,1) for 3 seconds before starting new cycle ***\n\n");

                    write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
                    sleep_msec(3000);

                    printf("\n*** STARTING NEW CYCLE #%d ***\n\n", goal_reached_count + 1);

                    motor_frequency = 5;
                    motor_amplitude = 5;
                    stress_level = 9;
                    previous_stress = 9;
                    prev_motor_amplitude = 5;
                    prev_motor_frequency = 5;
                    prev_stress_level = 9;
                    should_backtrack = 0;

                    path_length = 0;
                    replaying_path = 0;
                    replay_index = 0;
                    waiting_at_start_after_panic = 0;

                    crying_sample_index = 0;
                    crying_at_k5 = 0;
                    crying_at_k4 = 0;
                    crying_step_size = 0;
                    crying_step_calibrated = 0;
                    collecting_k4_samples = 0;
                    for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                        crying_samples[i] = 0;

                    for(int i = 0; i < 5; i++)
                        for(int j = 0; j < 5; j++)
                            failed_moves[i][j] = 0;

                    if(!test_mode)
                    {
                        for(int i = 0; i < 5; i++)
                            for(int j = 0; j < 5; j++)
                                stress_matrix[i][j] = UNVISITED_CELL;
                    }
                }
                else if(motor_frequency == 5 && motor_amplitude == 5 && just_left_goal)
                {
                    printf("\n*** STARTING NEW CYCLE #%d ***\n\n", goal_reached_count + 1);

                    just_left_goal = 0;

                    stress_level = 9;
                    previous_stress = 9;
                    prev_motor_amplitude = 5;
                    prev_motor_frequency = 5;
                    prev_stress_level = 9;
                    should_backtrack = 0;

                    path_length = 0;
                    replaying_path = 0;
                    replay_index = 0;
                    waiting_at_start_after_panic = 0;

                    crying_sample_index = 0;
                    crying_at_k5 = 0;
                    crying_at_k4 = 0;
                    crying_step_size = 0;
                    crying_step_calibrated = 0;
                    collecting_k4_samples = 0;
                    for(int i = 0; i < CONVERGENCE_SAMPLES_START; i++)
                        crying_samples[i] = 0;

                    for(int i = 0; i < 5; i++)
                        for(int j = 0; j < 5; j++)
                            failed_moves[i][j] = 0;

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
            if(waiting_at_start_after_panic)
            {
                printf("Waiting at (5,5) for baby to calm down... (sample %d/%d)\n",
                       samples_at_position, required_samples);
            }
            else
            {
                printf("Waiting for stress to converge... (sample %d/%d)\n",
                       samples_at_position, required_samples);
            }
        }

        write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
        sleep_msec(50);
    }

    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}
