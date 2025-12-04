//TODO 
// MAKE HB AND CRYING ERROR DETECTION FOR STRESS CALC
//CHECK CURVE OF CRYING ON BABY STRESS
//
// Created by marijn on 10/14/25.
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


//THIS IS HARDCODED, TODO CHANGE SO IT IS EASIER TO ADJUST/VARIABLE
#define CRYING_MAX_VALUE               50000
display_t display;
FontxFile fx;

typedef struct {
    int amp;
    int freq;
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


uint32_t calc_stress(uint32_t crying_level, uint32_t heart_rate, uint32_t stress_level) 
{
    float crying_volume = (float)crying_level/CRYING_MAX_VALUE;
    stress_level = heart_rate*0.5 - 20;
    if(stress_level >= 50) 
    {
    } else 
    {
        uint32_t stress_level_c = (uint32_t)(crying_volume*100.0 + 25.0)/2.5;
        stress_level =(uint32_t)(stress_level+stress_level_c)/2.0;
        //stress_level = stress_level_c;
    }
    return stress_level;
}

around check_around(uint32_t stress_matrix[5][5], uint32_t motor_amplitude, uint32_t motor_frequency)
{
    around around_result;
    if(((int32_t)motor_frequency-1-1)<0) //left
    {
        around_result.l = 1000;
    } else 
    {
        around_result.l = stress_matrix[motor_frequency-1-1][motor_amplitude-1];
    }
    if((motor_frequency-1+1)>5) //right
    {
        around_result.r = 1000;
    } else 
    {
        around_result.r = stress_matrix[motor_frequency-1+1][motor_amplitude-1];
    }
    if(((int32_t)motor_amplitude-1-1)<0) //up
    {
        around_result.u = 1000;
    } else 
    {
        around_result.u = stress_matrix[motor_frequency-1][motor_amplitude-1-1];
    }
    if((motor_amplitude-1+1)>4) //down
    {
        around_result.d = 1000;
    } else 
    {
        around_result.d = stress_matrix[motor_frequency-1][motor_amplitude-1+1];
    }
    return around_result;
    
}

int find(pos pos_history[100], pos target)
{
    for(int i = 0; i < 100; i++)
    {
        if(pos_history[i].freq == target.freq && pos_history[i].amp == target.amp)
        {
            return 1;
        }
    }
    return 0;
}

int find_history_index(pos pos_history[100])
{
    for(int i = 0; i < 100; i++)
    {
        if(pos_history[i].freq == 0 && pos_history[i].amp == 0)
        {
            return i;
        }
    }

    return 0;
}


pos calc_next_pos(uint32_t stress_matrix[5][5], uint32_t stress_level, uint32_t motor_amplitude, uint32_t motor_frequency, pos pos_history[100])
{
    pos next_pos;
    next_pos.amp = 0;
    next_pos.freq = 0;
    int x = motor_frequency;
    int y = motor_amplitude;
    around around_result = check_around(stress_matrix, motor_amplitude, motor_frequency);
    
    if(motor_amplitude == 1 && motor_frequency == 1) 
    {
        return (pos){motor_frequency, motor_amplitude};
    } else 
    {
        //left
        if(around_result.l != 1000)
        {
            pos target;
            target.freq = x - 1;
            target.amp = y;
            if((!find(pos_history, target) || (target.freq == 1 && target.amp == 1)) && around_result.l < stress_level)
            {
                //commits this move to history
                pos_history[find_history_index(pos_history)] = target;
                return target;
            }
        }
        //up
        if(around_result.u != 1000)
        {
            pos target;
            target.freq = x;
            target.amp = y-1;
            if((!find(pos_history, target) || (target.freq == 1 && target.amp == 1)) && around_result.u < stress_level)
            {
                //commits this move to history
                pos_history[find_history_index(pos_history)] = target;
                return target;
            }
        }
        //right
        if(around_result.r != 1000)
        {
            pos target;
            target.freq = x+1;
            target.amp = y;
            if((!find(pos_history, target) || (target.freq == 1 && target.amp == 1)) && around_result.r < stress_level)
            {
                //commits this move to history
                pos_history[find_history_index(pos_history)] = target;
                return target;
            }
        }
        //down
        if(around_result.d != 1000)
        {
            pos target;
            target.freq = x;
            target.amp = y+1;
            if((!find(pos_history, target) || (target.freq == 1 && target.amp == 1)) && around_result.d < stress_level)
            {
                //commits this move to history
                pos_history[find_history_index(pos_history)] = target;
                return target;
            }
        }
    }
    return next_pos;
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


    uint32_t stress_matrix[5][5] = {0};


    //set known f and a values:

    stress_matrix[0][0] = 10;
    stress_matrix[4][4] = 100;

    pos pos_history[100];
    pos_history[0] = (pos){5,5};
    
    //printf("%u", stress_matrix[0][0]);

    // display setup

    displayFillScreen(&display, RGB_BLACK);

    // loop
    for (;;) {
        // read crying SM
        iic_read_register(
            IIC0, CRYING_SM_ADDR,
            CRYING_SM_LEVEL_REG,
            (void*)&crying_level, 4
        );
        sleep_msec(1);
        iic_read_register(
            IIC0, HEARTBEAT_SM_ADDR,
            HEARTBEAT_SM_HEARTRATE_REG,
            (void*)&heart_rate, 4
        );
        // write motor amplitude
        iic_write_register(
            IIC0, MOTOR_DRIVER_SM_ADDR,
            MOTOR_DRIVER_SM_AMPLITUDE_REG,
            (void*)&motor_amplitude, 4
        );
        sleep_msec(1);

        // write motor frequency
        iic_write_register(
            IIC0, MOTOR_DRIVER_SM_ADDR,
            MOTOR_DRIVER_SM_FREQUENCY_REG,
            (void*)&motor_frequency, 4
        );

        stress_level = calc_stress(crying_level, heart_rate, stress_level);
        stress_matrix[motor_frequency-1][motor_amplitude-1] = stress_level;
        pos next_pos = calc_next_pos(stress_matrix, stress_level, motor_amplitude, motor_frequency, pos_history);
        motor_frequency = next_pos.freq;
        motor_amplitude = next_pos.amp;
        printf("%d , %d stress in matrix %d. real stress %d\n", motor_frequency, motor_amplitude, stress_matrix[motor_frequency-1][motor_amplitude-1], stress_level);


        write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency, stress_level);
        sleep_msec(50);
    }

    // return
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}

