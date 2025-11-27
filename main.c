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

display_t display;
FontxFile fx;

void write_to_screen(uint32_t crying_level, uint32_t heart_rate, uint32_t motor_amplitude, uint32_t motor_frequency)
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
    uint32_t heart_rate      = 0;
    uint32_t motor_amplitude = 0;
    uint32_t motor_frequency = 0;

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
        

        write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency);
        sleep_msec(200);
    }

    // return
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}


