//
// Created by marijn on 10/14/25.
//
#include <libpynq.h>
#include <fontx.h>

#define DECISION_SM_ADDR		0x50U

#define MOTOR_DRIVER_SM_ADDR			0x51U
#define MOTOR_DRIVER_SM_AMPLITUDE_REG	0x00U
#define MOTOR_DRIVER_SM_FREQUENCY_REG	0x01U

#define HEARTBEAT_SM_ADDR			0x52U
#define HEARTBEAT_SM_HEARTRATE_REG	0x00U

#define CRYING_SM_ADDR			0x53U
#define CRYING_SM_LEVEL_REG		0x00U


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
  
	InitFontx(&fx, "/home/student/libpynq-5EID0-2023-v0.3.0/fonts/ILGH24XB.FNT", "");
  
	display_init(&display);
	switchbox_init();

	switchbox_set_pin(IO_AR12, SWB_IIC0_SCL);
	switchbox_set_pin(IO_AR13, SWB_IIC0_SDA);

	iic_init(IIC0);
	iic_reset(IIC0);

	// vars
	uint32_t crying_level =		0;
	uint32_t heart_rate =		0;
	uint32_t motor_amplitude =	0;
	uint32_t motor_frequency =	0;

	displayFillScreen(&display, RGB_BLACK);
	

	// loop
	for (;;) {
		if (iic_read_register(
			IIC0, CRYING_SM_ADDR,
			CRYING_SM_LEVEL_REG,
			(uint8_t*)&crying_level, 4
		)) {
			printf("Error reading crying_sm\n");
		} else {
			printf("crying_sm: ");
			for (uint8_t c = 0; c < 4; c++) {
				printf("%c", ((char*)(&crying_level))[3-c]);
			}
			printf("\n");
		}
		
    
		if (iic_read_register(
			IIC0, HEARTBEAT_SM_ADDR,
			HEARTBEAT_SM_HEARTRATE_REG,
			(uint8_t*)&heart_rate, 4
		)) {
			printf("Error reading heartbeat_sm\n");
		} else {
			printf("heartbeat_sm: ");
			for (uint8_t c = 0; c < 4; c++) {
				printf("%c", ((char*)(&heart_rate))[3-c]);
			}
			printf("\n");
		}
		

		
		if (iic_write_register(
			IIC0, MOTOR_DRIVER_SM_ADDR,
			MOTOR_DRIVER_SM_AMPLITUDE_REG,
			(uint8_t*)&motor_amplitude, 4
		)) {
			printf("Error writing motor amplitude\n");
		}
		

		if (iic_write_register(
			IIC0, MOTOR_DRIVER_SM_ADDR,
			MOTOR_DRIVER_SM_FREQUENCY_REG,
			(uint8_t*)&motor_frequency, 4
		)) {
			printf("Error writing motor frequency\n");
		}
		

		write_to_screen(crying_level, heart_rate, motor_amplitude, motor_frequency);
		sleep_msec(600);  
	}


	// return
	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}
