//
// Created by marijn on 10/14/25.
//
#include <libpynq.h>

#define DECISION_SM_ADDR		0x50U

#define MOTOR_DRIVER_SM_ADDR			0x51U
#define MOTOR_DRIVER_SM_AMPLITUDE_REG	0x00U
#define MOTOR_DRIVER_SM_FREQUENCY_REG	0x01U

#define HEARTBEAT_SM_ADDR			0x52U
#define HEARTBEAT_SM_HEARTRATE_REG	0x00U

#define CRYING_SM_ADDR			0x53U
#define CRYING_SM_LEVEL_REG		0x00U


display_t display;

int main(void) {
	// init
	pynq_init();
	display_init(&display);
	switchbox_init();
	iic_init(IIC0);

	// pins
	switchbox_set_pin(IO_PMODA3, SWB_IIC0_SCL);
	switchbox_set_pin(IO_PMODA4, SWB_IIC0_SDA);

	// vars
	uint32_t crying_level =		0;
	uint32_t heart_rate =		0;
	uint32_t motor_amplitude =	0x414D5021UL;
	uint32_t motor_frequency =	0x66726571UL;

	displayFillScreen(&display, RGB_BLUE);
	displayDrawPixel(&display, 50, 50, RGB_YELLOW);
	displayDrawFillRect(&display, 10, 100, 110, 200, RGB_RED);
	displayDrawCircle(&display, 60, 40, 15, RGB_RED);

	// loop
	for (;;) {
		iic_read_register(
			IIC0, CRYING_SM_ADDR,
			CRYING_SM_LEVEL_REG,
			(void*)&crying_level, 4
		);
		iic_read_register(
			IIC0, HEARTBEAT_SM_ADDR,
			HEARTBEAT_SM_HEARTRATE_REG,
			(void*)&heart_rate, 4
		);

		printf("crying_sm: ");
		for (uint8_t c = 0; c < 4; c++) {
			printf("%c", ((char*)(&crying_level))[3-c]);
		}
		printf("\n");

		printf("heartbeat_sm: ");
		for (uint8_t c = 0; c < 4; c++) {
			printf("%c", ((char*)(&heart_rate))[3-c]);
		}
		printf("\n");

		iic_write_register(
			IIC0, MOTOR_DRIVER_SM_ADDR,
			MOTOR_DRIVER_SM_AMPLITUDE_REG,
			(void*)&motor_amplitude, 4
		);
		iic_write_register(
			IIC0, MOTOR_DRIVER_SM_ADDR,
			MOTOR_DRIVER_SM_FREQUENCY_REG,
			(void*)&motor_frequency, 4
		);


		sleep_msec(100);
	}


	// return
	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}


/*
 for (uint8_t i = 0; i < CRYING_SM_RMAP_SIZE; i++) {
			crying_level = rand();
			while (iic_write_register(
				IIC0, CRYING_SM_ADDR,
				i,
				(void*)&crying_level, 4
			)) {
				sleep_msec(10);
			}
			//printf("read: %ul\n", crying_level);
		}
 */