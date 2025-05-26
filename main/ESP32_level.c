#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "inttypes.h"
#include "mpu6050.h"
#include "math.h"

#define BUTTON_PIN		GPIO_NUM_4
#define SDA_PIN			GPIO_NUM_21
#define SCL_PIN			GPIO_NUM_22

#define DEBOUNCE_TIME_MS	50

char message_buffer[40];
float pitch, pitch_0, pitch_1, roll, roll_0, roll_1;
mpu6050_acce_value_t acce_value_0, acce_value_1;

TimerHandle_t debounce_timer = NULL;
SemaphoreHandle_t xSemaphore = NULL;
mpu6050_handle_t mpu6050_0_handle = NULL, mpu6050_1_handle = NULL;

/* Conversion to Degrees */
float rad_to_deg(float radians)
{
	return radians * (180.0 / M_PI);
}

/* ISR: Handle interrupt when button is pressed */
static void IRAM_ATTR button_isr_handler(void* arg)
{
	xTimerResetFromISR(debounce_timer, NULL);
}

/* Debounce timer callback */
void debounce_timer_callback(TimerHandle_t Timer)
{
	if(gpio_get_level(BUTTON_PIN) == 0)
	{
		xSemaphoreGive(xSemaphore);
	}
}

void app_main(void)
{
	// BUTTON_GPIO configuration
	gpio_reset_pin(BUTTON_PIN);
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
	gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);

	// ISR initialization
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);


	/* HD44780 initialization
*/
	// Create timer debounce
	debounce_timer=	xTimerCreate("debounce_timer",
			pdMS_TO_TICKS(DEBOUNCE_TIME_MS),
			pdFALSE,
			NULL,
			debounce_timer_callback);

	if(debounce_timer != NULL)
	{
		printf("Debounce timer created successfully\n");
	}

	// I2C initialization
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = SCL_PIN,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000
	};
	
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

	// Create Semaphore
	xSemaphore = xSemaphoreCreateBinary();

	if(xSemaphore != NULL)
	{
		printf("Semaphore created successfully\n");
	}

	// MPU6050 initialization
	mpu6050_0_handle = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS_0);
	mpu6050_1_handle = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);

	mpu6050_config(mpu6050_0_handle, ACCE_FS_2G, GYRO_FS_250DPS);
	mpu6050_config(mpu6050_1_handle, ACCE_FS_2G, GYRO_FS_250DPS);

	if(mpu6050_0_handle != NULL && mpu6050_1_handle != NULL)
	{
		printf("Successful MPU6050 initialization\n");
	}
	
	mpu6050_wake_up(mpu6050_0_handle);
	mpu6050_wake_up(mpu6050_1_handle);
	
	while(1)
	{
		/*
		if(xSemaphoreTake(xSemaphore,(TickType_t) 0))
		{
			
		}
		*/
//		mpu6050_wake_up(mpu6050_0_handle);
		mpu6050_get_acce(mpu6050_0_handle, &acce_value_0);
//		mpu6050_sleep(mpu6050_0_handle);

//		mpu6050_wake_up(mpu6050_1_handle);
		mpu6050_get_acce(mpu6050_1_handle, &acce_value_1);
//		mpu6050_sleep(mpu6050_1_handle);



		
		pitch_0 = fabs(rad_to_deg(atan( acce_value_0.acce_x / sqrt( pow(acce_value_0.acce_y, 2) + pow(acce_value_0.acce_z, 2)))));
		pitch_1 = fabs(rad_to_deg(atan( acce_value_1.acce_x / sqrt( pow(acce_value_1.acce_y, 2) + pow(acce_value_1.acce_z, 2)))));
		pitch = (pitch_0 + pitch_1) / 2;
		printf("Pitch: %f\r\n", pitch);
		
		roll_0 = fabs(rad_to_deg(atan( acce_value_0.acce_y / sqrt( pow(acce_value_0.acce_x, 2) + pow(acce_value_0.acce_z, 2)))));
		roll_1 = fabs(rad_to_deg(atan( acce_value_1.acce_y / sqrt( pow(acce_value_1.acce_x, 2) + pow(acce_value_1.acce_z, 2)))));
		roll = (roll_0 + roll_1) / 2;
		printf("Roll: %f\r\n", roll);
		
		vTaskDelay(100);
	}
}

