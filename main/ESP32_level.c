#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"
#include "mpu6050.h"
#include "hd44780.h"
#include "math.h"

#define BUTTON_PIN		GPIO_NUM_4
#define SDA_PIN			GPIO_NUM_21
#define SCL_PIN			GPIO_NUM_22

#define DEBOUNCE_TIME_MS	50

char message_buffer[40];
float pitch, pitch_0, pitch_1, roll, roll_0, roll_1;
mpu6050_acce_value_t acce_value_0, acce_value_1;

hd44780_t lcd = {
	.write_cb = NULL,
	.pins = {
		.rs =	GPIO_NUM_23,
		.e =	GPIO_NUM_18,
		.d4 =	GPIO_NUM_25,
		.d5 =	GPIO_NUM_26,
		.d6 =	GPIO_NUM_27,
		.d7 =	GPIO_NUM_14,
		.bl =	HD44780_NOT_USED
	},
	.font = HD44780_FONT_5X8,
	.lines = 2,
	.backlight = 0 
};

// TimerHandle_t debounce_timer = NULL;
mpu6050_handle_t mpu6050_0_handle = NULL, mpu6050_1_handle = NULL;

/* Conversion to Degrees */
float rad_to_deg(float radians)
{
	return radians * (180.0 / M_PI);
}

/*
// ISR: Handle interrupt when button is pressed
static void IRAM_ATTR button_isr_handler(void* arg)
{
	xTimerResetFromISR(debounce_timer, NULL);
}

// Debounce timer callback 
void debounce_timer_callback(TimerHandle_t Timer)
{
	if(gpio_get_level(BUTTON_PIN) == 0)
	{
		xSemaphoreGive(xSemaphore);
	}
}
*/

void ActiveTask(void* pvParameters)
{
	while(1)
	{
			printf("Entering ActiveTask...\r\n");

			mpu6050_wake_up(mpu6050_0_handle);
			mpu6050_wake_up(mpu6050_1_handle);

			mpu6050_get_acce(mpu6050_0_handle, &acce_value_0);
			mpu6050_get_acce(mpu6050_1_handle, &acce_value_1);

			roll_0 = fabs( rad_to_deg( atan( acce_value_0.acce_x / sqrt( pow(acce_value_0.acce_y, 2) + pow(acce_value_0.acce_z, 2)))));
			roll_1 = fabs( rad_to_deg( atan( acce_value_1.acce_x / sqrt( pow(acce_value_1.acce_y, 2) + pow(acce_value_1.acce_z, 2)))));
			roll = (roll_0 + roll_1) / 2;
			printf("Roll: %f\n", roll);
		
			pitch_0 = fabs( rad_to_deg( atan( acce_value_0.acce_y / sqrt( pow(acce_value_0.acce_x, 2) + pow(acce_value_0.acce_z, 2)))));
			pitch_1 = fabs( rad_to_deg( atan( acce_value_1.acce_y / sqrt( pow(acce_value_1.acce_x, 2) + pow(acce_value_1.acce_z, 2)))));
			pitch = (pitch_0 + pitch_1) / 2;
			printf("Pitch: %f\n", pitch);
		
			mpu6050_sleep(mpu6050_0_handle);
			mpu6050_sleep(mpu6050_1_handle);

			sprintf(message_buffer, "Roll: %.3f", roll);
			hd44780_gotoxy(&lcd, 0, 0);
			hd44780_puts(&lcd, message_buffer);

			sprintf(message_buffer, "Pitch: %.3f", pitch);
			hd44780_gotoxy(&lcd, 0, 1);
			hd44780_puts(&lcd, message_buffer);

			vTaskDelay(1000);			// Displays data for 10s before turning lcd off
			hd44780_control(&lcd, 0, 0, 0);

			printf("Entering Sleep...\n");
			esp_deep_sleep_start();
	}
}

void app_main(void)
{
	// BUTTON_GPIO configuration
	gpio_reset_pin(BUTTON_PIN);
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
	esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);
//	gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);

	// ISR initialization
/*	
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
	

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
*/	

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

	// HD44780 initialization
	if(hd44780_init(&lcd) == ESP_OK)
	{
		printf("Succesful LCD initialization\n");
	}

	if(hd44780_control(&lcd, 1, 0, 0) == ESP_OK)
	{
		printf("Turning on LCD\n");
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
	
	// Create Tasks
	xTaskCreate(ActiveTask, "Active", 8192, NULL, 3, NULL);

	// Checking if sleep works correctly
	esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

	if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    		printf("Woken up by button press!\n");
	}
}

