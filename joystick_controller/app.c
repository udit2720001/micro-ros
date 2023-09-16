#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <driver/gpio.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#endif

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define JOYSTICK_X ADC1_CHANNEL_4
#define JOYSTICK_Y ADC1_CHANNEL_5
#define JOYSTICK_BUTTON 15
#define PWM_FREQUENCY 50

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;

void timer_callback(rcl_timer_t *timer, float last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		float adc1raw = adc1_get_raw(JOYSTICK_X);
		float adc2raw = adc1_get_raw(JOYSTICK_Y);

		float adc1org = 0;
		float adc2org = 0;

		bool joystick_button_state = gpio_get_level(JOYSTICK_BUTTON);

		// float adc1 = esp_adc_cal_raw_to_voltage(adc1raw, &adc1_chars); // for x axis
		// float adc2 = esp_adc_cal_raw_to_voltage(adc2raw, &adc2_chars); //

		float right_limit = -0.5;
		float left_limit = 0.5;
		if (joystick_button_state == 1) // for forward
		{
			adc1raw = adc1raw / 4096; // for x axis
			adc2raw = adc2raw - 2048; // for y axis
			adc2raw = adc2raw / 1024;
			if (adc2raw >= 0) // for left
			{
				if (adc2raw < left_limit)
				{
					msg.linear.x = adc1raw;
					msg.angular.z = 0;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
				else
				{
					adc2raw = adc2raw / 2;
					msg.linear.x = adc1raw;
					msg.angular.z = adc2raw;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
			}
			else if (adc2raw < 0) // for right
			{
				if (adc2raw > right_limit)
				{
					msg.linear.x = adc1raw;
					msg.angular.z = 0;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
				else
				{
					adc2raw = adc2raw / 2;
					msg.linear.x = adc1raw;
					msg.angular.z = adc2raw;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
			}
		}

		else if (joystick_button_state == 0) // for backward
		{
			adc1raw = (adc1raw / 4095) * (-1); // for x axis
			adc2raw = adc2raw - 2048;		   // for y axis
			adc2raw = adc2raw / 1024;
			if (adc2raw >= 0) // for left
			{
				if (adc2raw < left_limit)
				{
					msg.linear.x = adc1raw;
					msg.angular.z = 0;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
				else
				{
					adc2raw = adc2raw / 2;
					msg.linear.x = adc1raw;
					msg.angular.z = adc2raw;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
			}
			else if (adc2raw < 0) // for right
			{
				if (adc2raw > right_limit)
				{
					msg.linear.x = adc1raw;
					msg.angular.z = 0;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
				else
				{
					adc2raw = adc2raw / 2;
					msg.linear.x = adc1raw;
					msg.angular.z = adc2raw;
					RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				}
			}
		}

		vTaskDelay(1);

		// TO debug serially
		// printf("adc1: %f\n", adc1raw);
		// printf("adc2: %f\n", adc2raw);
		// printf("button: %d\n", joystick_button_state);

		// different approach for controlling the robot with joystick

		// printf("adc3: %f\n", adc3raw);
		// vTaskDelay(500/ portTICK_PERIOD_MS);
		// printf("adc4: %f\n", adc4);

		// if(adc1>upper_limit && adc2>upper_limit)
		// {
		// 	printf("forward\n");
		// 	adc3=adc3*(1);
		// 	adc4 = 0.0;
		//
		// 	msg.angular.z = adc4;
		// 	RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		// }
		// else if(adc1<lower_limit && adc2<lower_limit)
		// {
		// 	printf("backward\n");
		// 	adc3=adc3*(-1);
		// 	adc4 = 0.0;
		// 	msg.linear.x = adc3;
		// 	msg.angular.z = adc4;
		// 	RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		// }
		// else if(adc1>upper_limit && adc2<lower_limit)
		// {
		// 	printf("left\n");
		// 	adc4=adc4*(-1);
		// 	adc3 = 0.0;
		// 	msg.linear.x = adc3;
		// 	msg.angular.z = adc4;
		// 	RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		// }
		// else if(adc1<lower_limit && adc2>upper_limit)
		// {
		// 	printf("right\n");
		// 	adc4=adc4*(1);
		// 	adc3 = 0.0;
		// 	msg.linear.x = adc3;
		// 	msg.angular.z = adc4;
		// 	RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		// }

		// else
		// {
		// 	printf("stop\n");
		// 	msg.linear.x = 0;
		// 	msg.angular.z = 0;
		// 	RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		// }

		// printf("Publishing: '%f'\n", adc_value);
		//  vTaskDelay(1);
	}
}

void appMain(void *arg)
{
	adc1_config_width(ADC_WIDTH_BIT_12);

	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc2_chars);

	adc1_config_channel_atten(JOYSTICK_X, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(JOYSTICK_Y, ADC_ATTEN_DB_11);

	gpio_set_direction(JOYSTICK_BUTTON, GPIO_MODE_INPUT);
	gpio_set_pull_mode(JOYSTICK_BUTTON, GPIO_PULLUP_ONLY);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "freertos_teist_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"freertos_twist"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	// vTaskDelay(1);

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

	vTaskDelete(NULL);
}
