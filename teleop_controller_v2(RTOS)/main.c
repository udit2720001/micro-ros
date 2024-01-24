#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#define JOYSTICK_BUTTON 15
#define JOYSTICK_X ADC1_CHANNEL_4
#define JOYSTICK_Y ADC1_CHANNEL_5

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		float adc1raw = adc_oneshot_read(JOYSTICK_X);
		float adc2raw = adc1_get_raw(JOYSTICK_Y);

		bool joystick_button_state = gpio_get_level(JOYSTICK_BUTTON);

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
	}
}

void micro_ros_task(void *arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	// RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "teleop_joystick_controller", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel_controller"));

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

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

void app_main(void)
{

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	gpio_set_direction(JOYSTICK_BUTTON, GPIO_MODE_INPUT);
	gpio_set_pull_mode(JOYSTICK_BUTTON, GPIO_PULLUP_ONLY);

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(JOYSTICK_X, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(JOYSTICK_Y, ADC_ATTEN_DB_11);

	// pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task, "uros_task", 4096, NULL, 5, NULL);
}