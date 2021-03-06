/*******************************************************************************
* Copyright Mathieu Rondonneau (mathieu.rondonneau@gmail.com)
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/


#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <navio/navio_led.h>

#include <Navio/gpio.h>
#include "Navio/Util.h"
#include "Navio/PCA9685.h"

using namespace Navio;

const uint32_t led_tab[][3] = {
		//  B,    G,    R
		{4095,    0,    0}, // yellow
		{4095,    0, 4095}, // green
		{0   ,    0, 4095}, // cyan
		{0   , 4095, 4095}, // blue
		{0   , 4095,    0}, // magenta
		{4095, 4095,    0}, // red
};

enum led_idx color = LED_YELLOW;

void led_set(PCA9685 &pwm, enum led_idx color)
{
	uint32_t i;

	for (i=0; i<3; i++)
	{
		pwm.setPWM(i, led_tab[color][i]);
	}
}

void led_callback(const std_msgs::Int32ConstPtr msg)
{
	if ((msg->data <= 4095) &&
		(msg->data >= 0))
		color = (enum led_idx)msg->data;
}

int main(int argc, char *argv[])
{
    static const uint8_t outputEnablePin = RPI_GPIO_27;

    if (check_apm()) {
        return 1;
    }

    Pin pin(outputEnablePin);

    if (pin.init()) {
        pin.setMode(Pin::GpioModeOutput);
        pin.write(0); /* drive Output Enable low */
    } else {
        fprintf(stderr, "Output Enable not set. Are you root?\n");
        return 1;
    }

	ros::init(argc,argv,"navio_led");
	ros::NodeHandle n;

	// 1Hz
	ros::Rate loop_rate(1);

    PCA9685 pwm;

    pwm.initialize();

    // initialize subscribers
    ros::Subscriber led_sub = n.subscribe("navio_led",
    		1,
    		&led_callback);

    while(ros::ok())
    {
    	led_set(pwm, color);
    	ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}
