/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control RGB LED with PCA9685 driver onboard of Navio shield for Raspberry Pi.

RGB LED is connected to 0,1,2 channels of PWM controller PCA9685.
As channels are connected to LED's cathodes the logic is inverted,
that means that 0 value sets max brightness and 4095 sets min brightness.

To run this example navigate to the directory containing it and run following commands:
make
./LED
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <Navio/gpio.h>
#include "Navio/Util.h"
#include "Navio/PCA9685.h"

using namespace Navio;

enum led_idx {
	LED_YELLOW,
	LED_GREEN,
	LED_CYAN,
	LED_BLUE,
	LED_MAGENTA,
	LED_RED,
};

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
