/*
This code is provided under the BSD license.
Copyright (c) 2014, Emlid Limited. All rights reserved.
Written by Igor Vereninov and Mikhail Avkhimenia
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Application: Mahory AHRS algorithm supplied with data from MPU9250.
Outputs roll, pitch and yaw in the console and publish quaternion.

To achieve stable loop you need to run this application with a high priority
on a linux kernel with real-time patch. Raspbian distribution with real-time
kernel is available at emlid.com and priority can be set with chrt command:
chrt -f -p 99 PID
*/

#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include "Navio/MPU9250.h"
#include "AHRS.hpp"
#include "sensor_msgs/Imu.h"

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

MPU9250 imu;    // MPU9250
AHRS    ahrs;   // Mahony AHRS
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, yaw;
float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;
int sockfd;
struct sockaddr_in servaddr = {0};
char sendline[80];


void imuSetup()
{
    imu.initialize();

	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}

	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
}


int main(int argc, char *argv[])
{
	ros::init(argc,argv,"navio_ahrs");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu/data",1);
	ros::Time current_time, last_time;

	imuSetup();

	// Container for sensor data
	sensor_msgs::Imu imu_msg = sensor_msgs::Imu();

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(100);

	while(n.ok()) {


		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();

		// Read raw measurements from the MPU and update AHRS

#if 1
		// Accel + gyro.
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

#else
		// Accel + gyro + mag.
		// Soft and hard iron calibration required for proper function.
		imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
		ahrs.update(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, my, mx, -mz, dt);
#endif
		//------------------------ Read Euler angles

		ahrs.getEuler(&roll, &pitch, &yaw);

		// Discard the time of the first cycle

		if (!isFirst) {
			if (dt > maxdt) maxdt = dt;
			if (dt < mindt) mindt = dt;
		}
		isFirst = 0;

		// Network output with a lowered rate

		imu_msg.header.stamp = current_time;
		imu_msg.header.frame_id = "navio";

		imu_msg.orientation.x = ahrs.getX();
		imu_msg.orientation.y = ahrs.getY();
		imu_msg.orientation.z = ahrs.getZ();
		imu_msg.orientation.w = ahrs.getW();;

		//imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
		imu_msg.angular_velocity.x = gx;
		imu_msg.angular_velocity.y = gy;
		imu_msg.angular_velocity.z = gz;

		//imu_msg.linear_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
		imu_msg.linear_acceleration.x = ax;
		imu_msg.linear_acceleration.y = ay;
		imu_msg.linear_acceleration.z = az;

		pub.publish(imu_msg);

		ros::spinOnce();

		last_time = current_time;
		r.sleep();
	}
}

