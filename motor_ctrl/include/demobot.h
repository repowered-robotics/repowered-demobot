#ifndef _DEMOBOT_H_
#define _DEMOBOT_H_

#include <ros/ros.h>
#include <motor_ctrl/SetMotor.h>
#include <inttypes.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <pigpiod_if2.h>

#define SPI_SPEED_HZ 	1000000

#define HDDR_SIZE 	1
#define REG_DATA_SIZE 	4
#define CMD_READ_REG  	(0x00)
#define WRITE_BIT 	7

enum REG_MAP {
	MOTOR_1_SETPT,
	MOTOR_1_SPEED,
	MOTOR_1_TACH,
	MOTOR_1_POWER,
	MOTOR_2_SETPT,
	MOTOR_2_SPEED,
	MOTOR_2_TACH,
	MOTOR_2_POWER,
	TIMESTAMP
};

class DemoBot{
public:
	DemoBot(void);
	int set_motor_speed(int mtr, float speed);
	float get_motor_speed(int mtr);
	int get_motor_tach(int mtr);
	int get_motor_power(int mtr);
	int get_timestamp();
	bool set_motor_callback(motor_ctrl::SetMotor::Request& request, motor_ctrl::SetMotor::Response& response);
private:
	int read_reg(uint8_t reg_id, void* value);
	int gpio_handle;
	int spi_handle;
	int write_stat;
	int read_stat;
};

#endif
