#ifndef _DEMOBOT_H_
#define _DEMOBOT_H_

#include <ros/ros.h>
#include <motor_ctrl/SetMotor.h>
#include <motor_ctrl/MotorData.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <inttypes.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <demobot_spi.h>

#define SPI_SPEED_HZ 	1000000

#define HDDR_SIZE 		1
#define REG_DATA_SIZE 	4
#define CMD_READ_REG  	(0x00)
#define WRITE_BIT 		7

#define CTRL_MODE_POWER 	0
#define CTRL_MODE_PWM 		1
#define CTRL_MODE_RPM 		2

enum STATUS {
	STATUS_OK = 0,
	STATUS_BAD_REG
};

enum REG_MAP {
	REG_STATUS = 0,
	MOTOR_1_SETPT,
	MOTOR_1_SPEED,
	MOTOR_1_TACH,
	MOTOR_1_POWER,
	MOTOR_1_PWM,
	MOTOR_2_SETPT,
	MOTOR_2_SPEED,
	MOTOR_2_TACH,
	MOTOR_2_POWER,
	MOTOR_2_PWM,
	TIMESTAMP,
	TICKS_PER_ROT,
	CONTROL_MODE,
	CONTROL_KP
};

class DemoBot{
public:
	DemoBot(ros::NodeHandle nh, std::string spi_path);
	int set_motor_setpt(int mtr, float setpt);
	int set_control_mode(int mode);
	float get_motor_speed(int mtr);
	int get_motor_tach(int mtr);
	int get_motor_power(int mtr);
	int get_motor_pwm(int mtr);
	int get_timestamp();
	void send_motor_updates();
	bool set_motor_callback(motor_ctrl::SetMotor::Request& request, motor_ctrl::SetMotor::Response& response);
	void cleanup();
private:
	ros::NodeHandle nh;
	ros::ServiceServer set_motor_srv;
	ros::Publisher motor_data_pub;
	int read_reg(uint8_t reg_id, void* value);
	int write_reg(uint8_t reg_id, void* value);
	int spi_handle;
	int write_stat;
	int read_stat;
	int control_mode;
};

#endif
