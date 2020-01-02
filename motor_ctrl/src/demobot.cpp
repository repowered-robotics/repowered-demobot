#include <demobot.h>

DemoBot::DemoBot(void){
	
	if ((this->gpio_handle = pigpio_start(NULL, NULL)) < 0){//gpioInitialise() < 0){
		// pigpio initialisation failed.
		ROS_INFO("pigpio initialization failed :(");
	}else{	
		// pigpio initialised okay.
		ROS_INFO("pigpio initialized OK!");
	}

	uint32_t spi_config = 0;
	this->spi_handle = spi_open(gpio_handle, 0, SPI_SPEED_HZ, spi_config);
}

bool DemoBot::set_motor_callback(motor_ctrl::SetMotor::Request& request, motor_ctrl::SetMotor::Response& response){
	if(request.motor_id == 1 || request.motor_id == 2){
		response.status = this->set_motor_speed(request.motor_id, request.setpt);
		return true;
	}else{
		response.status = -1;
		return true;
	}
}

int DemoBot::read_reg(uint8_t reg_id, void* value){
	char cmd[HDDR_SIZE + REG_DATA_SIZE];
	cmd[0] = reg_id;
	memset(cmd + HDDR_SIZE, 0, REG_DATA_SIZE);
	
	char rpy[HDDR_SIZE + REG_DATA_SIZE];
	
	this->write_stat = spi_write(this->gpio_handle, this->spi_handle, cmd, HDDR_SIZE + REG_DATA_SIZE);//spiWrite(spi_fd, test_tx_data, 5);
	ros::Duration(0.005).sleep();
	this->read_stat = spi_read(this->gpio_handle, this->spi_handle, rpy, HDDR_SIZE + REG_DATA_SIZE);
	
	memcpy(value, rpy + HDDR_SIZE, REG_DATA_SIZE);
	
	if(this->write_stat == HDDR_SIZE + REG_DATA_SIZE && this->read_stat == HDDR_SIZE + REG_DATA_SIZE && rpy[0] != 0xFF){
		return 0;
	}else{
		return -1;
	}
}

int DemoBot::get_timestamp(){
	int retval;
	
	if(this->read_reg(TIMESTAMP, &retval) < 0)
		ROS_INFO("Read timestamp failed");

	return retval;
}

int DemoBot::set_motor_speed(int mtr, float speed){
	char cmd[HDDR_SIZE + REG_DATA_SIZE];
	char rpy[HDDR_SIZE + REG_DATA_SIZE];
	
	if(mtr <= 1)
		cmd[0] = (1 << 7) | MOTOR_1_SETPT;
	else
		cmd[0] = (1 << 8) | MOTOR_2_SETPT;
	this->write_stat = spi_write(this->gpio_handle, this->spi_handle, cmd, HDDR_SIZE + REG_DATA_SIZE);//spiWrite(spi_fd, test_tx_data, 5);
	ros::Duration(0.005).sleep();
	this->read_stat = spi_read(this->gpio_handle, this->spi_handle, rpy, HDDR_SIZE + REG_DATA_SIZE);
	if(this->write_stat == HDDR_SIZE + REG_DATA_SIZE && this->read_stat == HDDR_SIZE + REG_DATA_SIZE && rpy[0] != 0xFF){
		return 0;
	}else{
		return -1;
	}
}


float DemoBot::get_motor_speed(int mtr){
	float retval;
	
	if(mtr <= 1)
		this->read_reg(MOTOR_1_SPEED, &retval);
	else
		this->read_reg(MOTOR_2_SPEED, &retval);

	return retval;
}

int DemoBot::get_motor_tach(int mtr){
	int retval;

	if(mtr <= 1)
		this->read_reg(MOTOR_1_SPEED, &retval);
	else
		this->read_reg(MOTOR_2_SPEED, &retval);

	return retval;
}

int DemoBot::get_motor_power(int mtr){
	int retval;

	if(mtr <= 1)
		this->read_reg(MOTOR_1_SPEED, &retval);
	else
		this->read_reg(MOTOR_2_SPEED, &retval);

	return retval;
}

