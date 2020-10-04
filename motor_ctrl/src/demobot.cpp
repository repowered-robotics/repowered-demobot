#include <demobot.h>

DemoBot::DemoBot(ros::NodeHandle nh, std::string spi_path){
	this->nh = nh;
	this->spi_handle = spi_init(spi_path.c_str());
	if(this->spi_handle < 0){
		ROS_ERROR("SPI initialization failed.");
	}else{
		spi_set_mode(this->spi_handle, SPI_DEFAULT_MODE);
		spi_set_speed(this->spi_handle, SPI_SPEED_HZ);
	}
	// motor data publisher
	this->motor_data_pub = this->nh.advertise<motor_ctrl::MotorData>("motor_data", 128);

	// provide the set_motor service
	this->set_motor_srv = this->nh.advertiseService("set_motor", &DemoBot::set_motor_callback, this);
}

bool DemoBot::set_motor_callback(motor_ctrl::SetMotor::Request& request, motor_ctrl::SetMotor::Response& response){
	int new_mode = (int)request.control_mode;
	if(new_mode != this->control_mode){
		this->control_mode = new_mode;
		this->set_control_mode(this->control_mode);
	}
	if(request.motor_id == 1 || request.motor_id == 2){
		response.status = this->set_motor_setpt(request.motor_id, request.setpt);
		return true;
	}else{
		response.status = -1;
		return true;
	}
}

void DemoBot::send_motor_updates(){
	motor_ctrl::MotorData msg_1;
	motor_ctrl::MotorData msg_2;

	msg_1.id = 1;
	msg_1.rpm = this->get_motor_speed(1);
	msg_1.pwm = this->get_motor_power(1);
	msg_1.tach = this->get_motor_tach(1);
	msg_1.timestamp = ros::Time::now();

	msg_2.id = 2;
	msg_2.rpm = this->get_motor_speed(2);
	msg_2.pwm = this->get_motor_power(2);
	msg_2.tach = this->get_motor_tach(2);
	msg_2.timestamp = ros::Time::now();
	
	this->motor_data_pub.publish(msg_1);
	this->motor_data_pub.publish(msg_2);
}

int DemoBot::read_reg(uint8_t reg_id, void* value){
	uint8_t data[REG_DATA_SIZE];
	memset(data, 0, REG_DATA_SIZE);
	
	struct spi_ioc_transfer xfer[2];
	int status;
	memset(xfer, 0, sizeof xfer);
	
    xfer[0].tx_buf 	= (unsigned long)(&reg_id);
    xfer[0].rx_buf 	= (unsigned long)(&reg_id);
    xfer[0].len 	= 1;
    xfer[0].delay_usecs = 500;

	xfer[1].tx_buf  = (unsigned long)data;
    xfer[1].rx_buf 	= (unsigned long)data;
	xfer[1].len 	= REG_DATA_SIZE;

    status = ioctl(this->spi_handle, SPI_IOC_MESSAGE(2), xfer);
	memcpy(value, data, REG_DATA_SIZE);

	return status;
}

int DemoBot::write_reg(uint8_t reg_id, void* value){
	
	struct spi_ioc_transfer xfer[2];
	int status;
	reg_id |= (1 << 7);
	memset(xfer, 0, sizeof xfer);
    xfer[0].tx_buf 	= (unsigned long)(&reg_id);
    xfer[0].rx_buf 	= (unsigned long)(&reg_id);
    xfer[0].len 	= 1;
    xfer[0].delay_usecs = 500;

	xfer[1].tx_buf  = (unsigned long)value;
    xfer[1].rx_buf 	= (unsigned long)value;
	xfer[1].len 	= REG_DATA_SIZE;

    status = ioctl(this->spi_handle, SPI_IOC_MESSAGE(2), xfer);

	return status;
}

int DemoBot::get_timestamp(){
	int retval;
	
	if(this->read_reg(TIMESTAMP, &retval) < 0)
		ROS_INFO("Read timestamp failed");

	return retval;
}

int DemoBot::set_motor_setpt(int mtr, float setpt){
	int retval;
	if(mtr <= 1){
		retval = this->write_reg(MOTOR_1_SETPT, &setpt);
	}else{
		retval = this->write_reg(MOTOR_2_SETPT, &setpt);
	}
	return retval;
}


int DemoBot::set_control_mode(int mode){
	return this->write_reg(CONTROL_MODE, &mode);
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
		this->read_reg(MOTOR_1_TACH, &retval);
	else
		this->read_reg(MOTOR_2_TACH, &retval);

	return retval;
}

int DemoBot::get_motor_power(int mtr){
	int retval;

	if(mtr <= 1)
		this->read_reg(MOTOR_1_POWER, &retval);
	else
		this->read_reg(MOTOR_2_POWER, &retval);

	return retval;
}

int DemoBot::get_motor_pwm(int mtr){
	int retval;

	if(mtr <= 1)
		this->read_reg(MOTOR_1_PWM, &retval);
	else
		this->read_reg(MOTOR_2_PWM, &retval);

	return retval;
}

void DemoBot::cleanup(){
	close(this->spi_handle);
}