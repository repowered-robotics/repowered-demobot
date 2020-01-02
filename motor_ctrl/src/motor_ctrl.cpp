#include <motor_ctrl.h>
/*
int spi_open(std::string fname){
	int retval = open(fname.c_str(), O_RDWR); // open for read and write	
	if(retval < 0)
		ROS_INFO("Could not open %s for read-write", fname.c_str());

	// SPI_IOC_RD_MAX_SPEED_HZ, SPI_IOC_WR_MAX_SPEED_HZ
	uint32_t spi_speed = SPI_SPEED_HZ;

	if(ioctl(retval, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed) < 0)
		ROS_INFO("Could not assign SPI read speed");

	if(ioctl(retval, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
		ROS_INFO("Could not assign SPI write speed");

	return retval;
}
*/
