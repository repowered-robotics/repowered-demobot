#ifndef _DEMOBOT_SPI_H_
#define _DEMOBOT_SPI_H_

#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>

#define SPI_DEFAULT_MODE          SPI_MODE_0
#define SPI_ERR_OPEN_FAILED       (-1)
#define SPI_ERR_SET_MODE_FAILED   (-2)
#define SPI_ERR_SET_SPEED_FAILED  (-3)
#define SPI_ERR_WRITE_FAILED      (-4)
#define SPI_ERR_READ_FAILED       (-5)

int spi_init(const char* path);

/* set max clock speed in Hz */
int spi_set_speed(int f, uint32_t speed);

/* set the SPI mode */
int spi_set_mode(int f, uint8_t mode);

/* transfer buf_len bytes from buf, store received bytes in buf */
char* spi_transfer(int fd, char * buf, int buf_len);

#endif