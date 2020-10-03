#include <demobot_spi.h>

int spi_init(const char* path){
  int fd;

  if((fd = open(path, O_RDWR)) < 0){
    return SPI_ERR_OPEN_FAILED;
  }
  return fd;
}

int spi_set_speed(int f, uint32_t speed){
    // will return 0 if no error
    if(ioctl(f, SPI_IOC_WR_MAX_SPEED_HZ, &speed) != 0){
        return SPI_ERR_SET_SPEED_FAILED;
    }
    return 0;
}

int spi_set_mode(int f, uint8_t spi_mode){
    if(spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_1 && spi_mode != SPI_MODE_2 && spi_mode != SPI_MODE_3){
        spi_mode = SPI_MODE_0;
    }
    if(ioctl(f, SPI_IOC_WR_MODE, &spi_mode) != 0) {
        return SPI_ERR_SET_MODE_FAILED;
    }
    return 0;// 0 if success
}

char* spi_transfer(int fd, char * buf, int buf_len){
    struct spi_ioc_transfer xfer[2];
    int status;
    memset(xfer, 0, sizeof xfer);
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].rx_buf = (unsigned long)buf;
    xfer[0].len = buf_len;
    xfer[0].delay_usecs = 400;
    status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
    return buf;
}
