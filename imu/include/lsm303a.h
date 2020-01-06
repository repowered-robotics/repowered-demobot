#ifndef _LSM303A_H_
#define _LSM303A_H_

#include <ros/ros.h>
#include <imu/ImuData.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <inttypes.h>
#include <pigpiod_if2.h>

#define NUM_SAMPLES 4

#define LSM303A_MAG_ADDR  30
#define LSM303A_XL_ADDR   25
#define WHO_AM_I_A  0x0F
#define WHO_AM_I_M  0x4F

#define CFG_REG_A_M 0x60
#define CFG_REG_C_M 0x62
#define CTRL_REG1_A 0x20

#define STATUS_REG_A  0x27
#define OUT_X_L_A     0x28
#define OUT_X_H_A     0x29
#define OUT_Y_L_A     0x2A
#define OUT_Y_H_A     0x2B
#define OUT_Z_L_A     0x2C
#define OUT_Z_H_A     0x2D

#define STATUS_REG_M 0x67
#define OUTX_L_REG_M 0x68
#define OUTX_H_REG_M 0x69
#define OUTY_L_REG_M 0x6A
#define OUTY_H_REG_M 0x6B
#define OUTZ_L_REG_M 0x6C
#define OUTZ_H_REG_M 0x6D

float f_running_avg(float* samples, int n_samples);

class Imu {
public:
  Imu();
  int x_accel;
  int y_accel;
  int z_accel;
  int x_mag;
  int y_mag;
  int z_mag;
  int read_all_params();
  void publish_data();
  float counts_to_accel(int counts);
  float counts_to_gauss(int counts);
private:
  int sample_ind = 0;
  float x_mag_samples[NUM_SAMPLES];
  float y_mag_samples[NUM_SAMPLES];
  float z_mag_samples[NUM_SAMPLES];
  int gpio_handle;
  int mag_i2c_handle;
  int xl_i2c_handle;
  ros::Publisher imu_data_pub;
  ros::NodeHandle nh;
};

#endif
