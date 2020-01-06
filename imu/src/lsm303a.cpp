#include <lsm303a.h>

Imu::Imu(){
  if((this->gpio_handle = pigpio_start(NULL, NULL)) < 0){
    ROS_INFO("GPIO initialization failed :(");
  }else{
    ROS_INFO("GPIO initialized successfully!");
    this->mag_i2c_handle = i2c_open(this->gpio_handle, 1, LSM303A_MAG_ADDR, 0);
    this->xl_i2c_handle = i2c_open(this->gpio_handle, 1, LSM303A_XL_ADDR, 0);

  }
  int xl_rpy = i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, WHO_AM_I_A);
  int mag_rpy = i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, WHO_AM_I_M);

  ROS_INFO("WHO AM I, accel: 0x%X, mag: 0x%X", xl_rpy, mag_rpy);
  this->imu_data_pub = this->nh.advertise<imu::ImuData>("imu_data", 10);

  /* General Purpose startup sequence
  1. Write CFG_REG_A_M = 00h // Mag = 10 Hz (high-resolution and continuous mode)
  2. Write CFG_REG_C_M = 01h // Mag data-ready interrupt enable
  3. Write CTRL_REG1_A = 57h // Accel = 100 Hz (normal mode)
  */
  int w_ok;
  w_ok = i2c_write_byte_data(this->gpio_handle, this->mag_i2c_handle, CFG_REG_A_M, 0x00);
  w_ok += i2c_write_byte_data(this->gpio_handle, this->mag_i2c_handle, CFG_REG_C_M, 0x01);
  w_ok += i2c_write_byte_data(this->gpio_handle, this->xl_i2c_handle, CTRL_REG1_A, 0x57);
  if(w_ok != 0)
    ROS_INFO("LSM303A setup failed :(");
  else
    ROS_INFO("LSM303A setup presumed successful");
}

int Imu::read_all_params(){
  this->x_accel = i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, OUT_X_L_A);
  this->x_accel |= i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, OUT_X_H_A) << 8;
  this->y_accel = i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, OUT_Y_L_A);
  this->y_accel |= i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, OUT_Y_H_A) << 8;
  this->z_accel = i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, OUT_Z_L_A);
  this->z_accel |= i2c_read_byte_data(this->gpio_handle, this->xl_i2c_handle, OUT_Z_H_A) << 8;

  this->x_mag = i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, OUTX_L_REG_M);
  this->x_mag |= i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, OUTX_H_REG_M) << 8;
  this->y_mag = i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, OUTY_L_REG_M);
  this->y_mag |= i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, OUTY_H_REG_M) << 8;
  this->z_mag = i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, OUTZ_L_REG_M);
  this->z_mag |= i2c_read_byte_data(this->gpio_handle, this->mag_i2c_handle, OUTZ_H_REG_M) << 8;

  this->x_mag_samples[this->sample_ind] = this->counts_to_gauss(this->x_mag);
  this->y_mag_samples[this->sample_ind] = this->counts_to_gauss(this->y_mag);
  this->z_mag_samples[this->sample_ind] = this->counts_to_gauss(this->z_mag);
  this->sample_ind++;

  if(this->sample_ind >= NUM_SAMPLES)
    this->sample_ind = 0;

  return 0;
}

float f_running_avg(float* samples, int n_samples){
  float weight = 1.0/float(n_samples);
  float retval = 0.0;
  for(int i = 0; i < n_samples; i++){
    retval += weight * samples[i];
  }
  return retval;
}

float Imu::counts_to_accel(int counts){
  // assuming normal mode
  int raw = counts & 0x3FF; // preserve only 10-bits
  if(raw > int((0x3ff / 2.0) + 0.5)){
    raw = raw - 0x3FF; // subtract the correct amount
  }
  return float(raw);//(-9.81*3.9*float(raw))/1000.0;
}

float Imu::counts_to_gauss(int counts){
  int16_t raw = int16_t(counts & 0xFFFF); // preserve only 16-bits and cast to correct sign
  return 1.5*float(raw)/1000.0;
}

void Imu::publish_data(){
  imu::ImuData msg;
  msg.timestamp = ros::Time::now();
  msg.xl_x = this->counts_to_accel(this->x_accel);
  msg.xl_y = this->counts_to_accel(this->y_accel);
  msg.xl_z = this->counts_to_accel(this->z_accel);
  msg.mag_x = f_running_avg(this->x_mag_samples, NUM_SAMPLES);
  msg.mag_y = f_running_avg(this->y_mag_samples, NUM_SAMPLES);
  msg.mag_z = f_running_avg(this->z_mag_samples, NUM_SAMPLES);
  this->imu_data_pub.publish(msg);
}
