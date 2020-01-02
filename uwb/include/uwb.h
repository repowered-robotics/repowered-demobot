#ifndef UWB_H_
#define USB_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <uwb/UwbData.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define HDDR_SIZE       2
#define NUM_FIELDS      8
#define FIELD_SIZE 	4
#define STOP_BYTE       0xA5
#define ANCHOR_DATA_SIZE 33

enum CMD_TYPES {
	CMD_READ_CONFIG		= 0x11,
	CMD_READ_ANCHORS 	= 0x12,
	CMD_SET_CONFIG 		= 0x22,
	CMD_RANGE 		= 0x33,
	CMD_RESTART 		= 0x44,
	CMD_RESET 		= 0x55,
	CMD_SAVE_CONFIG 	= 0x66
};

enum FIELDS {
	FIELD_SELF_ID 	= 0x00,
	FIELD_MODE 		= 0x01,
	FIELD_CHANNEL 	= 0x02,
	FIELD_SAMPLES_PER_RANGE 	= 0x03,
	FIELD_NUMBER_OF_ANCHORS 	= 0x04,
	FIELD_X = 0x05,
	FIELD_Y = 0x06,
	FIELD_Z = 0x07
};

class AnchorData{
public:
    AnchorData() {}
    AnchorData(uint8_t* data);
    AnchorData(const AnchorData &anchor);
    uint32_t id;
    uint32_t timestamp;
    float x;
    float y;
    float z;
    float distance;
    float rx_power;
    float fp_power;
    float fp_snr;
	ros::Time ros_timestamp;
    void update_data(uint8_t* data);
    std::string to_string();
};

class UwbModule {
public:
	UwbModule(std::string name);
	std::string if_name;
	int com;
	int serial_status;
	uint8_t self_id;
	uint8_t mode;
	uint8_t channel;
	uint8_t samples_per_range;
	uint8_t number_of_anchors;
	float x;
	float y;
	float z;
	void read_config();
	void write_config();
	void read_anchors();
	std::string info;
	int find_anchor(uint8_t id);
	std::string anchors_to_string();
	std::string config_to_string();
	void publish_anchor_data();
private:
	ros::NodeHandle nh;
	ros::Publisher uwb_data_pub;
	std::vector<AnchorData> anchors;
	int open_serial(std::string name, std::string& error_msg);
	int serial_write(uint8_t* data, int len);
	int serial_read(uint8_t* data, int* len);
	void update_fields_from_rpy(uint8_t* rpy);
};

#endif
