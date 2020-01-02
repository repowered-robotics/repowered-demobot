#include <uwb.h>

UwbModule::UwbModule(std::string name){
	this->if_name = name;
	this->info.append("UART STATUS: ");

	this->serial_status = this->open_serial(this->if_name, this->info);
	ROS_INFO(this->info.c_str());
	
	this->uwb_data_pub = this->nh.advertise<uwb::UwbData>("uwb_data", 10); 
}

void UwbModule::publish_anchor_data(){
	uwb::UwbData msg;

	for(auto anchor = this->anchors.begin(); anchor != this->anchors.end(); ++anchor){
		msg.timestamp 	= (*anchor).ros_timestamp;
		msg.x 		= (*anchor).x;
		msg.y 		= (*anchor).y;
		msg.z 		= (*anchor).z;
		msg.distance 	= (*anchor).distance;
		this->uwb_data_pub.publish(msg);
	}
}

/**
 * @return 0 if open was a success, negative numbers for errors, positive numbers for warnings
 */
int UwbModule::open_serial(std::string name, std::string& error_msg){
	int retval = 0;
		
	int fd = open(name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1){
		retval -= 2;
		error_msg.append("Could not open " + name + "\r\n");
		return retval;	
	}		

	if(!isatty(fd)){
		retval -= 1;
		error_msg.append(name + " is not a TTY device.\r\n");
		return retval;
	}

	struct termios  config;

	//
	// Get the current configuration of the serial interface
	//
	if(tcgetattr(fd, &config) < 0) {
		retval -= 3;
		error_msg.append("Could not read configuration\r\n");
		return retval;
	}

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off, 
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	//
	// Turn off character processing
	//
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;

	//
	// Communication speed (simple version, using the predefined
	// constants)
	//
	if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
		retval -= 4;
		error_msg.append("Could not set baud rate\r\n");
		return retval;
	}
	
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0) { 
		retval -= 5;
		error_msg.append("Could not apply new configuration\r\n");
		return retval;
	}	
	
	this->com = fd;
	
	if(retval == 0){
		error_msg.append("No error, setup success!\r\n");
	}
	return retval;
}

std::string UwbModule::config_to_string(){
	char temp[256];
	sprintf(temp, "ID: %d\r\nMode: %d\r\nChannel: %d\r\nSamples: %d\r\nAnchors: %d\r\n", this->self_id, this->mode, this->channel, this->samples_per_range, this->number_of_anchors);
	return std::string(temp);
}

void UwbModule::update_fields_from_rpy(uint8_t* rpy){
	int packet_size = HDDR_SIZE + rpy[1] + 1; // expected packet size
	int ind = HDDR_SIZE; // start at beginning of the body
	uint8_t field_id = 0;
	uint32_t temp = 0;
	for( ; ind < packet_size - 1; ind += FIELD_SIZE ){
		field_id = rpy[ind++];
		switch(field_id){
		case FIELD_SELF_ID:{
			temp = rpy[ind] | rpy[ind+1] << 8 | rpy[ind+2] << 16 | rpy[ind+3] << 24;
			this->self_id = (uint8_t)(temp & 0xFF);
			break;
		}case FIELD_MODE:{
			temp = rpy[ind] | rpy[ind+1] << 8 | rpy[ind+2] << 16 | rpy[ind+3] << 24;
			this->mode = (uint8_t)(temp & 0xFF);
			break;
		}case FIELD_CHANNEL:{	
			temp = rpy[ind] | rpy[ind+1] << 8 | rpy[ind+2] << 16 | rpy[ind+3] << 24;
			this->channel = (uint8_t)(temp & 0xFF);
			break;
		}case FIELD_SAMPLES_PER_RANGE:{
			temp = rpy[ind] | rpy[ind+1] << 8 | rpy[ind+2] << 16 | rpy[ind+3] << 24;
			this->samples_per_range = (uint8_t)(temp & 0xFF);
			break;
		}case FIELD_NUMBER_OF_ANCHORS:{
			temp = rpy[ind] | rpy[ind+1] << 8 | rpy[ind+2] << 16 | rpy[ind+3] << 24;
			this->number_of_anchors = (uint8_t)(temp & 0xFF);
			break;
		}case FIELD_X:{
			memcpy(&this->x, rpy + ind, FIELD_SIZE);
			break;
		}case FIELD_Y:{
			memcpy(&this->y, rpy + ind, FIELD_SIZE);
			break;
		}case FIELD_Z:{
			memcpy(&this->z, rpy + ind, FIELD_SIZE);
			break;
		}default:
			break;
		}
	}
}

void UwbModule::read_config(){	
	if(this->serial_status != 0){
		return;
	}

	int packet_size = HDDR_SIZE + NUM_FIELDS + 1;
	uint8_t cmd[packet_size];
	
	int ind = 0, field_id = 0;
	cmd[ind++] = CMD_READ_CONFIG;
	cmd[ind++] = NUM_FIELDS;
	
	for( ; ind < packet_size - 1; ind++){
		cmd[ind] = field_id++;
	}
	cmd[ind++] = STOP_BYTE; 

	int write_status = this->serial_write(cmd, packet_size);
	
	if(write_status < 0){
		ROS_INFO("Write error\r\n");
		return;
	}
	
	int rpy_size = HDDR_SIZE + NUM_FIELDS*(FIELD_SIZE+1) + 1;
	int read_size = rpy_size;
	uint8_t rpy[rpy_size];
	
	ros::Duration(0.01).sleep();
	int read_status = this->serial_read(rpy, &read_size);

	if(read_size != rpy_size){
		ROS_INFO("Unexpected read size\r\n");
	}

	this->update_fields_from_rpy(rpy);
}

void UwbModule::write_config(){
	int packet_size = HDDR_SIZE + NUM_FIELDS*(0 + FIELD_SIZE) + 1;
	uint8_t cmd[packet_size];
	
	int ind = 0, field_id = 0;
	cmd[ind++] = CMD_SET_CONFIG;
	cmd[ind++] = NUM_FIELDS*(1 + FIELD_SIZE); // will populate late

	for( ; ind < packet_size - 1; ind += FIELD_SIZE ){
		cmd[ind++] = field_id;
		switch(field_id){
		case FIELD_SELF_ID:{
			cmd[ind] = this->self_id;
			break;
		}case FIELD_MODE:{
			cmd[ind] = this->mode;
			break;
		}case FIELD_CHANNEL:{
			cmd[ind] = this->channel;
			break;
		}case FIELD_SAMPLES_PER_RANGE:{
			cmd[ind] = this->samples_per_range;
			break;
		}case FIELD_NUMBER_OF_ANCHORS:{
			cmd[ind] = this->number_of_anchors;
			break;
		}case FIELD_X:{
			memcpy(cmd + ind, &this->x, FIELD_SIZE);
			break;
		}case FIELD_Y:{
			memcpy(cmd + ind, &this->y, FIELD_SIZE);
			break;
		}case FIELD_Z:{
			memcpy(cmd + ind, &this->z, FIELD_SIZE);
			break;
		}default:
			break;
		}
	}
	cmd[ind++] = STOP_BYTE;

	int write_status = this->serial_write(cmd, packet_size);
	
	if(write_status < 0){
		this->info.append("Write error\r\n");
		return;
	}
	
	int rpy_size = HDDR_SIZE + 1;
	int read_size = rpy_size;
	uint8_t rpy[rpy_size];

	int read_status = this->serial_read(rpy, &read_size);

	if(read_size != rpy_size){
		this->info.append("Unexpected read size\r\n");
	}

}

void UwbModule::read_anchors(){
	uint8_t cmd[HDDR_SIZE + 1];

	cmd[0] = CMD_READ_ANCHORS;
	cmd[1] = 0;
	cmd[2] = STOP_BYTE;

	int write_status = this->serial_write(cmd, HDDR_SIZE + 1);

	ros::Duration(0.01).sleep(); // sleep for 10ms

	uint8_t rpy_hddr[HDDR_SIZE];
	int read_size = HDDR_SIZE;
	this->serial_read(rpy_hddr, &read_size);
	
	if(read_size != HDDR_SIZE){
		return;
	}

	read_size = rpy_hddr[1]; // body size
	uint8_t rpy_body[read_size];
	this->serial_read(rpy_body, &read_size);
	
	if(read_size != rpy_hddr[1]){
		return;
	}
	
	int n_anchors = read_size / (ANCHOR_DATA_SIZE); // number of anchors	
	uint8_t rpy_stop;
	read_size = 1;
	this->serial_read(&rpy_stop, &read_size);
	
	int anchor_ind = 0, existing_ind = 0;
	uint8_t anchor_id = 0;
	while(anchor_ind < n_anchors){
		anchor_id = rpy_body[anchor_ind * ANCHOR_DATA_SIZE];
		if((existing_ind = this->find_anchor(anchor_id)) >= 0){
			// update anchor
			this->anchors.at(existing_ind).update_data(rpy_body + anchor_ind*ANCHOR_DATA_SIZE);
		}else{
			AnchorData anchor(rpy_body + anchor_ind*ANCHOR_DATA_SIZE);
			this->anchors.push_back(anchor); // this anchor doesn't exist yet, add it now
		}
		anchor_ind++;
	}
}

int UwbModule::find_anchor(uint8_t id){
	for(int i = 0; i < this->anchors.size(); i++){
		if(this->anchors.at(i).id == id){
			return i;
		}
	}
	return -1;
}

std::string UwbModule::anchors_to_string(){
	std::string retval;
	for(int i = 0; i < this->anchors.size(); i++){
		retval.append(this->anchors[i].to_string());
		retval.append("\r\n");
	}

	return retval;
}

/**
 *
 */
int UwbModule::serial_write(uint8_t* data, int len){
	int retval = 0;
	
	if(this->serial_status >= 0){
		retval = write(this->com, data, len);
	}else{
		retval = -1;
	}
	
	return retval;
}

/**
 * 
 */
int UwbModule::serial_read(uint8_t* data, int* len){
	int retval = 0;
	
	if(this->serial_status >= 0){
		// attempt to read *len many bytes
		// *len becomes actual number of bytes read
		*len = read(this->com, data, *len);
	}else{
		retval = -1;
	}

	return retval;
}

AnchorData::AnchorData(uint8_t* data){
	int ind = 0;
	this->id = data[ind++];
	this->timestamp = data[ind] | data[ind+1] << 8 | data[ind+2] << 16 | data[ind+3] << 24;
	ind += 4;
	memcpy(&this->x, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->y, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->z, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->rx_power, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->fp_power, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->fp_snr, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
}

AnchorData::AnchorData(const AnchorData &anchor){
    id = anchor.id;
    timestamp = anchor.timestamp;
    x = anchor.x;
    y = anchor.y;
    z = anchor.z;
    distance = anchor.distance;
    rx_power = anchor.rx_power;
    fp_power = anchor.fp_power;
    fp_snr = anchor.fp_snr;
}

void AnchorData::update_data(uint8_t* data){
	this->ros_timestamp = ros::Time::now();
	int ind = 0;
	this->id = data[ind++];
	this->timestamp = data[ind] | data[ind+1] << 8 | data[ind+2] << 16 | data[ind+3] << 24;
	ind += 4;
	memcpy(&this->x, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->y, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->z, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->rx_power, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->fp_power, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
	memcpy(&this->fp_snr, data + ind, FIELD_SIZE);
	ind += FIELD_SIZE;
}

std::string AnchorData::to_string(){
	char temp[256];
	sprintf(temp, "%d (%.2f, %.2f, %.2f) %.3fm %.1fdBm", this->id, this->x, this->y, this->z, this->distance, this->rx_power);
	std::string retval(temp);
	return retval;
}
