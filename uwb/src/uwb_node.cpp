#include <uwb.h>

int main(int argc, char** argv){
	ROS_INFO("UWB Node");
	ros::init(argc, argv, "uwb");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	
	std::string tty_device = "/dev/ttyAMA0";
	
	UwbModule uwb_module(tty_device);

	uwb_module.read_config();

	ROS_INFO(uwb_module.config_to_string().c_str());

	while(ros::ok()){
		uwb_module.read_anchors();
		uwb_module.publish_anchor_data();
		ROS_INFO(uwb_module.anchors_to_string().c_str());
		loop_rate.sleep();
		ros::spinOnce();
	}
}
