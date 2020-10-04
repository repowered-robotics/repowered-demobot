#include <demobot.h>

int main(int argc, char** argv){
	ROS_INFO("Motor Control Node");
	ros::init(argc, argv, "motor_ctrl");
	ros::NodeHandle n;
	
	DemoBot bot(n, "/dev/spidev0.0"); // initialize the robot, spi-bus, and whatever else

	ROS_INFO("Starting main loop...");
	while(ros::ok()){
		ros::Duration(0.1).sleep();
		bot.send_motor_updates();
		ros::spinOnce();
	}
	ROS_INFO("Exiting...");
	bot.cleanup();
	return 0;
}
