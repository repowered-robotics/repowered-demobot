#include <motor_ctrl.h>
#include <demobot.h>

int main(int argc, char** argv){
	ROS_INFO("Motor Control Node");
	ros::init(argc, argv, "motor_ctrl");
	ros::NodeHandle n;
	
	DemoBot bot; // initialize the robot, spi-bus, and whatever else

	// provide the set_motor service
	ros::ServiceServer set_motor_srv = n.advertiseService("set_motor", &DemoBot::set_motor_callback, &bot);
	while(ros::ok()){
		
		ROS_INFO("Timestamp %d", bot.get_timestamp());

		bot.set_motor_speed(1, 250);
		bot.set_motor_speed(2, 250);

		ros::Duration(1.0).sleep();
		
		ros::spinOnce();
	}
}
