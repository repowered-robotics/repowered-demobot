#include <lsm303a.h>

int main(int argc, char** argv){
	ROS_INFO("IMU Node");
	ros::init(argc, argv, "imu");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);
  Imu imu;

	while(ros::ok()){
		imu.read_all_params();
		imu.publish_data();

		loop_rate.sleep();

		ros::spinOnce();
	}
}
