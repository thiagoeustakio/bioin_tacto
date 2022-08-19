#include <ros/ros.h>
#include <bioin_tacto/raw_barometer.h>
#include "bioin_tacto/raw_barometer_serial.h"

#define NUM_SENSORS 1 //multiple sensing modules can have their data signals multiplexed 

bioin_tacto::raw_barometer_serial tmm_baros;

ros::Publisher pub;

void baroReadCallback(const bioin_tacto::raw_barometerConstPtr& msg) {

	if (msg->sensor_id < NUM_SENSORS){
		tmm_baros.baros[msg->sensor_id].sensor_id = msg->sensor_id;
		tmm_baros.baros[msg->sensor_id].header = msg->header;
		tmm_baros.baros[msg->sensor_id].baro_level = msg->baro_level;
		tmm_baros.baros[msg->sensor_id].tempe = msg->tempe;
	pub.publish(tmm_baros);
	}
}

int main(int argc, char** argv) {
	tmm_baros.baros.resize(NUM_SENSORS);

	ros::init(argc, argv, "middle_baros"); //
	ros::NodeHandle n; //
	ros::Subscriber sub = n.subscribe("raw_barometers_teensy", 10000, baroReadCallback); //
	pub = n.advertise<bioin_tacto::raw_barometer_serial>(
			"m_baros_serial", 1000); //
	ros::spin();

	return 0;
}
