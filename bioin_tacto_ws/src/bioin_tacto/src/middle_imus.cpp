#include <ros/ros.h>
#include <bioin_tacto/raw_imu.h>
#include "bioin_tacto/raw_imu_serial.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#define NUM_SENSORS 1

bioin_tacto::raw_imu_serial tmm_imus;

ros::Publisher pub;

void imuReadCallback(const bioin_tacto::raw_imuConstPtr &msg) {
    if (msg->sensor_id < NUM_SENSORS) {
            tmm_imus.imus[msg->sensor_id].sensor_id = msg->sensor_id;
            tmm_imus.imus[msg->sensor_id].header = msg->header;
            tmm_imus.imus[msg->sensor_id].tempe = msg->tempe;
            tmm_imus.imus[msg->sensor_id].gx = msg->gx * 0.0174532925; //deg/sec to rad/sec
            tmm_imus.imus[msg->sensor_id].gy = msg->gy * 0.0174532925;
            tmm_imus.imus[msg->sensor_id].gz = msg->gz * 0.0174532925;
            tmm_imus.imus[msg->sensor_id].ax = msg->ax * 9.80665; //g to m/s2
            tmm_imus.imus[msg->sensor_id].ay = msg->ay * 9.80665;
            tmm_imus.imus[msg->sensor_id].az = msg->az * 9.80665;
            tmm_imus.imus[msg->sensor_id].mx = msg->mx;
            tmm_imus.imus[msg->sensor_id].my = msg->my;
            tmm_imus.imus[msg->sensor_id].mz = msg->mz;
        }

    pub.publish(tmm_imus);
}

int main(int argc, char **argv) {
    tmm_imus.imus.resize(NUM_SENSORS);

    ros::init(argc, argv, "middle_imus"); //
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("raw_imus_teensy", 10000,
                                      imuReadCallback); //
    pub = n.advertise<bioin_tacto::raw_imu_serial>(
            "m_imus_serial", 1000);

    ros::spin();

    return 0;
}