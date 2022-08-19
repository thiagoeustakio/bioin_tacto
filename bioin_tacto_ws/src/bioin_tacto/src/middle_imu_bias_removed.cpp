#include <ros/ros.h>
#include <bioin_tacto/bias_srv.h>
#include <bioin_tacto/raw_imu_serial.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#define NUM_SENSORS 1

bioin_tacto::bias_srv srv;
bioin_tacto::bias_srv::Response r;

bioin_tacto::raw_imu_serial tmm_imus;

sensor_msgs::Imu imu_data_raw;
sensor_msgs::MagneticField mag_field;

ros::Publisher pub;
ros::Publisher pub_imu_r;
ros::Publisher pub_mag_r;

void imuReadCallback(const bioin_tacto::raw_imu_serialConstPtr &msg) {
    tmm_imus.imus[0].sensor_id = msg->imus[0].sensor_id;
    mag_field.header = imu_data_raw.header = tmm_imus.imus[0].header = msg->imus[0].header;
    tmm_imus.imus[0].tempe = msg->imus[0].tempe;
    imu_data_raw.angular_velocity.x = tmm_imus.imus[0].gx = msg->imus[0].gx - srv.response.b_gx[0]; //deg/sec to rad/sec
    imu_data_raw.angular_velocity.y = tmm_imus.imus[0].gy = msg->imus[0].gy - srv.response.b_gy[0];
    imu_data_raw.angular_velocity.z = tmm_imus.imus[0].gz = msg->imus[0].gz - srv.response.b_gz[0];
    imu_data_raw.linear_acceleration.x = tmm_imus.imus[0].ax = msg->imus[0].ax;// - srv.response.b_ax[0]; //g to m/s2
    imu_data_raw.linear_acceleration.y = tmm_imus.imus[0].ay = msg->imus[0].ay;// - srv.response.b_ay[0];
    imu_data_raw.linear_acceleration.z = tmm_imus.imus[0].az = msg->imus[0].az;// - (9.80665 + srv.response.b_az[0]);
    mag_field.magnetic_field.x = tmm_imus.imus[0].mx =
            msg->imus[0].mx;
    mag_field.magnetic_field.y = tmm_imus.imus[0].my = msg->imus[0].my;
    mag_field.magnetic_field.z = tmm_imus.imus[0].mz = msg->imus[0].mz;
    pub_imu_r.publish(imu_data_raw);
    pub_mag_r.publish(mag_field);

    for (int i = 1; i < NUM_SENSORS; ++i) {
        tmm_imus.imus[i].sensor_id = msg->imus[i].sensor_id;
        tmm_imus.imus[i].header = msg->imus[i].header;
        tmm_imus.imus[i].tempe = msg->imus[i].tempe;
        tmm_imus.imus[i].gx = msg->imus[i].gx; //deg/sec to rad/sec
        tmm_imus.imus[i].gy = msg->imus[i].gy;
        tmm_imus.imus[i].gz = msg->imus[i].gz;
        tmm_imus.imus[i].ax = msg->imus[i].ax; //g to m/s2
        tmm_imus.imus[i].ay = msg->imus[i].ay;
        tmm_imus.imus[i].az = msg->imus[i].az;
        tmm_imus.imus[i].mx = msg->imus[i].mx;
        tmm_imus.imus[i].my = msg->imus[i].my;
        tmm_imus.imus[i].mz = msg->imus[i].mz;
    }
    pub_imu_r.publish(imu_data_raw);
    pub_mag_r.publish(mag_field);
    pub.publish(tmm_imus);
}

int main(int argc, char **argv) {
    tmm_imus.imus.resize(NUM_SENSORS);

    r.b_ax.resize(NUM_SENSORS, 0.0);
    r.b_ay.resize(NUM_SENSORS, 0.0);
    r.b_az.resize(NUM_SENSORS, 0.0);
    r.b_gx.resize(NUM_SENSORS, 0.0);
    r.b_gy.resize(NUM_SENSORS, 0.0);
    r.b_gz.resize(NUM_SENSORS, 0.0);

    ros::init(argc, argv, "middle_imus_bias_removed");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("m_imus_serial", 10000,
                                      imuReadCallback);

    pub = n.advertise<bioin_tacto::raw_imu_serial>(
            "m_imus_bias_removed", 1000);

    pub_imu_r = n.advertise<sensor_msgs::Imu>(
            "imu/data_raw", 1000);
    pub_mag_r = n.advertise<sensor_msgs::MagneticField>(
            "imu/mag", 1000);

    ros::ServiceClient client = n.serviceClient<bioin_tacto::bias_srv>("/bias_imu_srv");

    srv.request.num_secs = 10;

    if (client.call(srv)) {
        ROS_INFO("Bias:");
        ROS_INFO("%f", srv.response.b_ax[0]);
        ROS_INFO("%f", srv.response.b_ay[0]);
        ROS_INFO("%f", srv.response.b_az[0]);
        ROS_INFO("%f", srv.response.b_gx[0]);
        ROS_INFO("%f", srv.response.b_gy[0]);
        ROS_INFO("%f", srv.response.b_gz[0]);
    } else {
        ROS_FATAL("Bias not calculated!!!");
        return 1;
    }

    mag_field.header.frame_id = "mag_field";
    imu_data_raw.header.frame_id = "imu_frame";

    ros::spin();

    return 0;
}