#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <bioin_tacto/raw_barometer_serial.h>
#include <sensor_msgs/Imu.h>

#define NUM_SENSORS 16

double min_level[NUM_SENSORS];
double max_level[NUM_SENSORS];
double length = 1.0;
bioin_tacto::raw_barometer_serial tmm_baros;
sensor_msgs::Imu imu_data;
geometry_msgs::Quaternion quat1;

// double roll = 0, pitch = 0, yaw = 0.5 * M_PI;

double roll = 0, pitch = 0, yaw = 0;
tf::Quaternion quat;
tf::Quaternion quat2;

void baroReadSCallback(const bioin_tacto::raw_barometer_serialConstPtr &msg) {
    length = msg->baros[0].baro_level;

    if(min_level[0] < length)
        min_level[0] = length;
    if(max_level[0] > length)
        max_level[0] = length;

    length -= max_level[0]-1;

    //ROS_FATAL("%f, %f, %f", length, min_level[0], max_level[0]);
}

void imuReadSCallback(const sensor_msgs::ImuConstPtr &msg) {
    quat.setX(msg->orientation.x);
    quat.setY(msg->orientation.y);
    quat.setZ(msg->orientation.z);
    quat.setW(msg->orientation.w);

    quat = quat*quat2;
    //ROS_FATAL("%f, %f, %f", length, min_level[0], max_level[0]);
}

int main(int argc, char **argv) {

    for(int i=0; i<NUM_SENSORS; i++){
        min_level[i] = LDBL_MAX;
        max_level[i] = LDBL_MIN;
    }

    ros::init(argc, argv, "pressure_array");

    ros::NodeHandle n;
    ros::Rate r(100);

    ros::Subscriber sub = n.subscribe("m_baros_serial", 10000, baroReadSCallback); //
    ros::Subscriber sub2 = n.subscribe("imu/data", 10000, imuReadSCallback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("pressure_viz", 100);

    uint32_t shape = visualization_msgs::Marker::ARROW;

    quat2.setEuler(yaw, pitch, roll);
    quat2 = quat2.inverse();

   ROS_INFO("%lf,%lf,%lf,%lf", (double) quat.getX(), (double) quat.getY(), (double) quat.getZ(), (double) quat.getW());
    while (ros::ok()) {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "cell_0";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // marker.ns = "pressure_array";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = quat.getX();
        marker.pose.orientation.y = quat.getY();
        marker.pose.orientation.z = quat.getZ();
        marker.pose.orientation.w = quat.getW();


        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = (length / 100) / 2;
        marker.scale.y = 0.5 / 2;//*length / 100;
        marker.scale.z = 0.5 / 2;//*length / 100;

        // Set the color -- be sure to set alpha to something non-zero!
        //TODO: change color according to the temperature
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6;
        
        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);

        r.sleep();
        ros::spinOnce();
    }
}
