#include <ros/ros.h>
#include <bioin_tacto/bias_srv.h>
#include <bioin_tacto/raw_imu_serial.h>
#include <ros/callback_queue.h>

#define NUM_SENSORS 1 //multiple sensing modules can have their data signals multiplexed 

ros::CallbackQueue quimusC, qusrv;
ros::SubscribeOptions ops_sub;
ros::AdvertiseServiceOptions ops_serv;

bioin_tacto::bias_srv::Response r;
bool calculating_bias = false;
long n_samples = 0;

void imuBiasCallback(const bioin_tacto::raw_imu_serialConstPtr &msg) {
    if (calculating_bias) {
        for(int i=0; i<NUM_SENSORS; i++){
            r.b_ax[i] += msg->imus[i].ax;
            r.b_ay[i] += msg->imus[i].ay;
            r.b_az[i] += msg->imus[i].az;
            r.b_gx[i] += msg->imus[i].gx;
            r.b_gy[i] += msg->imus[i].gy;
            r.b_gz[i] += msg->imus[i].gz;
        }
        n_samples++;
    }
}

bool add(bioin_tacto::bias_srv::Request &req,
         bioin_tacto::bias_srv::Response &res) {
    calculating_bias = true;
    if (req.num_secs > 0) ros::Duration((double) req.num_secs).sleep();
    calculating_bias = false;

    for(int i=0; i<NUM_SENSORS; i++) {
        r.b_ax[i] /= n_samples;
        r.b_ay[i] /= n_samples;
        r.b_az[i] /= n_samples;
        r.b_gx[i] /= n_samples;
        r.b_gy[i] /= n_samples;
        r.b_gz[i] /= n_samples;
    }

    res = r;
    n_samples = 0;
    return true;
}

int main(int argc, char **argv) {

    r.b_ax.resize(NUM_SENSORS, 0.0);
    r.b_ay.resize(NUM_SENSORS, 0.0);
    r.b_az.resize(NUM_SENSORS, 0.0);
    r.b_gx.resize(NUM_SENSORS, 0.0);
    r.b_gy.resize(NUM_SENSORS, 0.0);
    r.b_gz.resize(NUM_SENSORS, 0.0);

    ros::init(argc, argv, "middle_bias_imus");
    ros::NodeHandle n;

    ops_sub = ros::SubscribeOptions::create<bioin_tacto::raw_imu_serial>(
            "/m_imus_serial", // topic name
            10000, // queue length
            imuBiasCallback, // callback
            ros::VoidPtr(), // tracked object, we don't need one thus NULL
            &quimusC // pointer to callback queue object
    );
    ros::Subscriber sub = n.subscribe(ops_sub);
    ros::AsyncSpinner spinner(1, &quimusC);
    spinner.start();

    ops_serv = ros::AdvertiseServiceOptions::create<bioin_tacto::bias_srv>("/bias_imu_srv", add, ros::VoidPtr(),
                                                                                  &qusrv);
    ros::ServiceServer service = n.advertiseService(ops_serv);
    ros::AsyncSpinner spinner2(1,&qusrv);
    spinner2.start();

    ROS_INFO("Bias estimation.");
    ros::spin();

    return 0;
}