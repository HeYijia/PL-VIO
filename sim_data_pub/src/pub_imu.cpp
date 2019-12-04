#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
using namespace std;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


void LoadPose(std::string filename, std::vector<double>& timestamp,std::vector<Eigen::Vector3d>& gyros, std::vector<Eigen::Vector3d>& accs)
{

    std::ifstream f;
    f.open(filename.c_str());

    if(!f.is_open())
    {
        std::cerr << " can't open LoadFeatures file "<<std::endl;
        return;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f,s);

        if(! s.empty())
        {
            std::stringstream ss;
            ss << s;

            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss>>time;
            ss>>q.w();
            ss>>q.x();
            ss>>q.y();
            ss>>q.z();
            ss>>t(0);
            ss>>t(1);
            ss>>t(2);
            ss>>gyro(0);
            ss>>gyro(1);
            ss>>gyro(2);
            ss>>acc(0);
            ss>>acc(1);
            ss>>acc(2);

            timestamp.push_back(time);
            gyros.push_back(gyro);
            accs.push_back(acc);
        }
    }

}

int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "imu");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    ros::Publisher out_pub = n.advertise<sensor_msgs::Imu >("/imu0", 1000);
    ros::Rate loop_rate(200);
    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accs;

    std::string sim_file;
    if (!n.getParam("sim_file_path", sim_file))
    {
        sim_file = std::string("/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/");
    }
    ROS_INFO_STREAM("Loaded " << "sim_file_path" << ": " << sim_file);

    LoadPose(sim_file + "imu_pose.txt",timestamp, gyros, accs);

    sleep(1);
    //while (ros::ok())
    for (int i = 0; i < timestamp.size(); ++i)
    {
        //new imu message
        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();

        ros::Time t(timestamp[i]);
        imu_msg.header.stamp = t;
        imu_msg.header.frame_id = "imu0";

        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1.0;
        //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        Eigen::Vector3d gyro = gyros[i];
        Eigen::Vector3d acc = accs[i];
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];
        //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        //publish the new imu message
        out_pub.publish(imu_msg);
        ROS_INFO("%s", "send an imu message");

        //done publishing
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
