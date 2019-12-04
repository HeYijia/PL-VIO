#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
using namespace std;

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

void LoadPointObs(std::string filename, std::vector<Eigen::Vector2d>& obs)
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

            Eigen::Vector3d p;
            Eigen::Vector2d ob;

            ss>>p(0);
            ss>>p(1);
            ss>>p(2);
            ss>>ob(0);
            ss>>ob(1);

            obs.push_back(ob);

        }
    }

}

void LoadLineObs(std::string filename, std::vector<Eigen::Vector4d>& obs)
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

            Eigen::Vector4d ob;
            ss>>ob(0);
            ss>>ob(1);
            ss>>ob(2);
            ss>>ob(3);

            obs.push_back(ob);

        }
    }

}

int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "imu_features");
    ros::NodeHandle n;
    ros::Publisher out_pub = n.advertise<sensor_msgs::Imu >("/imu0", 1000);
    ros::Publisher pub_pobs = n.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 1000);
    ros::Publisher pub_lobs = n.advertise<sensor_msgs::PointCloud>("/linefeature_tracker/linefeature", 1000);

    ros::Rate loop_rate(100);

    std::vector<double> imutimestamp, camtimestamp;
    std::vector<Eigen::Vector3d> gyros, temp1;
    std::vector<Eigen::Vector3d> accs, temp2;
    LoadPose("/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/imu_pose.txt",imutimestamp, gyros, accs);
    LoadPose("/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/cam_pose.txt",camtimestamp, temp1, temp2);

    //while (ros::ok())
    int imu_pub_cnt = 0;
    int cam_time_index = 0;
    for (int i = 0; i < imutimestamp.size(); ++i)
    {
        //new imu message
        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();

        ros::Time t(imutimestamp[i]);
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

        if(imu_pub_cnt % 5 == 0)
        {
            stringstream ss;
            ss << "/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/keyframe/all_points_"<< cam_time_index <<".txt";
            std::vector<Eigen::Vector2d> pobs;
            LoadPointObs(ss.str(),pobs);

            stringstream ss1;
            ss1 << "/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/keyframe/all_lines_"<< cam_time_index <<".txt";
            std::vector<Eigen::Vector4d> lobs;
            LoadLineObs(ss1.str(),lobs);

            // point feature
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;   //  feature id
            sensor_msgs::ChannelFloat32 u_of_point;    //  u
            sensor_msgs::ChannelFloat32 v_of_point;    //  v

            ros::Time t(camtimestamp[cam_time_index]);
            feature_points->header.stamp = t;
            feature_points->header.frame_id = "world";

            for (int j = 0; j < pobs.size(); ++j) {
                int p_id = j;
                geometry_msgs::Point32 p;
                p.x = pobs[j].x();
                p.y = pobs[j].y();
                p.z = 1;

                feature_points->points.push_back(p);
                id_of_point.values.push_back(p_id * 1.0);
                u_of_point.values.push_back((float)pobs[j].x());
                v_of_point.values.push_back((float)pobs[j].y());
            }
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            ROS_INFO("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
            pub_pobs.publish(feature_points);

            // line feature
            sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
            sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
            sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v

            feature_lines->header = feature_points->header;
            feature_lines->header.frame_id = "world";

            for (unsigned int j = 0; j < lobs.size(); j++)
            {

                int p_id = j;
                geometry_msgs::Point32 p;
                p.x = lobs[j](0);
                p.y = lobs[j](1);
                p.z = 1;

                feature_lines->points.push_back(p);
                id_of_line.values.push_back((float)p_id);
                u_of_endpoint.values.push_back(lobs[j](2));
                v_of_endpoint.values.push_back(lobs[j](3));
                //ROS_ASSERT(inBorder(cur_pts[j]));
            }
            feature_lines->channels.push_back(id_of_line);
            feature_lines->channels.push_back(u_of_endpoint);
            feature_lines->channels.push_back(v_of_endpoint);
//        ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
            pub_lobs.publish(feature_lines);

            cam_time_index++;

        }
        imu_pub_cnt++;



        //done publishing
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
