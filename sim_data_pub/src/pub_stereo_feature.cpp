#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>


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
            double  temp;
            ss>>p(0);
            ss>>p(1);
            ss>>p(2);
            ss>>temp;
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
 
    ros::init(argc, argv, "feature");
    ros::NodeHandle n;
    ros::Publisher pub_pobs = n.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 1000);

    ros::Rate loop_rate(30);
    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accs;
    LoadPose("/home/hyj/my_slam/vio_sim/vio_stereo_house/bin/cam_pose.txt",timestamp, gyros, accs);

    sleep(1);   //等vins代码启动
    //while (ros::ok())
    for (int i = 0; i < timestamp.size(); ++i)
    {

        stringstream ss, ss_r;
        ss << "/home/hyj/my_slam/vio_sim/vio_stereo_house/bin/keyframe/all_points_"<< i <<".txt";
        ss_r << "/home/hyj/my_slam/vio_sim/vio_stereo_house/bin/keyframe/all_points_r_"<< i <<".txt";
        std::vector<Eigen::Vector2d> pobs,pobs_r;
        LoadPointObs(ss.str(),pobs);
        LoadPointObs(ss_r.str(),pobs_r);

        // point feature
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_point;    //  u
        sensor_msgs::ChannelFloat32 v_of_point;    //  v

        ros::Time t(timestamp[i]);
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
            u_of_point.values.push_back((float)pobs_r[j].x());
            v_of_point.values.push_back((float)pobs_r[j].y());
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        //ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        pub_pobs.publish(feature_points);

        //done publishing
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
