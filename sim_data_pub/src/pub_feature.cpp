#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>


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
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    ros::Publisher pub_pobs = n.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 1000);
    ros::Publisher pub_lobs = n.advertise<sensor_msgs::PointCloud>("/linefeature_tracker/linefeature", 1000);

    ros::Rate loop_rate(30);
    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accs;

    std::string sim_file;
    if (!n.getParam("sim_file_path", sim_file))
    {
        sim_file = std::string("/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/");
    }
    ROS_INFO_STREAM("Loaded " << "sim_file_path" << ": " << sim_file);

    LoadPose(sim_file+"cam_pose.txt",timestamp, gyros, accs);

    sleep(1);   //等vins代码启动
    ROS_INFO("START PUB POINTS AND LINES FEATRUE");
    //while (ros::ok())
    for (int i = 0; i < timestamp.size(); ++i)
    {

        stringstream ss;
        ss <<sim_file<< "keyframe/all_points_"<< i <<".txt";
        std::vector<Eigen::Vector2d> pobs;
        LoadPointObs(ss.str(),pobs);

        stringstream ss1;
        ss1 <<sim_file<< "keyframe/all_lines_"<< i <<".txt";
        std::vector<Eigen::Vector4d> lobs;
        LoadLineObs(ss1.str(),lobs);

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
            u_of_point.values.push_back((float)pobs[j].x());
            v_of_point.values.push_back((float)pobs[j].y());
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        //ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
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
        pub_lobs.publish(feature_lines);

        ROS_INFO("publish Points and lines timestamp: %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
        //done publishing
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
