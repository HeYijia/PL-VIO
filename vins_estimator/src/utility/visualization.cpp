#include "visualization.h"
#include "line_geometry.h"
#include "../feature_manager.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_loop_path;
ros::Publisher pub_point_cloud, pub_margin_cloud, pub_lines, pub_marg_lines;
ros::Publisher pub_key_poses;

ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual, pub_pose_graph;
nav_msgs::Path path, loop_path;
CameraPoseVisualization cameraposevisual(0, 0, 1, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path_no_loop", 1000);
    pub_loop_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
    pub_lines = n.advertise<visualization_msgs::Marker>("lines_cloud", 1000);
    pub_marg_lines = n.advertise<visualization_msgs::Marker>("history_lines_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<geometry_msgs::PoseStamped>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);

    cameraposevisual.setScale(0.2);
    cameraposevisual.setLineWidth(0.01);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    ROS_INFO_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = Quaterniond(estimator.Rs[WINDOW_SIZE]).x();
        odometry.pose.pose.orientation.y = Quaterniond(estimator.Rs[WINDOW_SIZE]).y();
        odometry.pose.pose.orientation.z = Quaterniond(estimator.Rs[WINDOW_SIZE]).z();
        odometry.pose.pose.orientation.w = Quaterniond(estimator.Rs[WINDOW_SIZE]).w();
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = loop_correct_r * estimator.Ps[WINDOW_SIZE] + loop_correct_t;
        correct_q = loop_correct_r * estimator.Rs[WINDOW_SIZE];
        correct_v = loop_correct_r * estimator.Vs[WINDOW_SIZE];
        odometry.pose.pose.position.x = correct_t.x();
        odometry.pose.pose.position.y = correct_t.y();
        odometry.pose.pose.position.z = correct_t.z();
        odometry.pose.pose.orientation.x = correct_q.x();
        odometry.pose.pose.orientation.y = correct_q.y();
        odometry.pose.pose.orientation.z = correct_q.z();
        odometry.pose.pose.orientation.w = correct_q.w();
        odometry.twist.twist.linear.x = correct_v(0);
        odometry.twist.twist.linear.y = correct_v(1);
        odometry.twist.twist.linear.z = correct_v(2);
        pub_odometry.publish(odometry);

        pose_stamped.pose = odometry.pose.pose;
        loop_path.header = header;
        loop_path.header.frame_id = "world";
        loop_path.poses.push_back(pose_stamped);
        pub_loop_path.publish(loop_path);

//        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << header.stamp.toSec() << " ";
        foutC.precision(5);
        foutC << correct_t.x() << " "
              << correct_t.y() << " "
              << correct_t.z() << " "
              << correct_q.x() << " "
              << correct_q.y() << " "
              << correct_q.z() << " "
              << correct_q.w() <<endl;
//              << correct_v(0) << ","
//              << correct_v(1) << ","
//              << correct_v(2) << "," << endl;
        foutC.close();

//        ofstream foutC(VINS_RESULT_PATH, ios::app);
//        Eigen::Matrix4d T;
//        T.block(0,0,3,3) = correct_q.toRotationMatrix();
//        T.block(0,3,3,1) = correct_t;
//        foutC.setf(ios::fixed, ios::floatfield);
//        foutC.precision(9);
//        foutC << header.stamp.toSec() << " ";
//        foutC.precision(5);
//        for (int i = 0; i < 3; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                foutC << T(i,j) << " ";
//            }
//        }
//        foutC << endl;
//        foutC.close();

    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = loop_correct_r * estimator.key_poses[i] + loop_correct_t;
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    int idx2 = WINDOW_SIZE - 1;
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header = header;
        camera_pose.header.frame_id = std::to_string(estimator.Headers[i].stamp.toNSec());
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);
        P = (loop_correct_r * estimator.Ps[i] + loop_correct_t) + (loop_correct_r * estimator.Rs[i]) * estimator.tic[0];
        R = Quaterniond((loop_correct_r * estimator.Rs[i]) * estimator.ric[0]);
        camera_pose.pose.position.x = P.x();
        camera_pose.pose.position.y = P.y();
        camera_pose.pose.position.z = P.z();
        camera_pose.pose.orientation.w = R.w();
        camera_pose.pose.orientation.x = R.x();
        camera_pose.pose.orientation.y = R.y();
        camera_pose.pose.orientation.z = R.z();

        pub_camera_pose.publish(camera_pose);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        camera_pose.header.frame_id = "world";
        cameraposevisual.publish_by(pub_camera_pose_visual, camera_pose.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;

        //std::cout<< "used num: " <<used_num<<" point id: "<<it_per_id.feature_id<<std::endl;

        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                              + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud, loop_margin_cloud;
    margin_cloud.header = header;
    loop_margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                             + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

visualization_msgs::Marker marg_lines_cloud;  // 全局变量用来保存所有的线段
std::list<visualization_msgs::Marker> marg_lines_cloud_last10frame;
void pubLinesCloud(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.header.frame_id = "world";
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    lines.id = 0; //key_poses_id++;
    lines.scale.x = 0.03;
    lines.scale.y = 0.03;
    lines.scale.z = 0.03;
    lines.color.b = 1.0;
    lines.color.a = 1.0;

    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
//        int used_num;
//        used_num = it_per_id.linefeature_per_frame.size();
//
//        if (!(used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))
//            continue;
//        //if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.is_triangulation == false)
            continue;

        //std::cout<< "used num: " <<used_num<<" line id: "<<it_per_id.feature_id<<std::endl;

        int imu_i = it_per_id.start_frame;

        Vector3d pc, nc, vc;
        // pc = it_per_id.line_plucker.head(3);
        // nc = pc.cross(vc);
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);
        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
        Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

//
//        if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//            continue;

        //std::cout <<"visual: "<< it_per_id.feature_id <<" " << it_per_id.line_plucker <<"\n\n";
        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));

        Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
        Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;


/*
        Vector3d diff_1 = it_per_id.ptw1 - w_pts_1;
        Vector3d diff_2 = it_per_id.ptw2 - w_pts_2;
        if(diff_1.norm() > 1 || diff_2.norm() > 1)
        {
            std::cout <<"visual: "<<it_per_id.removed_cnt<<" "<<it_per_id.all_obs_cnt<<" " << it_per_id.feature_id <<"\n";// << it_per_id.line_plucker <<"\n\n" << it_per_id.line_plk_init <<"\n\n";
            std::cout << it_per_id.Rj_ <<"\n" << it_per_id.tj_ <<"\n\n";
            std::cout << estimator.Rs[imu_i] <<"\n" << estimator.Ps[imu_i] <<"\n\n";
            std::cout << obs <<"\n\n" << it_per_id.obs_j<<"\n\n";

        }

        w_pts_1 = it_per_id.ptw1;
        w_pts_2 = it_per_id.ptw2;
*/
/*
        Vector3d w_pts_1 =  estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + estimator.Ps[imu_i];
        Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + estimator.Ps[imu_i];

        Vector3d d = w_pts_1 - w_pts_2;
        if(d.norm() > 4.0 || d.norm() < 2.0)
            continue;
*/
        geometry_msgs::Point p;
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        lines.points.push_back(p);
        p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        lines.points.push_back(p);

    }
    //std::cout<<" viewer lines.size: " <<lines.points.size() << std::endl;
    pub_lines.publish(lines);


//    visualization_msgs::Marker marg_lines_cloud_oneframe; // 最近一段时间的
//    marg_lines_cloud_oneframe.header = header;
//    marg_lines_cloud_oneframe.header.frame_id = "world";
//    marg_lines_cloud_oneframe.ns = "lines";
//    marg_lines_cloud_oneframe.type = visualization_msgs::Marker::LINE_LIST;
//    marg_lines_cloud_oneframe.action = visualization_msgs::Marker::ADD;
//    marg_lines_cloud_oneframe.pose.orientation.w = 1.0;
//    marg_lines_cloud_oneframe.lifetime = ros::Duration();
//
//    //marg_lines_cloud.id = 0; //key_poses_id++;
//    marg_lines_cloud_oneframe.scale.x = 0.05;
//    marg_lines_cloud_oneframe.scale.y = 0.05;
//    marg_lines_cloud_oneframe.scale.z = 0.05;
//    marg_lines_cloud_oneframe.color.g = 1.0;
//    marg_lines_cloud_oneframe.color.a = 1.0;

//////////////////////////////////////////////
    // all marglization line
    marg_lines_cloud.header = header;
    marg_lines_cloud.header.frame_id = "world";
    marg_lines_cloud.ns = "lines";
    marg_lines_cloud.type = visualization_msgs::Marker::LINE_LIST;
    marg_lines_cloud.action = visualization_msgs::Marker::ADD;
    marg_lines_cloud.pose.orientation.w = 1.0;
    marg_lines_cloud.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    //marg_lines_cloud.id = 0; //key_poses_id++;
    marg_lines_cloud.scale.x = 0.05;
    marg_lines_cloud.scale.y = 0.05;
    marg_lines_cloud.scale.z = 0.05;
    marg_lines_cloud.color.r = 1.0;
    marg_lines_cloud.color.a = 1.0;
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
//        int used_num;
//        used_num = it_per_id.linefeature_per_frame.size();
//        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
//            continue;
//        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
//        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.linefeature_per_frame.size() <= 2
            && it_per_id.is_triangulation == true )
        {
            int imu_i = it_per_id.start_frame;

            Vector3d pc, nc, vc;
            // pc = it_per_id.line_plucker.head(3);
            // nc = pc.cross(vc);
            nc = it_per_id.line_plucker.head(3);
            vc = it_per_id.line_plucker.tail(3);
            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
            Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
            Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            Vector3d cam = Vector3d( 0, 0, 0 );

            Vector4d pi1 = pi_from_ppp(cam, p11, p12);
            Vector4d pi2 = pi_from_ppp(cam, p21, p22);

            Vector4d e1 = Lc * pi1;
            Vector4d e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

//            if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//                continue;
//
            double length = (e1-e2).norm();
            if(length > 10) continue;

            //std::cout << e1 <<"\n\n";
            Vector3d pts_1(e1(0),e1(1),e1(2));
            Vector3d pts_2(e2(0),e2(1),e2(2));

            Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                               + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
            Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                               + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

            //w_pts_1 = it_per_id.ptw1;
            //w_pts_2 = it_per_id.ptw2;

            geometry_msgs::Point p;
            p.x = w_pts_1(0);
            p.y = w_pts_1(1);
            p.z = w_pts_1(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
            p.x = w_pts_2(0);
            p.y = w_pts_2(1);
            p.z = w_pts_2(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
        }
    }
//    if(marg_lines_cloud_oneframe.points.size() > 0)
//        marg_lines_cloud_last10frame.push_back(marg_lines_cloud_oneframe);
//
//    if(marg_lines_cloud_last10frame.size() > 50)
//        marg_lines_cloud_last10frame.pop_front();
//
//    marg_lines_cloud.points.clear();
//    list<visualization_msgs::Marker>::iterator itor;
//    itor = marg_lines_cloud_last10frame.begin();
//    while(itor != marg_lines_cloud_last10frame.end())
//    {
//        for (int i = 0; i < itor->points.size(); ++i) {
//            marg_lines_cloud.points.push_back(itor->points.at(i));
//        }
//        itor++;
//    }

//    ofstream foutC("/home/hyj/catkin_ws/src/VINS-Mono/config/euroc/landmark.txt");
//    for (int i = 0; i < marg_lines_cloud.points.size();) {
//
//        geometry_msgs::Point pt1 = marg_lines_cloud.points.at(i);
//        geometry_msgs::Point pt2 = marg_lines_cloud.points.at(i+1);
//        i = i + 2;
//        foutC << pt1.x << " "
//              << pt1.y << " "
//              << pt1.z << " "
//              << pt2.x << " "
//              << pt2.y << " "
//              << pt2.z << "\n";
//    }
//    foutC.close();
    pub_marg_lines.publish(marg_lines_cloud);

}


void pubPoseGraph(CameraPoseVisualization* posegraph, const std_msgs::Header &header)
{
    posegraph->publish_by(pub_pose_graph, header);
    
}

void updateLoopPath(nav_msgs::Path _loop_path)
{
    loop_path = _loop_path;
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = loop_correct_r * estimator.Ps[WINDOW_SIZE] + loop_correct_t;
    correct_q = loop_correct_r * estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));
}
