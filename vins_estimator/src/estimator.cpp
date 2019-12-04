#include "estimator.h"
//#define LINEINCAM
Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
    failure_occur = 0;
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info =  FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    lineProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
//    lineProjectionFactor::sqrt_info =  Matrix2d::Identity();

    baseline_ = BASE_LINE;
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    relocalize = false;
    retrive_data_vector.clear();
    relocalize_t = Eigen::Vector3d(0, 0, 0);
    relocalize_r = Eigen::Matrix3d::Identity();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    // 如果 预积分数组pre_integrations 里还没有指向第frame_count帧的预积分量，那就创建
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; // acc_0, gyr_0 上一时刻的imu测量值
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);   // 它会记录滑动窗口里两个关键帧之间的预积分量
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);         // 在每一个新的图像帧上，会创建这个东西。它只记录两个图像之间的预积分量

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // 用imu预测这个时刻的状态量p,q,v. 这些状态量会在swf里优化
        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();              // R_{wi+1} = R_{wi} * R_{ii+1}
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    //if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
    if(f_manager.addFeatureCheckParallax(frame_count, image, lines))       // 对检测到的特征进行存放处理
        marginalization_flag = MARGIN_OLD;
    else                                                                   // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();          // 三角化新特征 并 swf优化
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];

            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();      // 三角化新特征 并 swf优化
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const map<int, vector<pair<int, Vector8d>>> &lines, const std_msgs::Header &header)
{

    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    //if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
    if(f_manager.addFeatureCheckParallax(frame_count, image, lines))       // 对检测到的特征进行存放处理
        marginalization_flag = MARGIN_OLD;
    else                                                                   // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();          // 三角化新特征 并 swf优化
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];

            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();      // 三角化新特征 并 swf优化
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
        marginalization_flag = MARGIN_OLD;
    else                                                                 // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
               result = initialStructure();
               initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();          // 三角化新特征 并 swf优化
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();      // 三角化新特征 并 swf优化
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            for (auto &i_p : id_pts.second)
            {
                //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            //cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_WARN("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        //std::cout<<corres.size()<<"\n";
        if (corres.size() > 20)  // 20
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
//            std::cout << "average_parallax: "<<average_parallax * 460 << std::endl;
//            if(average_parallax * 460 > 15 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) // 30
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);

        if(baseline_ > 0) //stereo
            f_manager.triangulateLine(baseline_);
        else
            f_manager.triangulateLine(Ps, tic, ric);

        ROS_DEBUG("triangulation costs %f", t_tri.toc());

        // optimization();

        onlyLineOpt();   // 三角化以后，优化一把
        optimizationwithLine();

#ifdef LINEINCAM
        LineBAincamera();
#else
//        LineBA();
#endif
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

#ifdef LINEINCAM
    MatrixXd lineorth = f_manager.getLineOrthVectorInCamera();
#else
    MatrixXd lineorth = f_manager.getLineOrthVector(Ps,tic,ric);
#endif

    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i) {
        para_LineFeature[i][0] = lineorth.row(i)[0];
        para_LineFeature[i][1] = lineorth.row(i)[1];
        para_LineFeature[i][2] = lineorth.row(i)[2];
        para_LineFeature[i][3] = lineorth.row(i)[3];
        if(i > NUM_OF_F)
            std::cerr << " 1000  1000 1000 1000 1000 \n\n";
    }

}

void Estimator::double2vector()
{
    // 相机姿态需要变化考虑优化以后，把yaw量旋转回去
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);    //优化之前的0th的姿态
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    // 优化以后的0th的姿态
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());

    // 优化前后，yaw的变化                                                      
    double y_diff = origin_R0.x() - origin_R00.x();    
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    // 由于VI系统的（绝对位置x,y,z,以及yaw）是不可观的。而优化过程中没有固定yaw角，因此yaw会朝着使得误差函数最小的方向优化，但这不一定是正确的。
    // 所以这里把 yaw角的变化量给旋转回去。
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // Position 也转移到yaw角优化前的 0th坐在的世界坐标下
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // 跟yaw没关系，所以不用管优化前后yaw的变化
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }


    //std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(),4);
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i) {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);
        lineorth_vec.row(i) = orth;

    }
#ifdef LINEINCAM
    f_manager.setLineOrthInCamera(lineorth_vec);
#else
    f_manager.setLineOrth(lineorth_vec,Ps,Rs,tic,ric);
#endif

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

}


void Estimator::double2vector2()
{
    // 六自由度优化的时候，整个窗口会在空间中任意优化，这时候我们需要把第一帧在yaw,position上的增量给去掉，因为vins在这几个方向上不可观，他们优化的增量也不可信。
    // 所以这里的操作过程就相当于是 fix 第一帧的 yaw 和 postion, 使得整个轨迹不会在空间中任意飘。
    // 相机姿态需要变化考虑优化以后，把yaw量旋转回去
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);    //优化之前的0th的姿态
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    // 优化以后的0th的姿态
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());

    // 优化前后，yaw的变化
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    // 由于VI系统的（绝对位置x,y,z,以及yaw）是不可观的。而优化过程中没有固定yaw角，因此yaw会朝着使得误差函数最小的方向优化，但这不一定是正确的。
    // 所以这里把 yaw角的变化量给旋转回去。
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // Position 也转移到yaw角优化前的 0th坐在的世界坐标下
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // 跟yaw没关系，所以不用管优化前后yaw的变化
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }


    // 先把line旋转到相机坐标系下
    Matrix3d Rwow1 = rot_diff;
    Vector3d tw1b(para_Pose[0][0],para_Pose[0][1],para_Pose[0][2]);
    Vector3d twow1 = -Rwow1 * tw1b + origin_P0;

    //std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(),4);;
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i) {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);

      // 将line_w优化以后的角度变化yaw的变化旋转回去
        Vector6d line_w1 = orth_to_plk(orth);
        Vector6d line_wo = plk_to_pose(line_w1, Rwow1,twow1);
        orth = plk_to_orth(line_wo);

        lineorth_vec.row(i) = orth;

    }
    f_manager.setLineOrth(lineorth_vec,Ps,Rs,tic,ric);

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        return true;
    }
    return false;
}

void  Estimator::onlyLineOpt()
{
    //固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
        /*
        std::cout << para_LineFeature[feature_index][0] <<" "
                << para_LineFeature[feature_index][1] <<" "
                << para_LineFeature[feature_index][2] <<" "
                << para_LineFeature[feature_index][3] <<"\n";
        */
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[feature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                //continue;
            }
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[feature_index]);
            f_m_cnt++;
        }
    }

    if(feature_index < 3)
    {
        return;
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    //std::cout <<"!!!!!!!!!!!!!onlyLineOpt!!!!!!!!!!!!!\n";
    double2vector();
    //std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric);


}


void  Estimator::LineBA()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        //problem.SetParameterBlockConstant(para_Pose[i]);
    }
    //  fix the first camera pose in window
    // problem.SetParameterBlockConstant( para_Pose[0] );
    //problem.SetParameterBlockConstant( para_Pose[1] );
    //problem.SetParameterBlockConstant( para_Pose[2] );

    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去
    //std::cout << Ps[9] <<"\n" << Ps[10]<<"\n";

    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }


//     所有特征
//    int f_m_cnt = 0;
//    int feature_index = -1;
//    for (auto &it_per_id : f_manager.feature)
//    {
//        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
//        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
//            continue;
//
//        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
//
//        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
//
//        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标
//
//        for (auto &it_per_frame : it_per_id.feature_per_frame)
//        {
//            imu_j++;
//            if (imu_i == imu_j)
//            {
//                continue;
//            }
//            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
//            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
//            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
//            f_m_cnt++;
//        }
//    }

/////////////////////////////////////
    // Line feature
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                //continue;
            }
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[linefeature_index]);
            line_m_cnt++;
        }
    }

    std::cout << "------------ linefeature_index : " << linefeature_index <<"\n";
    if(linefeature_index < 3)
    {
        // return;
    }

////////////////////////////////////////

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    double2vector2();
    //std::cout << Ps[9] <<"\n" << Ps[10]<<"\n";
    std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric);


}


void  Estimator::LineBAincamera()
{
    //固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        //problem.SetParameterBlockConstant(para_Pose[i]);
    }
    //  fix the first camera pose in window
    //problem.SetParameterBlockConstant( para_Pose[0] );
    //problem.SetParameterBlockConstant( para_Pose[1] );
    //problem.SetParameterBlockConstant( para_Pose[2] );

    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去
    //std::cout << Ps[9] <<"\n" << Ps[10]<<"\n";
    //std::cout<<"11111111111\n";
    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }


    // 所有特征
    int f_m_cnt = 0;
//    int feature_index = -1;
//    for (auto &it_per_id : f_manager.feature)
//    {
//        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
//        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
//            continue;
//
//        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
//
//        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
//
//        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标
//        for (auto &it_per_frame : it_per_id.feature_per_frame)
//        {
//            imu_j++;
//            if (imu_i == imu_j)
//            {
//                continue;
//            }
//            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
//            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
//            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
//            f_m_cnt++;
//        }
//    }

/////////////////////////////////////
    // Line feature
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            if (imu_i == imu_j)
            {
                lineProjectionFactor_instartframe *f = new lineProjectionFactor_instartframe(obs);     // 特征重投影误差
                problem.AddResidualBlock(f, loss_function,
                                         para_LineFeature[linefeature_index]);
            } else
            {
                lineProjectionFactor_incamera *f = new lineProjectionFactor_incamera(obs);     // 特征重投影误差
                problem.AddResidualBlock(f, loss_function,
                                         para_Pose[imu_i],             // 特征都是第i帧初始化的
                                         para_Pose[imu_j],
                                         para_Ex_Pose[0],
                                         para_LineFeature[linefeature_index]);

            }
            line_m_cnt++;
        }
    }

    std::cout << "------------ linefeature_index : " << linefeature_index <<"\n";
    if(linefeature_index < 3)
    {
        //    return;
    }
////////////////////////////////////////

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    double2vector();
  //  std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric);


}

//#define DebugFactor
void Estimator::optimizationwithLine()
{
    frame_cnt_ ++;
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q,v,ba,bg加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);                // v,ba,bg
    }
    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    TicToc t_whole, t_prepare;    // 统计程序运行时间
    TicToc t_solver;
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去

#ifdef DebugFactor
    /* Debug: 监视 prior factor*/
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    int marg_residual_size=0;
    std::vector<ceres::ResidualBlockId> imufactor_residual_block_ids;
    //end debud
#endif
    // 滑动窗口marg以后，上一次的prior factor
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ceres::ResidualBlockId block_id = problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
#ifdef DebugFactor
        marg_residual_size = marginalization_factor->num_residuals();  // used to debug
        residual_block_ids.push_back(block_id);           // used to debug
#endif
    }

    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        ceres::ResidualBlockId block_id = problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
#ifdef DebugFactor
        imufactor_residual_block_ids.push_back(block_id);
#endif
    }

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            f_m_cnt++;
        }
    }
/////////////////////////////////////
    // Line feature
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                //continue;
            }
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[linefeature_index]);
            line_m_cnt++;
        }
    }
    ROS_INFO("lineFactor: %d, pointFactor:%d", line_m_cnt, f_m_cnt);

    // if(line_m_cnt > 20)
    // {
    //     double scale = std::min(f_m_cnt /(2. * line_m_cnt), 10.);
    //     lineProjectionFactor::sqrt_info =  scale * FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    //     std::cout << "========== Line factor weight ========== \n" << scale  << std::endl; 
    // }

////////////////////////////////////////

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;

//    if (marginalization_flag == MARGIN_OLD)
//        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//    else
//        options.max_solver_time_in_seconds = SOLVER_TIME;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //cout << summary.BriefReport() << endl;
//    ROS_INFO("Points Lines Iterations : %d", static_cast<int>(summary.iterations.size()));
    sum_solver_time_ += t_solver.toc();
    mean_solver_time_ = sum_solver_time_/frame_cnt_;
    ROS_INFO("Points Lines solver costs: %f", mean_solver_time_);


#ifdef DebugFactor
    //----------- debug --------------------
    // marg factor
    Eigen::VectorXd err(marg_residual_size);
    err.setZero();
    ceres::Problem::EvaluateOptions opts;
    opts.residual_blocks = residual_block_ids;
    double total_cost = 0.0;
    vector<double> residuals;
    problem.Evaluate(opts,&total_cost, &residuals, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        //std::cout << residuals[i]<<std::endl;
        err[i] = residuals[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-before.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<" error befor: " << err.squaredNorm()<<" " << total_cost <<std::endl;

    // imu factor
    ceres::Problem::EvaluateOptions imufactor_opts;
    imufactor_opts.residual_blocks = imufactor_residual_block_ids;
    double total_cost_imufactor = 0.0;
    vector<double> residuals_imufactor;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor, &residuals_imufactor, nullptr, nullptr);
    Eigen::VectorXd imufactor_err(residuals_imufactor.size());
    imufactor_err.setZero();
    for(int i=0; i< residuals_imufactor.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err[i] = residuals_imufactor[i];
    }
    std::cout<<" IMU error befor: " << imufactor_err.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    // --------------------  end debug -------------------------
#endif

    //double2vector();

    double2vector2();   // Line pose change
    TicToc t_culling;
    f_manager.removeLineOutlier(Ps,tic,ric);   // remove Line outlier
    ROS_INFO("culling line feautre: %f ms", t_culling.toc());

#ifdef DebugFactor
    // ----------------  debug  ----------------------
    vector2double();
    Eigen::VectorXd err2(marg_residual_size);
    err2.setZero();
    vector<double> residuals2;
    problem.Evaluate(opts,&total_cost, &residuals2, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        err[i] = residuals2[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-after.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<"error after: " << err.squaredNorm()<<" "<< total_cost <<std::endl;
    // imu factor
    double total_cost_imufactor2 = 0.0;
    vector<double> residuals_imufactor2;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor2, &residuals_imufactor2, nullptr, nullptr);
    Eigen::VectorXd imufactor_err2(residuals_imufactor2.size());
    imufactor_err2.setZero();
    for(int i=0; i< residuals_imufactor2.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err2[i] = residuals_imufactor2[i];
    }
    std::cout<<" IMU error after: " << imufactor_err2.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    //------------------- end debug  --------------------------------
#endif

    // 将优化以后要marg掉的部分转为prior factor
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        // 构建一个新的 prior info
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        /*
           将最老帧上约束转变为 prior, 那有哪些约束是跟这个最老的帧相关的呢？
           1. 上一次优化以后留下的 prior 里可能存在 
           2. 跟最老帧 存在 预积分imu 约束
           3. 最老帧上有很多特征观测约束
        */
        // 1. 上一次优化以后留下的 prior 里可能存在约束
        if (last_marginalization_info)     
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])    // 最老的一帧给丢掉
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // 2. 最老的两帧之间的 预积分 factor
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});  // vector<int>{0, 1} 表示要marg的参数下表，比如这里对应para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 3. 最老帧上有很多特征观测约束
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)    // 遍历所有特征
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)             // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)   //遍历这个特征的所有观测
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});// vector<int>{0, 3} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        {
            // Line feature
            int linefeature_index = -1;
            for (auto &it_per_id : f_manager.linefeature)
            {
                it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
                if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
                    continue;
                ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

                int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
                if (imu_i != 0)             // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                for (auto &it_per_frame : it_per_id.linefeature_per_frame)
                {
                    imu_j++;

                    std::vector<int> drop_set;
                    if(imu_i == imu_j)
                    {
//                        drop_set = vector<int>{0, 2};   // marg pose and feature,  !!!! do not need marg, just drop they  !!!
                        continue;
                    }else
                    {
                        drop_set = vector<int>{2};      // marg feature
                    }

                    Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
                    lineProjectionFactor *f = new lineProjectionFactor(obs);     // 特征重投影误差

                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_j], para_Ex_Pose[0], para_LineFeature[linefeature_index]},
                                                                                   drop_set);// vector<int>{0, 2} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }

        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];   //
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    sum_marg_time_ += t_whole_marginalization.toc();
    mean_marg_time_ = sum_marg_time_/frame_cnt_;
    ROS_INFO("whole marginalization costs: %f", mean_marg_time_);

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::optimization()
{
    frame_cnt_ ++;
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q,v,ba,bg加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);                // v,ba,bg
    }
    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    TicToc t_whole, t_prepare;    // 统计程序运行时间
    TicToc t_solver;
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去

#ifdef DebugFactor
    /* Debug: 监视 prior factor*/
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    int marg_residual_size=0;
    std::vector<ceres::ResidualBlockId> imufactor_residual_block_ids;
    //end debud
#endif
    // 滑动窗口marg以后，上一次的prior factor
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ceres::ResidualBlockId block_id = problem.AddResidualBlock(marginalization_factor, NULL,
                                                                   last_marginalization_parameter_blocks);
#ifdef DebugFactor
        marg_residual_size = marginalization_factor->num_residuals();  // used to debug
        residual_block_ids.push_back(block_id);           // used to debug
#endif
    }

    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        ceres::ResidualBlockId block_id = problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
#ifdef DebugFactor
        imufactor_residual_block_ids.push_back(block_id);
#endif
    }

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            f_m_cnt++;
        }
    }

//    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
//    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
//    if (marginalization_flag == MARGIN_OLD)
//        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//    else
//        options.max_solver_time_in_seconds = SOLVER_TIME;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    sum_solver_time_ += t_solver.toc();
    mean_solver_time_ = sum_solver_time_/frame_cnt_;
    ROS_INFO("whole solver costs: %f", mean_solver_time_);
    //cout << summary.BriefReport() << endl;
//    ROS_INFO("Only Points Iterations : %d", static_cast<int>(summary.iterations.size()));
//    ROS_INFO("Only Points solver costs: %f", t_solver.toc());

#ifdef DebugFactor
    //----------- debug --------------------
    // marg factor
    Eigen::VectorXd err(marg_residual_size);
    err.setZero();
    ceres::Problem::EvaluateOptions opts;
    opts.residual_blocks = residual_block_ids;
    double total_cost = 0.0;
    vector<double> residuals;
    problem.Evaluate(opts,&total_cost, &residuals, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        //std::cout << residuals[i]<<std::endl;
        err[i] = residuals[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-before.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<" error befor: " << err.squaredNorm()<<" " << total_cost <<std::endl;

    // imu factor
    ceres::Problem::EvaluateOptions imufactor_opts;
    imufactor_opts.residual_blocks = imufactor_residual_block_ids;
    double total_cost_imufactor = 0.0;
    vector<double> residuals_imufactor;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor, &residuals_imufactor, nullptr, nullptr);
    Eigen::VectorXd imufactor_err(residuals_imufactor.size());
    imufactor_err.setZero();
    for(int i=0; i< residuals_imufactor.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err[i] = residuals_imufactor[i];
    }
    std::cout<<" IMU error befor: " << imufactor_err.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    // --------------------  end debug -------------------------
#endif

    double2vector();

#ifdef DebugFactor
    // ----------------  debug  ----------------------
    vector2double();
    Eigen::VectorXd err2(marg_residual_size);
    err2.setZero();
    vector<double> residuals2;
    problem.Evaluate(opts,&total_cost, &residuals2, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        err[i] = residuals2[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-after.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<"error after: " << err.squaredNorm()<<" "<< total_cost <<std::endl;
    // imu factor
    double total_cost_imufactor2 = 0.0;
    vector<double> residuals_imufactor2;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor2, &residuals_imufactor2, nullptr, nullptr);
    Eigen::VectorXd imufactor_err2(residuals_imufactor2.size());
    imufactor_err2.setZero();
    for(int i=0; i< residuals_imufactor2.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err2[i] = residuals_imufactor2[i];
    }
    std::cout<<" IMU error after: " << imufactor_err2.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    //------------------- end debug  --------------------------------
#endif

    // 将优化以后要marg掉的部分转为prior factor
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        // 构建一个新的 prior info
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        /*
           将最老帧上约束转变为 prior, 那有哪些约束是跟这个最老的帧相关的呢？
           1. 上一次优化以后留下的 prior 里可能存在
           2. 跟最老帧 存在 预积分imu 约束
           3. 最老帧上有很多特征观测约束
        */
        // 1. 上一次优化以后留下的 prior 里可能存在约束
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])    // 最老的一帧给丢掉
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // 2. 最老的两帧之间的 预积分 factor
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});  // vector<int>{0, 1} 表示要marg的参数下标，比如这里对应para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 3. 最老帧上有很多特征观测约束
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)    // 遍历所有特征
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)             // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)   //遍历这个特征的所有观测
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});// vector<int>{0, 3} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }

    sum_marg_time_ += t_whole_marginalization.toc();
    mean_marg_time_ = sum_marg_time_/frame_cnt_;
    ROS_INFO("whole marginalization costs: %f", mean_marg_time_);
//    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            // 经过上面一系列的swap操作，WINDOW_SIZE - 1 这时是保存的 最新一帧图像的状态
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];      // 经过上面一系列的swap操作，最后这个就是多余的重复的
            // 但是要注意，最新一帧上的预积分量要累计下去
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                double t_0 = Headers[0].stamp.toSec();
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);

            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            // marg 掉的是倒数第2帧，这时候只需要把倒数第二帧的数据给替换掉
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];   // R0 窗口里marg掉的那个第i帧的相机坐标系到世界坐标系的变换矩阵Rwci
        R1 = Rs[0] * ric[0];     // 由于窗口移动了，丢掉最老的第i帧，数据都左移动。现在窗口里的第0帧是之前窗口里第1帧，这里Rs[0]对应以前的i+1帧，Rwi * Ric = Rwci+1
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();

}

