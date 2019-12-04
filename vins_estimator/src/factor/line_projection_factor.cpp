#include "line_projection_factor.h"
#include "../utility/line_geometry.h"
#include "line_parameterization.h"
#include "../utility/utility.h"

Eigen::Matrix2d lineProjectionFactor::sqrt_info;
double lineProjectionFactor::sum_t;

lineProjectionFactor::lineProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Tbc
  parameters[2]:  line_orth
*/
bool lineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
    Vector6d line_w = orth_to_plk(line_orth);

    Eigen::Matrix3d Rwb(Qi);
    Eigen::Vector3d twb(Pi);
    Vector6d line_b = plk_from_pose(line_w, Rwb, twb);
    //std::cout << line_b.norm() <<"\n";
    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_c = plk_from_pose(line_b, Rbc, tbc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

//    std::cout <<"---- sqrt_info: ------"<< sqrt_info << std::endl;
//    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout << residual <<"\n";
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        //std::cout << "jacobian_calculator:" << std::endl;
        if (jacobians[0])
        {
            //std::cout <<"jacobian_pose_i"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwb.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwb.transpose() * (nw + skew_symmetric(dw) * twb) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwb.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            //std::cout <<invTbc<<"\n"<<jaco_Lc_pose<<"\n\n";

            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {

            //std::cout <<"jacobian_ex_pose"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);

            Vector3d nb = line_b.head(3);
            Vector3d db = line_b.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = Rbc.transpose() * skew_symmetric(db);   // Lc_t
            jaco_Lc_ex.block(0,3,3,3) = skew_symmetric( Rbc.transpose() * (nb + skew_symmetric(db) * tbc) );  // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = skew_symmetric( Rbc.transpose() * db);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[2]);

            Eigen::Matrix3d Rwc = Rwb * Rbc;
            Eigen::Vector3d twc = Rwb * tbc + twb;
            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose() * skew_symmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Vector3d nw = line_w.head(3);
            Vector3d vw = line_w.tail(3);
            Vector3d u1 = nw/nw.norm();
            Vector3d u2 = vw/vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lw_orth;
            jaco_Lw_orth.setZero();
            jaco_Lw_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lw_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lw_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lw_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(3,3,3,1) = w(0) * u2;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
        }

    }

    // check jacobian
/*
    std::cout << "---------- check jacobian ----------\n";
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 16> num_jacobian;
    for (int k = 0; k < 16; k++)
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * Utility::deltaQ(delta);
        else if (a == 2)
            tic += delta;
        else if (a == 3)
            qic = qic * Utility::deltaQ(delta);
        else if (a == 4) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 5) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_w = orth_to_plk(line_orth);

        Eigen::Matrix3d Rwb(Qi);
        Eigen::Vector3d twb(Pi);
        Vector6d line_b = plk_from_pose(line_w, Rwb, twb);

        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_c = plk_from_pose(line_b, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_c.head(3);
        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );
        double l_trinorm = l_norm * l_sqrtnorm;

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}


//////////////////////////////////////////////////
Eigen::Matrix2d lineProjectionFactor_incamera::sqrt_info;
lineProjectionFactor_incamera::lineProjectionFactor_incamera(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_incamera::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
    Vector6d line_ci = orth_to_plk(line_orth);

    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

    Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
    Eigen::Vector3d twbi(Pi);
    Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

    Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
    Eigen::Vector3d twbj(Pj);
    Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

    Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_cj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

/*
            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Matrix6d invTwbj;
            invTwbj << Rwbj.transpose(), -Rwbj.transpose()*skew_symmetric(twbj),
                 en::Matrix3d::Zero(),  Rwbj.transpose();
*/

            Matrix3d Rwcj = Rwbj * Rbc;
            Vector3d twcj = Rwbj * tbc + twbj;
            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_bi.head(3);
            Vector3d dbi = line_bi.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwbi * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwbi * skew_symmetric( nbi) - skew_symmetric(twbi) * Rwbi * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwbi * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwbj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwbj.transpose() * (nw + skew_symmetric(dw) * twbj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwbj.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Vector3d nci = line_ci.head(3);
            Vector3d dci = line_ci.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = -Rbc.transpose() * Rbjbi * skew_symmetric( Rbc * dci) + Rbc.transpose() * skew_symmetric(Rbjbi * Rbc * dci);   // Lc_t
            Matrix3d tmp = skew_symmetric(tcjci) * Rcjci;
            jaco_Lc_ex.block(0,3,3,3) = -Rcjci * skew_symmetric(nci) + skew_symmetric(Rcjci * nci)
                                        -tmp * skew_symmetric(dci) + skew_symmetric(tmp * dci);    // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = -Rcjci * skew_symmetric(dci) + skew_symmetric(Rcjci * dci);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[3]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Vector3d nci = line_ci.head(3);
            Vector3d vci = line_ci.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lc_orth;
            jaco_Lc_orth.setZero();
            jaco_Lc_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lc_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lc_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lc_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc * Tcjci * jaco_Lc_orth;
        }

    }
/*
    // check jacobian
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    if(jacobians[3])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[3]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 22> num_jacobian;// 3 * 6 + 4
    for (int k = 0; k < 22; k++)
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * deltaQ(delta);
        else if (a == 6) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 7) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_ci = orth_to_plk(line_orth);
        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

        Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
        Eigen::Vector3d twbi(Pi);
        Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

        Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
        Eigen::Vector3d twbj(Pj);
        Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

        Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_cj.head(3);

        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}


Eigen::Matrix2d lineProjectionFactor_instartframe::sqrt_info;
lineProjectionFactor_instartframe::lineProjectionFactor_instartframe(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_instartframe::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

    Eigen::Vector4d line_orth( parameters[0][0],parameters[0][1],parameters[0][2],parameters[0][3]);
    Vector6d line_ci = orth_to_plk(line_orth);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_ci.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[0]);


            Vector3d nci = line_ci.head(3);
            Vector3d vci = line_ci.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lci_orth;
            jaco_Lci_orth.setZero();
            jaco_Lci_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lci_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lci_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lci_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc  * jaco_Lci_orth;
        }

    }

    return true;
}
