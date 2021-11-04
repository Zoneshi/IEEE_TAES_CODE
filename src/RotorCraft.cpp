#include "RotorCraft.h"

VectorXd ClosedLoopEquation(const double& t, const VectorXd& SysStates)
{
    //************************************************************************
    //解析系统状态
    //************************************************************************
    Vector3d pos = Vector3d(SysStates(0), SysStates(1), SysStates(2));
    Vector3d vel = Vector3d(SysStates(3), SysStates(4), SysStates(5));
    Quaterniond q = Quaterniond(SysStates(6), SysStates(7), SysStates(8), SysStates(9)).normalized();
    Vector3d omega = Vector3d(SysStates(10), SysStates(11), SysStates(12));
    Quaterniond omega_bar = Quaterniond(0.0, 0.5 * SysStates(10), 0.5 * SysStates(11), 0.5 * SysStates(12));

    Vector3d ThrustVectorAux = Vector3d(SysStates(13), SysStates(14), SysStates(15));
    Vector3d DesiredOmegaAux = Vector3d(SysStates(16), SysStates(17), SysStates(18));

    double thetav1 = SysStates(19);          //位置回路自适应参数thetav1
    Vector5d thetav2 = SysStates(seq(20, 24)); //位置回路气动力自适应参数thetav3
    Vector3d thetaw1 = SysStates(seq(25, 27)); //姿态回路自适应参数thetaw1
    Vector3d thetaw2 = SysStates(seq(28, 30)); //姿态回路自适应参数thetaw2
    Vector6d thetaw3 = SysStates(seq(31, 36)); //姿态回路自适应参数thetaw3
    Vector3d xiv = SysStates(seq(37, 39));
    Vector3d xiw = SysStates(seq(40, 42));
    Vector18d YFv_vec = SysStates(seq(43, 60));
    Vector36d YFw_vec = SysStates(seq(61, 96));
    Vector36d YFvYFv_vec = SysStates(seq(97, 132));
    Vector6d YFvYFvthetav = SysStates(seq(133, 138));
    Vector144d YFwYFw_vec = SysStates(seq(139, 282));
    Vector12d YFwYFwthetaw = SysStates(seq(283, 294));

    Matrix3X6 YFv = YFv_vec.reshaped(3, 6);
    Matrix3X12 YFw = YFw_vec.reshaped(3, 12);
    Matrix6X6 YFvYFv = YFvYFv_vec.reshaped(6, 6);
    Matrix12X12 YFwYFw = YFwYFw_vec.reshaped(12, 12);
    //************************************************************************
    //计算简单的复用向量
    //************************************************************************
    // 单位矩阵
    Matrix3d I = MatrixXd::Identity(3, 3); // 3x3的单位矩阵
    Matrix3d Ib = BodyIxIyIz.asDiagonal();
    // 计算当前的攻角和侧滑角
    Vector3d vel_B = q.toRotationMatrix().transpose() * vel;

    // 根据预置参数计算当前的指令位置、速度和加速度
    Vector3d pos_d = VectorXd::Zero(3);
    Vector3d vel_d = VectorXd::Zero(3);
    Vector3d acc_d = VectorXd::Zero(3);
    double phi_d = 0.0;
    double dot_phi_d = 0.0;
    // 根据用户需求生成指令位置、速度和加速度
    ReferenceTrajectory(pos_d, vel_d, acc_d, phi_d, dot_phi_d, t);

    // 计算动压
    double V = vel.norm();
    double Q = 0.5 * AirDensity * V * V;

    // 计算式(17)中的矩阵ff
    Matrix3X5 ff{ {vel_B.x()/V, 0.0, 0.0,0.0, 0.0}, {0.0, vel_B.y() / V, 0.0, 0.0, 0.0}, {0.0, 0.0, vel_B.z() / V, 0.0, 0.0} };
    ff = Q * SRef * ff;
    Vector3d F = ff * cf;

    // 计算式(13)中的fm
    Matrix3X6 fm{
        {omega.x(), 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, vel_B.z() / V, omega.y(), 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, vel_B.y() / V, omega.z(), 0.0} };
    fm = Q * SRef * LRef * fm;
    Vector3d M_A = fm * cm;

    //************************************************************************
    //位置回路
    //************************************************************************
    // 计算回归向量Yv
    Matrix3X5 Yv2 = q.toRotationMatrix() * ff;

    // 计算辅助向量
    Vector3d pos_e = pos - pos_d;
    Vector3d vel_e = vel - vel_d;
    Vector3d vel_r = vel_d - alpha_p * pos_e;
    Vector3d vel_a = vel - vel_r;
    Vector3d dot_vel_r = acc_d - alpha_p * vel_e;
    // 计算期望推力
    Vector3d T_d = (-k_v * vel_a - k_p * pos_e - Gravity_g + dot_vel_r - Yv2 * thetav2) / (1 + thetav1);
    // 计算期望推力矢量
    Vector3d r = T_d.normalized();
    // 计算机体系下的推力
    Vector3d T = Vector3d(T_d.norm(), 0.0, 0.0);

    // 计算位置和速度微分方程
    Vector3d dot_pos = vel;
    Vector3d dot_vel = (q.toRotationMatrix() * (T + AeroForce * F) + BodyMass * Gravity_g) / BodyMass;

    VectorXd DotPosition(6);
    DotPosition << dot_pos, dot_vel;
    //************************************************************************
    // 位置到姿态的过渡
    //************************************************************************

    // 计算期望推力矢量的变化率
    Vector3d dot_r = (r - ThrustVectorAux) / LowPassTimeConstant;

    // 计算期望的姿态四元数
    Quaterniond q_m = Quaterniond(
        (1 + r.x()) / (sqrt(2 + 2 * r.x())), 0.0, -r.z() / (sqrt(2 + 2 * r.x())), r.y() / (sqrt(2 + 2 * r.x())));
    Quaterniond q_phi = Quaterniond(cos(0.5 * phi_d), sin(0.5 * phi_d), 0.0, 0.0);
    Quaterniond q_d = q_m * q_phi;

    // 计算期望角速度
    Vector3d omega_m = Vector3d(
        (r.z() * dot_r.y() - r.y() * dot_r.z()) / (1 + r.x()),
        (r.z() * dot_r.x() - (1 + r.x()) * dot_r.z()) / (1 + r.x()),
        ((1 + r.x()) * dot_r.y() - r.y() * dot_r.x()) / (1 + r.x()));
    Vector3d omega_phi = Vector3d(dot_phi_d, 0.0, 0.0);
    Vector3d omega_d = omega_phi + q_phi.toRotationMatrix().transpose() * omega_m;

    //************************************************************************
    // 姿态回路
    //************************************************************************
    // 计算期望角速度的变化率
    Vector3d dot_omega_d = (omega_d - DesiredOmegaAux) / LowPassTimeConstant;

    // 计算回归向量Yw
    Vector3d Yw2_vec = Vector3d(omega.y() * omega.z(), omega.x() * omega.z(), omega.x() * omega.y());
    Matrix3d Yw2 = Yw2_vec.asDiagonal();
    Matrix3X6 Yw3 = fm;

    // 计算辅助向量
    Quaterniond q_e = q_d.inverse() * q;
    Vector3d omega_e = omega - q_e.toRotationMatrix().transpose() * omega_d;
    Vector3d omega_r = q_e.toRotationMatrix().transpose() * omega_d - alpha_q * q_e.vec();
    Vector3d omega_a = omega - omega_r;
    Vector3d dot_omega_r = -omega_e.cross(q_e.toRotationMatrix().transpose() * omega_d) +
        q_e.toRotationMatrix().transpose() * dot_omega_d - 0.5 * alpha_q * q_e.w() * omega_e -
        0.5 * alpha_q * q_e.vec().cross(omega_e);

    Matrix3d diag_thetaw1 = thetaw1.asDiagonal();
    Matrix3d Ithetaw1 = I + diag_thetaw1;

    // 计算力矩
    Vector3d M_T =
        Ithetaw1.inverse() * (-k_w * omega_a - k_q * q_e.vec() + dot_omega_r - Yw2 * thetaw2 - Yw3 * thetaw3);

    // 计算四元数和角速度速度微分方程
    Quaterniond dot_q = q * omega_bar;
    Vector3d dot_omega = Ib.inverse() * ((M_T + AeroMoment * M_A) - omega.cross(Ib * omega));
    VectorXd DotAngular(7);
    DotAngular << dot_q.w(), dot_q.x(), dot_q.y(), dot_q.z(), dot_omega;

    // 低通滤波器
    Vector3d DotThrustVectorAux = (r - ThrustVectorAux) / LowPassTimeConstant;
    Vector3d DotDesiredOmegaAux = (omega_d - DesiredOmegaAux) / LowPassTimeConstant;

    //************************************************************************
    // 自适应更新参数
    //************************************************************************

    // 位置回路
    Vector3d Yv1 = T_d;
    Matrix3X6 Yv;
    Yv << Yv1, Yv2;

    Vector6d theta_v;
    theta_v << thetav1, thetav2;

    MatrixXd Gamma_ThetaV = Gamma_thetav.asDiagonal();
    MatrixXd Gamma_CL_ThetaV = Gamma_CL_thetav.asDiagonal();
    VectorXd DotThetaV = ad_sw * Gamma_ThetaV * Yv.normalized().transpose() * vel_a.normalized() -
        cl_sw * Gamma_CL_ThetaV * (YFvYFv * theta_v - YFvYFvthetav);

    // 姿态回路
    Matrix3d Yw1 = M_T.asDiagonal();
    Matrix3X12 Yw;
    Yw << Yw1, Yw2, Yw3;

    Vector12d theta_w;
    theta_w << thetaw1, thetaw2, thetaw3;

    MatrixXd Gamma_ThetaW = Gamma_thetaw.asDiagonal();
    MatrixXd Gamma_CL_ThetaW = Gamma_CL_thetaw.asDiagonal();
    Vector12d DotThetaW = ad_sw * Gamma_ThetaW * Yw.normalized().transpose() * omega_a.normalized() -
        cl_sw * Gamma_CL_ThetaW * (YFwYFw * theta_w - YFwYFwthetaw);

    // 辅助函数动力学

    Vector3d wv = k_p * pos_e + Yv * theta_v;
    Vector3d ww = k_q * q_e.vec() + Yw * theta_w;

    Vector3d DotXiv = (wv - (1 / Tau - k_v) * vel_a - xiv) / Tau;
    Vector3d DotXiw = (ww - (1 / Tau - k_w) * omega_a - xiw) / Tau;

    Vector18d Yv_vec = Yv.reshaped(18, 1);
    Vector36d Yw_vec = Yw.reshaped(36, 1);

    // 根据滤波器反推出的理想YFvthetav和YFwthetaw
    Vector3d YFvthetav = xiv + vel_a / Tau;
    Vector3d YFwthetaw = xiw + omega_a / Tau;

    // 当前时刻的YFvYFv 和YFwYFw
    Matrix6X6 YFvYFv_t = YFv.transpose() * YFv;
    Matrix12X12 YFwYFw_t = YFw.transpose() * YFw;

    Vector6d DotYFvYFvthetav = VectorXd::Zero(6);
    Vector12d DotYFwYFwthetaw = VectorXd::Zero(12);
    Vector36d DotYFvYFv = VectorXd::Zero(36);
    Vector144d DotYFwYFw = VectorXd::Zero(144);

    if (t > LEARNING_START_TIME)
    {
        DotYFvYFvthetav = YFv.transpose() * YFvthetav;
        DotYFwYFwthetaw = YFw.transpose() * YFwthetaw;
        DotYFvYFv = YFvYFv_t.reshaped(36, 1);
        DotYFwYFw = YFwYFw_t.reshaped(144, 1);
    }

    Vector18d DotYFv = (Yv_vec - YFv_vec) / Tau;
    Vector36d DotYFw = (Yw_vec - YFw_vec) / Tau;

    //************************************************************************
    // 构造闭环系统动力学
    //************************************************************************
    VectorXd dSysStates_dt = VectorXd::Zero(295);
    dSysStates_dt << DotPosition, DotAngular, DotThrustVectorAux, DotDesiredOmegaAux, DotThetaV, DotThetaW, DotXiv,
        DotXiw, DotYFv, DotYFw, DotYFvYFv, DotYFvYFvthetav, DotYFwYFw, DotYFwYFwthetaw;
    return dSysStates_dt;
}

VectorXd GenerateInitialStates(const Vector3d& pos, const Vector3d& vel, const Vector3d& eul, const Vector3d& omega)
{
    const Quaterniond q_init = AngleAxisd(eul(0), Vector3d::UnitZ()) * AngleAxisd(eul(1), Vector3d::UnitY()) *
        AngleAxisd(eul(2), Vector3d::UnitX());
    Vector4d q0 = Vector4d(q_init.w(), q_init.x(), q_init.y(), q_init.z());
    Vector3d dot_r0 = VectorXd::Zero(3);
    Vector3d dot_omega_d0 = VectorXd::Zero(3);

    double thetav1_init = 1.0 / 8.0 - 1.0;
    Vector5d thetav2_init = VectorXd::Zero(5);

    Vector3d thetaw1_init = Vector3d(-0.0, -0.5, -0.5);
    Vector3d thetaw2_init = Vector3d(0.0, -0.1, -0.1);
    Vector6d thetaw3_init = VectorXd::Zero(6);

    Vector3d xiv_init = Vector3d(0.0, 0.0, 0.0);
    Vector3d xiw_init = Vector3d(0.0, 0.0, 0.0);

    Vector18d YFv_init = VectorXd::Zero(18);
    Vector36d YFw_init = VectorXd::Zero(36);

    Vector36d YFvYFv_init = VectorXd::Zero(36);
    Vector6d YFvYFvtheta_init = VectorXd::Zero(6);

    Vector144d YFwYFw_init = VectorXd::Zero(144);
    Vector12d YFwYFwtheta_init = VectorXd::Zero(12);

    VectorXd SysStates(295);
    SysStates << pos, vel, q0, omega, dot_r0, dot_omega_d0, thetav1_init, thetav2_init, thetaw1_init, thetaw2_init,
        thetaw3_init, xiv_init, xiw_init, YFv_init, YFw_init, YFvYFv_init, YFvYFvtheta_init, YFwYFw_init,
        YFwYFwtheta_init;

    return SysStates;
}

void ReferenceTrajectory(
    Vector3d& pos_d, Vector3d& vel_d, Vector3d& acc_d, double& phi_d, double& dot_phi_d, const double& t)
{
    pos_d = Vector3d(xy_vel * cos(fre_xy * t), xy_vel * sin(fre_xy * t), z_vel * sin(fre_z * t));
    vel_d =
        Vector3d(-xy_vel * fre_xy * sin(fre_xy * t), xy_vel * fre_xy * cos(fre_xy * t), z_vel * fre_z * cos(fre_z * t));
    acc_d = Vector3d(
        -xy_vel * fre_xy * fre_xy * cos(fre_xy * t), -xy_vel * fre_xy * fre_xy * sin(fre_xy * t),
        -z_vel * fre_z * fre_z * sin(fre_z * t));
    phi_d = M_PI / 4.0 * t;
    dot_phi_d = M_PI / 4.0;
}

/// @brief 四阶龙哥库塔积分函数
/// @param SysDynamics 类型为DifferentialEquation 的回调函数，表示系统动力学
/// @param t 当前时间t
/// @param SysStates 当前系统状态
/// @param TimeStep 积分步长
void RungeKuttaIntegral(DifferentialEquation SysDynamics, double& t, VectorXd& SysStates, const double& TimeStep)
{
    VectorXd k1 = SysDynamics(t, SysStates);

    t = t + 0.5 * TimeStep;

    VectorXd k2 = SysDynamics(t, SysStates + 0.5 * TimeStep * k1);

    VectorXd k3 = SysDynamics(t, SysStates + 0.5 * TimeStep * k2);

    t = t + 0.5 * TimeStep;

    VectorXd k4 = SysDynamics(t, SysStates + TimeStep * k3);

    SysStates = SysStates + TimeStep * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
}
/// @brief 将数据写入流文件，以'\t'为间隔，每一行为同一时间步下的仿真数据
/// @param t 要写入的时间
/// @param WriteData 要写入的数据
/// @param File 目标流文件
void WriteDataToFile(const double& t, const VectorXd& SysStates, ofstream& File)
{

    //************************************************************************
    //解析系统状态
    //************************************************************************
    Vector3d pos = Vector3d(SysStates(0), SysStates(1), SysStates(2));
    Vector3d vel = Vector3d(SysStates(3), SysStates(4), SysStates(5));
    Quaterniond q = Quaterniond(SysStates(6), SysStates(7), SysStates(8), SysStates(9)).normalized();
    Vector3d omega = Vector3d(SysStates(10), SysStates(11), SysStates(12));
    Quaterniond omega_bar = Quaterniond(0.0, 0.5 * SysStates(10), 0.5 * SysStates(11), 0.5 * SysStates(12));

    Vector3d ThrustVectorAux = Vector3d(SysStates(13), SysStates(14), SysStates(15));
    Vector3d DesiredOmegaAux = Vector3d(SysStates(16), SysStates(17), SysStates(18));

    double thetav1 = SysStates(19);          //位置回路自适应参数thetav1
    Vector5d thetav2 = SysStates(seq(20, 24)); //位置回路气动力自适应参数thetav3
    Vector3d thetaw1 = SysStates(seq(25, 27)); //姿态回路自适应参数thetaw1
    Vector3d thetaw2 = SysStates(seq(28, 30)); //姿态回路自适应参数thetaw2
    Vector6d thetaw3 = SysStates(seq(31, 36)); //姿态回路自适应参数thetaw3
    Vector3d xiv = SysStates(seq(37, 39));
    Vector3d xiw = SysStates(seq(40, 42));
    Vector18d YFv_vec = SysStates(seq(43, 60));
    Vector36d YFw_vec = SysStates(seq(61, 96));
    Vector36d YFvYFv_vec = SysStates(seq(97, 132));
    Vector6d YFvYFvthetav = SysStates(seq(133, 138));
    Vector144d YFwYFw_vec = SysStates(seq(139, 282));
    Vector12d YFwYFwthetaw = SysStates(seq(283, 294));

    Matrix3X6 YFv = YFv_vec.reshaped(3, 6);
    Matrix3X12 YFw = YFw_vec.reshaped(3, 12);
    Matrix6X6 YFvYFv = YFvYFv_vec.reshaped(6, 6);
    Matrix12X12 YFwYFw = YFwYFw_vec.reshaped(12, 12);
    //************************************************************************
    //计算简单的复用向量
    //************************************************************************
    // 单位矩阵
    Matrix3d I = MatrixXd::Identity(3, 3); // 3x3的单位矩阵
    Matrix3d Ib = BodyIxIyIz.asDiagonal();
    // 计算当前的攻角和侧滑角
    Vector3d vel_B = q.toRotationMatrix().transpose() * vel;

    // 根据预置参数计算当前的指令位置、速度和加速度
    Vector3d pos_d = VectorXd::Zero(3);
    Vector3d vel_d = VectorXd::Zero(3);
    Vector3d acc_d = VectorXd::Zero(3);
    double phi_d = 0.0;
    double dot_phi_d = 0.0;
    // 根据用户需求生成指令位置、速度和加速度
    ReferenceTrajectory(pos_d, vel_d, acc_d, phi_d, dot_phi_d, t);

    // 计算动压
    double V = vel.norm();
    double Q = 0.5 * AirDensity * V * V;

    // 计算式(17)中的矩阵ff
    Matrix3X5 ff{ {vel_B.x() / V, 0.0, 0.0,0.0, 0.0}, {0.0, vel_B.y() / V, 0.0, 0.0, 0.0}, {0.0, 0.0, vel_B.z() / V, 0.0, 0.0} };
    ff = Q * SRef * ff;
    Vector3d F = ff * cf;

    // 计算式(13)中的fm
    Matrix3X6 fm{
        {omega.x(), 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, vel_B.z() / V, omega.y(), 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, vel_B.y() / V, omega.z(), 0.0} };
    fm = Q * SRef * LRef * fm;
    Vector3d M_A = fm * cm;

    //************************************************************************
    //位置回路
    //************************************************************************
    // 计算回归向量Yv
    Matrix3X5 Yv2 = q.toRotationMatrix() * ff;

    // 计算辅助向量
    Vector3d pos_e = pos - pos_d;
    Vector3d vel_e = vel - vel_d;
    Vector3d vel_r = vel_d - alpha_p * pos_e;
    Vector3d vel_a = vel - vel_r;
    Vector3d dot_vel_r = acc_d - alpha_p * vel_e;
    // 计算期望推力
    Vector3d T_d = (-k_v * vel_a - k_p * pos_e - Gravity_g + dot_vel_r - Yv2 * thetav2) / (1 + thetav1);
    // 计算期望推力矢量
    Vector3d r = T_d.normalized();
    // 计算机体系下的推力
    Vector3d T = Vector3d(T_d.norm(), 0.0, 0.0);

    //************************************************************************
    // 位置到姿态的过渡
    //************************************************************************

    // 计算期望推力矢量的变化率
    Vector3d dot_r = (r - ThrustVectorAux) / LowPassTimeConstant;

    // 计算期望的姿态四元数
    Quaterniond q_m = Quaterniond(
        (1 + r.x()) / (sqrt(2 + 2 * r.x())), 0.0, -r.z() / (sqrt(2 + 2 * r.x())), r.y() / (sqrt(2 + 2 * r.x())));
    Quaterniond q_phi = Quaterniond(cos(0.5 * phi_d), sin(0.5 * phi_d), 0.0, 0.0);
    Quaterniond q_d = q_m * q_phi;

    // 计算期望角速度
    Vector3d omega_m = Vector3d(
        (r.z() * dot_r.y() - r.y() * dot_r.z()) / (1 + r.x()),
        (r.z() * dot_r.x() - (1 + r.x()) * dot_r.z()) / (1 + r.x()),
        ((1 + r.x()) * dot_r.y() - r.y() * dot_r.x()) / (1 + r.x()));
    Vector3d omega_phi = Vector3d(dot_phi_d, 0.0, 0.0);
    Vector3d omega_d = omega_phi + q_phi.toRotationMatrix().transpose() * omega_m;

    //************************************************************************
    // 姿态回路
    //************************************************************************
    // 计算期望角速度的变化率
    Vector3d dot_omega_d = (omega_d - DesiredOmegaAux) / LowPassTimeConstant;

    // 计算回归向量Yw
    Vector3d Yw2_vec = Vector3d(omega.y() * omega.z(), omega.x() * omega.z(), omega.x() * omega.y());
    Matrix3d Yw2 = Yw2_vec.asDiagonal();
    Matrix3X6 Yw3 = fm;

    // 计算辅助向量
    Quaterniond q_e = q_d.inverse() * q;
    Vector3d omega_e = omega - q_e.toRotationMatrix().transpose() * omega_d;
    Vector3d omega_r = q_e.toRotationMatrix().transpose() * omega_d - alpha_q * q_e.vec();
    Vector3d omega_a = omega - omega_r;
    Vector3d dot_omega_r = -omega_e.cross(q_e.toRotationMatrix().transpose() * omega_d) +
        q_e.toRotationMatrix().transpose() * dot_omega_d - 0.5 * alpha_q * q_e.w() * omega_e -
        0.5 * alpha_q * q_e.vec().cross(omega_e);

    Matrix3d diag_thetaw1 = thetaw1.asDiagonal();
    Matrix3d Ithetaw1 = I + diag_thetaw1;

    // 计算力矩
    Vector3d M_T =
        Ithetaw1.inverse() * (-k_w * omega_a - k_q * q_e.vec() + dot_omega_r - Yw2 * thetaw2 - Yw3 * thetaw3);

    VectorXd WriteData(37);
    WriteData << t, pos, pos_d, q.w(), q.vec(), q_d.w(), q_d.vec(), T.x(), M_T, thetav1, thetav2, thetaw1, thetaw2, thetaw3;
    for (auto data : WriteData)
    {
        File << data << '\t';
    }
    File << endl;
}