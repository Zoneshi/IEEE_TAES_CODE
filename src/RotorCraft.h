#pragma once

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 3, 5> Matrix3X5;
typedef Matrix<double, 3, 6> Matrix3X6;
typedef Matrix<double, 3, 7> Matrix3X7;
typedef Matrix<double, 3, 12> Matrix3X12;
typedef Matrix<double, 6, 6> Matrix6X6;
typedef Matrix<double, 7, 7> Matrix7X7;
typedef Matrix<double, 12, 12> Matrix12X12;

typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, 18, 1> Vector18d;
typedef Matrix<double, 21, 1> Vector21d;
typedef Matrix<double, 36, 1> Vector36d;
typedef Matrix<double, 49, 1> Vector49d;
typedef Matrix<double, 144, 1> Vector144d;
//*******************************************
//	定义全局变量
//*******************************************
const Vector3d BodyIxIyIz = Vector3d(2.0, 4.0, 4.0);
const double BodyMass = 10.0;
const Vector3d Gravity_g = Vector3d(0.0, 0.0, 9.8);
const double SRef = 0.018;
const double LRef = 0.1;
const double AirDensity = 1.2;
const Vector5d cf = { -0.5, -1.0, -1.0, 0.0, 0.0 };
const Vector6d cm = { -0.1, -0.2, -0.1, -0.2, -0.1, 0.0 };

const double AeroMoment = 1.0;
const double AeroForce = 1.0;

const double alpha_p = 0.5; // 位置回路的alpha
const double k_p = 0.2; // 位置误差增益
const double k_v = 0.2; // 速度误差增益

const double alpha_q = 2.0; // 姿态回路的alpha
const double k_q = 1.0; // 姿态误差增益
const double k_w = 2.0; // 角速度误差增益
const double LEARNING_START_TIME = 1.0;
const double cl_sw = 0.0;
const double ad_sw = 0.0;
//-----------------------------v1,   cf1, cf2, cf3,   cf4,     cf5
const Vector6d Gamma_thetav = { 0.05, 0.2, 0.2, 0.2, 0.0, 0.0 };
//---------------------------------v1,  cf1, cf2, cf3,   cf4,     cf5
const Vector6d Gamma_CL_thetav = { 0.0005, 0.0005, 0.002, 0.002, 0.0, 0.0 };
//-------------------------------w11  w12, w13, w21, w22, w23, cm1, cm2, cm3, cm4,cm5,  cm6
const Vector12d Gamma_thetaw = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.05, 0.05, 0.05, 0.05, 0.05, 0.0 };
//----------------------------------w11  w12, w13, w21, w22, w23, cm1, cm2, cm3,cm4, cm5, cm6
const Vector12d Gamma_CL_thetaw = { 1.0, 1.0, 1.0, 0.1, 1.0, 5.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0 };

const double fre_xy = M_PI / 20.0; // x和y方向的频率
const double fre_z = M_PI / 10.0; // z方向的频率
const double xy_vel = 400.0;       // x和y方向的位置赋值
const double z_vel = 100.0;       // z方向的位置赋值
const double LowPassTimeConstant = 0.01;        // 低通滤波器时间常数
const double Tau = 0.01;
//*******************************************
//	函数声明
//*******************************************

typedef VectorXd(*DifferentialEquation)(const double& t, const VectorXd& SysStates);

VectorXd ClosedLoopEquation(const double& t, const VectorXd& SysStates);

VectorXd GenerateInitialStates(const Vector3d& pos, const Vector3d& vel, const Vector3d& eul, const Vector3d& omega);

void ReferenceTrajectory(Vector3d& pos_d, Vector3d& vel_d, Vector3d& acc_d, double& phi_d, double& dot_phi_d, const double& t);

void RungeKuttaIntegral(DifferentialEquation SysDynamics, double& t, VectorXd& SysStates, const double& TimeStep);

void WriteDataToFile(const double& t, const VectorXd& WriteData, ofstream& File);

template <typename T> T deg2rad(T deg)
{
    return deg * M_PI / 180.0;
}
template <typename T> T rad2deg(T rad)
{
    return rad * 180.0 / M_PI;
}
