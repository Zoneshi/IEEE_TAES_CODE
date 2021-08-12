#ifndef _RUNGE_KUTTA_H
#define _RUNGE_KUTTA_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


// struct configuration parameters
typedef struct ConfigParameter{
    double initial_time;
    double time_step;
    Vector3d Ib;
    VectorXd initial_states;
} con_para;

// rungekutta class
class RungeKutta{
    private:
        double m_current_time;
        double m_time_step;
        VectorXd m_states;
        Vector3d m_Ib;
        VectorXd differential_equation(const double &t, const VectorXd &x);
    public:
        RungeKutta(const con_para &para);
        ~RungeKutta();
        void Integrate();
        double getCurrentTime();
        VectorXd getCurrentStates();
};

#endif