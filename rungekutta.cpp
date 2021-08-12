#include "rungekutta.h"

/**
 * @brief Construct a new Spring Mass Damper:: Spring Mass Damper object
 * 
 * @param para 
 */
RungeKutta::RungeKutta(const con_para &para){
    cout << "Configurating System Parameters..." << endl;
    m_current_time = para.initial_time;
    m_time_step    = para.time_step;
    m_states       = para.initial_states;
    cout << "Configuration Completed!" << endl;
}


/**
 * @brief Destroy the Spring Mass Damper:: Spring Mass Damper object
 * 
 */
RungeKutta::~RungeKutta(){
    cout <<"Simulation Completed!" << endl;
}


/**
 * @brief Get current states
 * 
 * @return VectorXd 
 */
VectorXd RungeKutta::getCurrentStates(){
    return m_states;
}


/**
 * @brief Get current time
 * 
 * @return double 
 */
double RungeKutta::getCurrentTime(){
    return m_current_time;
}

/**
 * @brief start integration
 * 
 */
void RungeKutta::Integrate(){

    VectorXd k1 = differential_equation(m_current_time,m_states);

    VectorXd k2 = differential_equation(m_current_time + 0.5 * m_time_step,m_states + 0.5 * m_time_step * k1);

    VectorXd k3 = differential_equation(m_current_time + 0.5 * m_time_step,m_states + 0.5 * m_time_step * k2);

    VectorXd k4 = differential_equation(m_current_time + m_time_step,m_states + m_time_step * k3);

    m_states = m_states + m_time_step * (k1 + 2*k2 + 2*k3 + k4)/ 6.0;
    m_current_time = m_current_time + m_time_step;
}


/**
 * @brief system dynamics in term of ordinary differential equation
 * 
 * @param t current time
 * @param x current states
 * @return VectorXd 
 */
VectorXd RungeKutta::differential_equation(const double &t, const VectorXd &x){
    // parse states
    double pos = x(0);
    double vel = x(1);
    double theta1 = x(2);
    double theta2 = x(3);
    double gamma = 2;

    // construct control input
    double s = pos + vel;
    double u = -4*s - theta1 * pos - theta2 * vel;
    // system dynamics
    double dx1 = vel;
    double dx2 = 2*pos - 1*vel + u;
    double dtheta1 = gamma * pos * s;
    double dtheta2 = gamma * vel * s;
    // return dot_x
    VectorXd dot_x(x.size());
    dot_x << dx1, dx2, dtheta1, dtheta2;

    return dot_x;
}
