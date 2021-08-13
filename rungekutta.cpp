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
    m_Ib           = para.Ib;
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
    //*****************************************************************************
    //  PARSE SYSTEM STATES
    //*****************************************************************************
    //angular velocity: omega
    Vector3d omega(x(0),x(1),x(2)); 

    // augmented angular velocity: omega_bar
    Quaterniond omega_bar(0,0.5*omega.x(),0.5*omega.y(),0.5*omega.z()); 

    // current attitude: q
    Quaterniond q(x(3),x(4),x(5),x(6));
    q = q.normalized();

    // current adaptivce gains
    VectorXd theta(6);
    theta << x(7), x(8), x(9), x(10), x(11), x(12);

    // desired attitude: qd
    Quaterniond qd(1.0,0.0,0.0,0.0);

    // desired angular velocity: omega_d
    Vector3d omega_d(0.0,0.0,0.0);

    // constant but unknown moment of inertia: Ib
    Matrix3d Ib = m_Ib.asDiagonal();

    // aerodynamic torque
    Vector3d MA(0.0,0.0,0.0);

    // regression matrix Y2
    Vector3d Y2(omega.y()*omega.z(),omega.x()*omega.z(),omega.x()*omega.y());

    // regression matrix Y_omega
    Matrix<double,3,6> Y_omega;

    //*****************************************************************************
    //  USER DEFINED AREA: TO CONSTRUCT A CONTROL SIGNAL!
    //*****************************************************************************
    // quaternion tracking error: qe        [Eq.(42)]
    Quaterniond qe = qd.inverse() * q; 
    qe = qe.normalized();

    // error angular velocity: omega_e      [Eq.(44)]
    Vector3d omega_e = omega - qe.toRotationMatrix().transpose()*omega_d;

    // reference angular velocity: omega_r  [Eq.(45)]
    Vector3d omega_r = qe.toRotationMatrix().transpose()*omega_d - q.vec();

    // auxiliary angular velocity: omega_a  [Eq.(46)]
    Vector3d omega_a = omega - omega_r;

    // control torque: MT                   [Eq.(50)]
    Vector3d MT = Ib*(-2*omega_a - 2*q.vec()-Ib.inverse()*omega.cross(Ib*omega));
    
    //*****************************************************************************
    // SYSTEM DYNAMICS
    //*****************************************************************************
    // rotational dynamics: dot_omega       [Eq.(11)]
    Vector3d dot_omega = Ib.inverse()*(MT + MA - omega.cross(Ib*omega));

    // attitude dynamics: dot_q             [Eq.(14)]
    Quaterniond dot_q  = q * omega_bar;

    // adaptive law: dot_theta
    VectorXd dot_theta(6);
    dot_theta = Y_omega.transpose()*omega_a;

    //*****************************************************************************
    //  RETURN VALUES
    //*****************************************************************************
    VectorXd dot_x(x.size());
    dot_x << dot_omega, dot_q.w(), dot_q.vec(), dot_theta;
    return dot_x;
}
