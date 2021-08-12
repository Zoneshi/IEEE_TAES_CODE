#include "rungekutta.h"
// #include <fstream>
// #include <string>

int main(){
    // output simulation results
    // string save_path = "/Users/zhongjiaoshi/VSCode/result.csv"; 
    // ofstream save_file(save_path);
    // save_file <<"time,pos,vel"<<endl;

    // system dimensions
    int num_states = 7;
    Vector3d omega0(1.0,0.0,0.0);
    Quaterniond q0(1.0,0.0,0.0,0.0);
    q0 = q0.normalized();

    // config system 
    con_para conpara;
    conpara.time_step      = 0.1;
    conpara.initial_time   = 0.0;
    conpara.Ib             = Vector3d(1.0,2.0,2.0);
    conpara.initial_states = VectorXd(num_states);
    conpara.initial_states << omega0, q0.w(), q0.vec();

    // constuct simulation instance
    RungeKutta rotor = RungeKutta(conpara);
    
    // simulation final time
    const double time_final = 15.0;

    // system states
    VectorXd sysStates(conpara.initial_states.size());

    // config output format
    cout.precision(6);
    cout.flags(ios::fixed);
    cout.setf(ios::right);

    // start simulation
    while (rotor.getCurrentTime() < time_final){
        sysStates = rotor.getCurrentStates();
        Quaterniond q(sysStates(3),sysStates(4),sysStates(5),sysStates(6));
        Vector3d euler = q.toRotationMatrix().eulerAngles(2,1,0);
        cout << rotor.getCurrentTime();
        for (int i = 0; i < sysStates.size(); i++)
        {
            cout << "\t" << sysStates(i);
        }
        cout << "\t" << euler(0) << "\t" << euler(1) << "\t" << euler(2);
        cout << endl;
        // save_file << rotor.getCurrentTime() << "," << sysStates(0) << "," << sysStates(1)  << endl;
        rotor.Integrate();
    }
    return 0;
}