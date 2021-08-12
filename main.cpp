#include "rungekutta.h"
// #include <fstream>
// #include <string>

int main(){
    // output simulation results
    // string save_path = "/Users/zhongjiaoshi/VSCode/result.csv"; 
    // ofstream save_file(save_path);
    // save_file <<"time,pos,vel"<<endl;

    // system dimensions
    int num_states = 4;

    // config system 
    con_para conpara;
    conpara.time_step      = 0.01;
    conpara.initial_time   = 0.0;
    conpara.initial_states = VectorXd(num_states);
    conpara.initial_states << 1.0, 0.0, 0.0, 0.0;

    // constuct simulation instance
    RungeKutta rotor = RungeKutta(conpara);
    
    // simulation final time
    const double time_final = 10.0;

    // system states
    VectorXd sysStates(conpara.initial_states.size());

    // config output format
    cout.precision(6);
    cout.flags(ios::fixed);
    cout.setf(ios::right);

    // start simulation
    while (rotor.getCurrentTime() < time_final){
        sysStates = rotor.getCurrentStates();
        cout << rotor.getCurrentTime() << "\t" << sysStates(0) << "\t" << sysStates(1) << endl;
        // save_file << rotor.getCurrentTime() << "," << sysStates(0) << "," << sysStates(1)  << endl;
        rotor.Integrate();
    }
    return 0;
}