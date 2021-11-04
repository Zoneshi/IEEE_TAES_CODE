#include "RotorCraft.h"

int main()
{
	// const string SavePath = "../adaptive_data.dat";
	// const string SavePath = "../simple_data.dat";
	const string SavePath = "../fixed_data.dat";
	ofstream ofile(SavePath);

	const double TimeStep = 0.005;
	const double FinalTime = 40.0;
	const double RecTimeStep = 0.1;
	const int RecInterStep = (int)(RecTimeStep / TimeStep);
	const Vector3d pos0 = Vector3d(400.0, 0.0, 0.0);
	const Vector3d vel0 = Vector3d(0.0, 20.0*M_PI, 10.0 * M_PI);
	const Vector3d eul0 = deg2rad(Vector3d(0.0, 0.0, 0.0));
	const Vector3d omega0(0.0, 0.0, 0.0);   

	VectorXd SysStates = GenerateInitialStates(pos0, vel0, eul0, omega0);

	double t = 0.0;
	int iter_num = 0;

	while (t <= FinalTime)
	{
		if ((iter_num % RecInterStep) == 0)
		{
			WriteDataToFile(t, SysStates, ofile);
		}
		RungeKuttaIntegral(ClosedLoopEquation, t, SysStates, TimeStep);
		iter_num = iter_num + 1;
	}
	ofile.close();

	return 0;
}