#include "../proj_classes/kdl_class.h"
#include "../../roadmap/classes/Rod_ODE_class.h"
#include "../validity_checkers/collisionDetection.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

double dist(State p1, State p2) {
	double sum = 0;
	for (int i = 0; i < p1.size(); i++)
		sum += (p1[i]-p2[i])*(p1[i]-p2[i]);

	return sqrt(sum);
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	kdl K(518.09);
	rod_ode R;
	collisionDetection C;

	State a = {1.13317, -4.08401, 2.74606, 6.78602, 11.6337, -5.10359};
	R.rod_solve(a);
	Matrix T = R.getT(R.get_Points_on_Rod()-1);
	K.printMatrix(T);

	double L = 150.0;
	//Matrix T = {{1,0,0,L},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

	std::ofstream pfile;
	pfile.open("../paths/rod_path.txt");
	pfile << 501 << endl;
	State temp(3);
	double dl = L / 500;
	// Log points on rod to file
	for (int k = 0; k < 500; k++) { //R.get_Points_on_Rod()
		temp = R.getP(k);
		//temp = {k*dl, 0, 0};
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;
	pfile.close();

	State q(14, 0);
	//q[0] = -PI/2;
	//q[3] = -PI/2;
	//q = {0,0,0,0,0,0,0};

	while (1) {
		q = K.rand_q(14);
		if (!K.GD(q, T)) {
			//cout << "Failed.\n";
			continue;
		}
		q = K.get_GD_result();
		if (!C.collision_state(R.getPMatrix(), q))
			break;
	}
	K.FK(q, T);
	K.printMatrix(K.get_FK_solution());

	std::ofstream myfile;
	myfile.open("../paths/path.txt");
	myfile << 1 << endl;
	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;
	myfile.close();

	cout << C.collision_state(R.getPMatrix(), q) << endl;
}

