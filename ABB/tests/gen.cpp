#include "../validity_checkers/StateValidityChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>

class gen : public StateValidityChecker
{
public:

	gen(string);

	StateVector sample();

	void log(StateVector);

	void log_sim(State, State);

	int ms_size, subms_size;
};

gen::gen(string prmfile) {
	load_data(prmfile);
	ms_size = ms.milestones.size();
	subms_size = subms.sub_milestones.size();
}


StateVector gen::sample() {
	int random_index;
	State q(nq), a(na);

	bool flag = false;

	while (!flag) {
		// Random feasible rod conf.
		if (rand() % 10 < 3)
			random_index = rand() % ms_size;
		else
			random_index = rand() % subms_size;
		a = subms.sub_milestones[random_index];

		for (int i = 0; i < 10; i++) {
			// Random joint angles
			for (int i = 0; i < nq; i++)
				q[i] = ((double) rand() / (RAND_MAX)) * 2 * PI_ - PI_;

			if (GDproject(a, q)) {
				flag = true;
				break;
			}
		}
	}

	StateVector S(nq/2);
	S.copy(a, q);

	return S;
}

void gen::log(StateVector S) {

	std::ofstream ft;
	ft.open("../data/abb_samples_data.txt", ios::app);

	for (int i = 0; i < na; i++)
		ft << S.a[i] << " ";
	for (int i = 0; i < nq; i++)
		ft << S.q[i] << " ";
	ft << endl;

	ft.close();
}

void gen::log_sim(State a, State q) {
	std::ofstream qfile, afile, pfile, ai;
	qfile.open("../paths/path.txt");
	afile.open("../paths/afile.txt");
	pfile.open("../paths/rod_path.txt");

	qfile << 1 << endl;
	pfile << 501 << endl;

	for (int j = 0; j < na; j++)
		afile << a[j] << " ";
	for (int j = 0; j < nq; j++)
		qfile << q[j] << " ";

	rod_solve(a);
	State temp(3);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	qfile.close();
	afile.close();
	pfile.close();
}



int main(int argn, char ** args) {
	int N;

	if (argn == 1)
		N = 1; // samples
	else
		N = atoi(args[1]);

	srand( time(NULL) );

	gen G("ms6D_500_4.prm");

	for (int i = 0; i < N; i++) {

		StateVector S = G.sample();

		G.log_sim(S.a, S.q);

		G.log(S);

		cout << "Generated " << i+1 << " out of " << N << " samples." << endl;
	}

	return 0;
}
