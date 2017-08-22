/*
 * smooth.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: avishai
 */

#include "StateValidityChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>

// Prototypes
class smooth : public StateValidityChecker
{
public:

	smooth(const int N);

	void copyfiles(string from, string to);
	void copyfiles_withNum(string from, string to, int);

	void read_robot_paths(Matrix &path);

	void retrieve(Vector c, Vector &a, Vector &q1, Vector &q2);

	double distance(Vector a, Vector b);

	Vector identify_state_ik(Vector c);

	void determine_active_ik(Vector, Vector, int);

	bool isValid(Vector c, int active_chain, int IK_sol);

	bool checkMotion(Vector s1, Vector s2, int active_chain, int ik_sol);

	bool check_neighbors(Vector s1, Vector s2, int active_chain);

	void erase(Matrix &C, int n1, int n2);

	void save2file(Matrix mpath);
};

void smooth::read_robot_paths(Matrix &path) {
	std::ifstream fq,fa;
	fq.open("./path/robot_paths.txt");
	fa.open("./path/afile.txt");

	char ch;
	Vector c_temp(18);

	int lines;
	fq >> lines; // Number of conf.
	cout << lines << endl;

	for (int i = 0; i < lines; i++) {
		for (int j=0; j < 6; j++)
			fa >> c_temp[j];
		for (int j=6; j < 18; j++) {
			fq >> c_temp[j];
			//fq >> ch;
		}
		path.push_back(c_temp);
	}
	fq.close();

	//printMatrix(path);
}

void smooth::retrieve(Vector c, Vector &a, Vector &q1, Vector &q2) {
	for (int i = 0; i < 6; i++) {
		a[i] = c[i];
		q1[i] = c[i+6];
		q2[i] = c[i+12];
	}
}

Vector smooth::identify_state_ik(Vector C) {

	Vector a(6), q1(6), q2(6), q_temp(6), ik(2);
	retrieve(C, a, q1, q2);

	ik = {-1, -1};

	if (!isRodFeasible(a)) {
		cout << "Failed a.\n";
		return ik;
	}
	Matrix Q = getT(get_Points_on_Rod()-1);

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	//printMatrix(get_FK_solution_T1());
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	if (n == 0) {
		return ik;
	}
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_2(i);
		if (normDistance(q_temp,q2)<1e-1) {
			ik[0] = get_valid_IK_solutions_indices_2(i);
			break;
		}
	}

	// If match was not found, find the closest
	if (ik[0]==-1) {
		int ik_min = 20;
		double minD = 1000;
		double D;
		for (int i = 0; i < n; i++) {
			q_temp = get_all_IK_solutions_2(i);
			D = normDistance(q_temp,q2);
			if (D < minD) {
				ik_min = get_valid_IK_solutions_indices_2(i);
				minD = D;
			}
		}
		if (minD>0.85)
			return ik;
		else
			ik[0] = ik_min;
	}

	// q2 is the active chain
	Matrix Tinv = Q;
	InvertMatrix(Q, Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	n = calc_all_IK_solutions_1(T1);
	if (n == 0) {
		return ik;
	}
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_1(i);
		if (normDistance(q_temp,q1)<1e-1) {
			ik[1] = get_valid_IK_solutions_indices_1(i);
			break;
		}
	}

	// If match was not found, find the closest
	if (ik[1]==-1) {
		int ik_min = 20;
		double minD = 1000;
		double D;
		for (int i = 0; i < n; i++) {
			q_temp = get_all_IK_solutions_1(i);
			D = normDistance(q_temp,q1);
			if (D < minD) {
				ik_min = get_valid_IK_solutions_indices_1(i);
				minD = D;
			}
		}

		if (minD>0.85)
			return ik;
		else
			ik[1] = ik_min;
	}

	return ik;
}

double smooth::distance(Vector a, Vector b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool smooth::isValid(Vector c, int active_chain, int IK_sol) {

	Vector a(6), q1(6), q2(6);
	retrieve(c, a, q1, q2);

	if (!isRodFeasible(a))
		return false;

	bool valid = false;
	switch (active_chain) {
	case 0:
		if (calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, IK_sol)) {
			Vector q_IK = get_IK_solution_q2();
			if (!collision_state(getPMatrix(), q1, q_IK))
				valid = true;
		}
		break;
	case 1:
		if (calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, IK_sol)) {
			Vector q_IK = get_IK_solution_q1();
			if (!collision_state(getPMatrix(), q_IK, q2))
				valid = true;
		}
	}
	return valid;
}

bool smooth::checkMotion(Vector s1, Vector s2, int active_chain, int ik_sol)
{
	//if (!check_neighbors(s1,s2,active_chain))
		//return false;

	// We assume motion starts and ends in a valid configuration - due to projection
	double dd = 0.25;
	double d = distance(s1, s2);
	int nd = ceil(d / dd);
	Vector C(18);

	for (int j = 1; j < nd; j++) {
		// interpolate
		for (int k = 0; k<18; k++)
			C[k] = s1[k] + ((double)j / (nd-1)) * (s2[k]-s1[k]);

		if (!isValid(C, active_chain, ik_sol))
			return false;
	}

	return true;
}

bool smooth::check_neighbors(Vector s1, Vector s2, int active_chain) {

	Vector a(6), q1n(6), q2n(6), q1(6), q2(6);

	retrieve(s1, a, q1, q2);
	retrieve(s2, a, q1n, q2n);

	if (!active_chain) {
		if (q2n[3]*q2[3] < 0)
			if (fabs(q2n[3]-q2[3]) > 3.14)
				return false;
		if (q2n[5]*q2[5] < 0)
			if (fabs(q2n[5]-q2[5]) > 3.14)
				return false;
		if (q2n[0]*q2[0] < 0)
			if (fabs(q2n[0]-q2[0]) > 3.14)
				return false;
	}
	else {
		if (q1n[3]*q1[3] < 0)
			if (fabs(q1n[3]-q1[3]) > 3.14)
				return false;
		if (q1n[5]*q1[5] < 0)
			if (fabs(q1n[5]-q1[5]) > 3.14)
				return false;
		if (q1n[0]*q1[0] < 0)
			if (fabs(q1n[0]-q1[0]) > 3.14)
				return false;
	}

	return true;
}

void smooth::erase(Matrix &C, int n1, int n2) {
	// Erase cells between n1 to n2, not including n1 and n2.
	C.erase(C.begin()+min(n1,n2)+1, C.begin()+max(n1,n2));
}

void smooth::save2file(Matrix mpath) {

	cout << "Saving smoothed path..." << endl;

	Vector a(6), q1(6), q2(6);
	int active_chain, ik_sol;

	{
		// Open a_path file
		std::ofstream myfile, afile, pfile;
		myfile.open("./path/robot_paths.txt");
		afile.open("./path/afile.txt");
		pfile.open("./path/rod_path.txt");

		myfile << mpath.size() << endl;
		pfile << mpath.size()*501 << endl;

		Vector temp;
		for (int i = 0 ; i < mpath.size() ; - ++i) {
			retrieve(mpath[i], a, q1, q2);

			for (int j = 0; j<6; j++) {
				myfile << q1[j] << " ";
				afile << a[j] << " ";
			}
			for (int j = 0; j<6; j++) {
				myfile << q2[j] << " ";
			}
			myfile << endl;
			afile << endl;

			rod_solve(a);
			// Log points on rod to file
			for (int k = 0; k < get_Points_on_Rod(); k++) {
				temp = getP(k);
				pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
			}
			pfile << endl;
		}

		myfile.close();
		afile.close();
		pfile.close();
	}
}

void smooth::copyfiles(string from, string to) {
	ifstream a;
	ofstream b;
	char ch;
	a.open(from); //The file from which the content will be copied
	b.open(to); //The file to which the content will be copied
	while (!a.eof())
	{
		a.get(ch); //reading from file object 'a'
		b<<ch; //writing to file 'b'
	}
	a.close();
	b.close();
}

smooth::smooth(const int N) {

	// Make a copy of the original files
	copyfiles("./path/robot_paths.txt", "./path/robot_pathsB4smooth.txt");
	copyfiles("./path/rod_path.txt", "./path/rod_pathB4smooth.txt");
	copyfiles("./path/afile.txt", "./path/afileB4smooth.txt");

	//Read from robots_path.txt
	Matrix path, IK_list;
	read_robot_paths(path);

	cout << path.size() << endl;
	int n1, n2, active_chain;
	Vector ik1(2), ik2(2);

	for (int i = 0; i < N; i++) {
		cout << (double)i/(N-1)*100 << "% finished with " << path.size() << " nodes left." << endl;

		do {
			n1 = rand() % path.size();
			n2 = rand() % path.size();
		} while (abs(n2-n1)<=1);
		cout << n1 << " " << n2 << endl;

		Vector c1 = path[n1];
		Vector c2 = path[n2];
		//printVector(c1);
		//printVector(c2);

		ik1 = identify_state_ik(c1);
		ik2 = identify_state_ik(c2);

		if (ik1[0]==-1 || ik2[0]==-1 || ik1[1]==-1 || ik2[1]==-1) {
			cout << "Error in IK identificationn\n";
			return;
		}

		printVector(ik1);
		printVector(ik2);

		bool valid = false;
		if (ik1[0]==ik2[0]) {
			active_chain = 0;

			if (checkMotion(c1,c2,active_chain,ik1[0]))
				valid  = true;
		}

		if (!valid && ik1[1]==ik2[1]) {
			active_chain = 1;

			if (checkMotion(c1,c2,active_chain,ik1[1]))
				valid = true;
		}

		//cout << "Valid: " << valid << endl;

		if (!valid)
			continue;

		// cut in between
		erase(path,n1,n2);
	}

	save2file(path);
	cout << path.size() << endl;
}


int main(int argn, char ** args) {
	int num;

	if (argn == 1)
		num = 20;
	else
		num = atoi(args[1]);

	smooth S(num);


	return 0;
}
