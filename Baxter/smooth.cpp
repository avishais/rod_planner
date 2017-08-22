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

	void retrieve(State c, State &a, State &q1, State &q2);

	double distance(State a, State b);

	bool isValid(State c, int active_chain);

	bool checkMotion(State s1, State s2, int active_chain);

	bool check_neighbors(State s1, State s2, int active_chain);

	void erase(Matrix &C, int n1, int n2);

	void save2file(Matrix mpath);
};

void smooth::read_robot_paths(Matrix &path) {
	std::ifstream fq,fa;
	fq.open("./path/robot_paths.txt");
	fa.open("./path/afile.txt");

	char ch;
	State c_temp(20);

	int lines;
	fq >> lines; // Number of conf.
	cout << lines << endl;

	for (int i = 0; i < lines; i++) {
		for (int j=0; j < 6; j++)
			fa >> c_temp[j];
		for (int j=0; j < 14; j++) {
			fq >> c_temp[j+6];
		}
		path.push_back(c_temp);
	}
	fq.close();
	//printMatrix(path);
}

void smooth::retrieve(State c, State &a, State &q1, State &q2) {
	for (int i = 0; i < 6; i++)
		a[i] = c[i];
	for (int i = 0; i < Nj; i++) {
		q1[i] = c[i+6];
		q2[i] = c[i+13];
	}
}

double smooth::distance(State a, State b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool smooth::isValid(State c, int active_chain) {

	State a(6), q1(Nj), q2(Nj);
	retrieve(c, a, q1, q2);

	if (!isRodFeasible(a))
		return false;

	bool valid = false;
	switch (active_chain) {
	case 0:
		if (calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, q2)) {
			q2 = get_IK_solution_q2();
			if (!collision_state(getPMatrix(), q1, q2))
				valid = true;
		}
		break;
	case 1:
		if (calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, q1)) {
			q1 = get_IK_solution_q1();
			if (!collision_state(getPMatrix(), q1, q2))
				valid = true;
		}
	}
	return valid;
}

bool smooth::checkMotion(State s1, State s2, int active_chain)
{
	//if (!check_neighbors(s1,s2,active_chain))
		//return false;

	// We assume motion starts and ends in a valid configuration - due to projection
	double dd = 0.2;
	double d = distance(s1, s2);
	int nd = ceil(d / dd);
	State C(20);

	for (int j = 1; j < nd; j++) {
		// interpolate
		for (int k = 0; k<20; k++)
			C[k] = s1[k] + ((double)j / (nd-1)) * (s2[k]-s1[k]);

		if (!isValid(C, active_chain))
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

	State a(6), q1(Nj), q2(Nj);
	int active_chain;

	{
		// Open a_path file
		std::ofstream myfile, afile, pfile;
		myfile.open("./path/robot_paths.txt");
		afile.open("./path/afile.txt");
		pfile.open("./path/rod_path.txt");

		myfile << mpath.size() << endl;
		pfile << mpath.size()*501 << endl;

		State temp;
		for (int i = 0 ; i < mpath.size() ; - ++i) {
			retrieve(mpath[i], a, q1, q2);

			for (int j = 0; j<6; j++)
				afile << a[j] << " ";
			for (int j = 0; j<Nj; j++)
				myfile << q1[j] << " ";
			for (int j = 0; j<Nj; j++)
				myfile << q2[j] << " ";
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

	bool flag = true;
	while (flag) {
		flag = false;
		for (int i = 0; i < N; i++) {
			cout << (double)i/(N-1)*100 << "% finished with " << path.size() << " nodes left." << endl;

			do {
				n1 = rand() % path.size();
				n2 = rand() % path.size();
			} while (abs(n2-n1)<=1);
			cout << n1 << " " << n2 << endl;

			State c1 = path[n1];
			State c2 = path[n2];
			//printVector(c1);
			//printVector(c2);

			bool valid = false;
			if (checkMotion(c1,c2,0))
				valid  = true;

			if (!valid)
				if (checkMotion(c1,c2,1))
					valid = true;

			if (!valid)
				continue;

			// cut in between
			erase(path,n1,n2);

			//if (path.size() < 30)
			//	break;

			flag = true;
		}
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
