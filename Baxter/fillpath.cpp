/*
 * fillpath.cpp
 *
 *  Created on: Jan 5, 2017
 *      Author: avishai
 */

#include "StateValidityChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>

// Prototypes
class fillpath : public StateValidityChecker
{
public:

	fillpath();

	void copyfiles(string from, string to);
	void copyfiles_withNum(string from, string to, int);

	void read_robot_paths(Matrix &path);

	void retrieve(State c, State &a, State &q1, State &q2);

	double distance(State a, State b);

	bool isValid(State c, int active_chain);

	bool checkMotion(State s1, State s2, int active_chain);

	int active_chain;
};

void fillpath::copyfiles(string from, string to) {
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

void fillpath::copyfiles_withNum(string from, string to, int num) {
	ifstream a;
	ofstream b;
	char ch;
	a.open(from); //The file from which the content will be copied
	b.open(to); //The file to which the content will be copied
	b << num << endl;

	while (!a.eof())
	{
		a.get(ch); //reading from file object 'a'
		b<<ch; //writing to file 'b'
	}
	a.close();
	b.close();
}

void fillpath::read_robot_paths(Matrix &path) {
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
			//fq >> ch;
			//cout << c_temp[j] << " ";
		}
		//cout << endl;
		path.push_back(c_temp);
	}
	fq.close();
	//printMatrix(path);
}

void fillpath::retrieve(State c, State &a, State &q1, State &q2) {
	for (int i = 0; i < 6; i++)
		a[i] = c[i];
	for (int i = 0; i < Nj; i++) {
		q1[i] = c[i+6];
		q2[i] = c[i+13];
	}
}

double fillpath::distance(State a, State b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}


// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool fillpath::isValid(State c, int active_chain) {

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

bool fillpath::checkMotion(State s1, State s2, int active_chain)
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

fillpath::fillpath() {

	//relax_joint_limits();

	State a(6), q1(Nj), q2(Nj), C(20), temp(3);

	// Make a copy of the original files
	copyfiles("./path/robot_paths.txt", "./path/robot_paths_B4fill.txt");
	copyfiles("./path/rod_path.txt", "./path/rod_path_B4fill.txt");
	copyfiles("./path/afile.txt", "./path/afile_B4fill.txt");

	//Read from robots_path.txt
	Matrix path;
	read_robot_paths(path);
	//printMatrix(path);

	// Open a_path file
	std::ofstream qfile, pfile, afile;
	qfile.open("./path/temp.txt", ios::out);
	pfile.open("./path/rod_path_temp.txt", ios::out);
	afile.open("./path/afile_temp.txt", ios::out);

	// First point
	retrieve(path[0], a, q1, q2);
	for (int j = 6; j < 20; j++) {
		qfile << path[0][j] << " ";
	}
	qfile << endl;

	for (int j = 0; j < 6; j++) {
		afile << a[j] << " ";
	}
	afile << endl;

	rod_solve(a);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	double dd = 0.2;
	int count = 1;
	for (int i = 1; i < path.size(); i++) {
		cout << (double)i/(path.size()-1)*100 << "% finished." << endl;

		active_chain = 0;
		bool valid = false;
		if (checkMotion(path[i-1],path[i],active_chain))
			valid = true;

		if (!valid) {
			active_chain = !active_chain;
			if (checkMotion(path[i-1],path[i],active_chain))
				valid = true;
		}

		/*active_chain = 0;
		bool valid;
		for (int t = 0; t < 2; t++) {
			valid = true;
			// Now write to file
			for (int j = 1; j < nd; j++) {
				// interpolate
				for (int k = 0; k<20; k++)
					C[k] = path[i-1][k] + ((double)j / (nd-1)) * (path[i][k]-path[i-1][k]);

				retrieve(C, a, q1, q2);
				if (!isRodFeasible(a)) {
					valid = false;
					cout << "Rod infeasible.\n";
					break;
				}

				if (!active_chain) {
					if (!calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, q2)) {
						valid = false;
						cout << "q1 at " << j << endl;
						break;
					}
					else
						q2 = get_IK_solution_q2();
				}
				else {
					if (!calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, q1)) {
						valid = false;
						cout << "q2 at " << j << endl;
						break;
					}
					else
						q1 = get_IK_solution_q1();
				}

				if (collision_state(getPMatrix(), q1, q2)) {
					valid = false;
					cout << "Collision at " << j << endl;
					break;
				}
			}
			if (!valid && t==0)
				active_chain = !active_chain;
		}*/

		if (!valid) {
			cout << "Failed!!!\n";
			exit(1);
		}

		double d = distance(path[i], path[i-1]);
		int nd = (2 < ceil(d / dd)) ? ceil(d/dd) : 2;

		cout << "Section " << i-1 << "-" << i << " passed." << endl;
		// Now write to file
		for (int j = 1; j < nd; j++) {
			// interpolate
			for (int k = 0; k < 20; k++)
				C[k] = path[i-1][k] + ((double)j / (nd-1)) * (path[i][k]-path[i-1][k]);

			retrieve(C, a, q1, q2);
			rod_solve(a);
			if (!active_chain) {
				if (!calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, q2)) {
					valid = false;
					cout << "q1 at " << j << endl;
					exit(1);
				}
				else
					q2 = get_IK_solution_q2();
			}
			else {
				if (!calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, q1)) {
					valid = false;
					cout << "q2 at " << j << endl;
					exit(1);
				}
				else
					q1 = get_IK_solution_q1();
			}
			
			for (int k = 0; k < Nj; k++) {
				qfile << q1[k] << " ";
			}
			for (int k = 0; k < Nj; k++) {
				qfile << q2[k] << " ";
			}
			qfile << endl;
			for (int k = 0; k < 6; k++) {
				afile << a[k] << " ";
			}
			afile << endl;

			rod_solve(a);
			// Log points on rod to file
			for (int k = 0; k < get_Points_on_Rod(); k++) {
				temp = getP(k);
				pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
			}
			pfile << endl;

			count++;
		}
	}

	qfile.close();
	pfile.close();
	afile.close();

	cout << "Saving to original files..." << endl;
	// Update file with number of conf.
	copyfiles_withNum("./path/temp.txt", "./path/robot_paths.txt", count);
	copyfiles_withNum("./path/rod_path_temp.txt", "./path/rod_path.txt", count*(get_Points_on_Rod()+1));
	copyfiles("./path/afile_temp.txt", "./path/afile.txt");

	std::remove("./path/temp.txt");
	std::remove("./path/rod_path_temp.txt");
	std::remove("./path/afile_temp.txt");
}

int main() {
	fillpath fp;

	return 0;
}
