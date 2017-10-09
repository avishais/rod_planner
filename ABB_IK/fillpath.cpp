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

	void retrieve(Vector c, Vector &a, Vector &q1, Vector &q2);

	double distance(Vector a, Vector b);

	Vector identify_state_ik(Vector c);

	void determine_active_ik(Vector, Vector, int);

	int active_chain;
	int ik_sol;
	int ik_sol_alter;

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
	Vector c_temp(18);

	int lines;
	fq >> lines; // Number of conf.

	for (int i = 0; i < lines; i++) {
		for (int j=0; j < 6; j++)
			fa >> c_temp[j];
		for (int j=6; j < 18; j++) {
			fq >> c_temp[j];
			//fq >> ch;
			//cout << c_temp[j] << " ";
		}
		//cout << endl;
		path.push_back(c_temp);
	}
	fq.close();
	//printMatrix(path);
}

void fillpath::retrieve(Vector c, Vector &a, Vector &q1, Vector &q2) {
	for (int i = 0; i < 6; i++) {
		a[i] = c[i];
		q1[i] = c[i+6];
		q2[i] = c[i+12];
	}
}

double fillpath::distance(Vector a, Vector b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}

Vector fillpath::identify_state_ik(Vector C) {

	Vector a(6), q1(6), q2(6), q_temp(6), ik(2);
	retrieve(C, a, q1, q2);

	ik = {-1, -1};

	if (!isRodFeasible(a))
		return ik;
	Matrix Q = getT(get_Points_on_Rod()-1);

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	//printMatrix(get_FK_solution_T1());
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	if (n == 0)
		return ik;
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
	if (n == 0)
		return ik;
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

void fillpath::determine_active_ik(Vector st1, Vector st2, int inx) {

	Vector ik1(2), ik2(2);

	ik1 = identify_state_ik(st1);
	ik2 = identify_state_ik(st2);
	printVector(ik1);
	printVector(ik2);

	if (ik1[0]==-1 || ik1[1]==-1 || ik2[0]==-1 || ik2[1]==-1){
		cout << "Error!!!\n";
		exit(1);
	}

	ik_sol_alter = -1;

	int count = 0;
	if (ik1[0]==ik2[0]) {
		active_chain = 0;
		ik_sol = ik1[0];
		count++;
	}
	if (ik1[1]==ik2[1] && count==0) {
		active_chain = 1;
		ik_sol = ik1[1];
		return;
	}
	if (ik1[1]==ik2[1] && count==1) {
		ik_sol_alter = ik1[1];
		return;
	}

	if (ik1[1]!=ik2[1] && count==0) {
		cout << "No common IK with nodes " << inx << " and " << inx-1 << ". Oh oh...\n";
		/*cout << ik1[0] << " " << ik1[1] << endl;
		cout << ik2[0] << " " << ik2[1] << endl;
		printVector(st1);
		printVector(st2);*/
		exit(1);
	}
}


fillpath::fillpath() {

	relax_joint_limits();

	Vector a(6), q1(6), q2(6), ik(2), ikp(2), C(18), temp(3);

	// Make a copy of the original files
	copyfiles("./path/robot_paths.txt", "./path/robot_paths_B4fill.txt");
	copyfiles("./path/rod_path.txt", "./path/rod_path_B4fill.txt");

	//Read from robots_path.txt
	Matrix path;
	read_robot_paths(path);

	// Open a_path file
	std::ofstream qfile, pfile;
	qfile.open("./path/temp.txt", ios::out);
	pfile.open("./path/rod_path_temp.txt", ios::out);

	// First point
	retrieve(path[0], a, q1, q2);
	for (int j = 6; j < 18; j++) {
		qfile << path[0][j] << " ";
	}
	qfile << endl;

	rod_solve(a);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	cout << "Original size: " << path.size() << endl;

	// Rest of points
	double dd = 0.25;
	int count = 1;
	for (int i = 1; i < path.size(); i++) {
		cout << (double)i/(path.size()-1)*100 << "% finished." << endl;
		double d = distance(path[i], path[i-1]);
		int nd = (2 < ceil(d / dd)) ? ceil(d/dd) : 2;

		determine_active_ik(path[i], path[i-1], i);

		cout << i << " " << dd << " " << nd << " " << active_chain << " " << ik_sol << " " << ik_sol_alter << endl;
		//printVector(path[i-1]);
		//printVector(path[i]);

		bool valid;
		for (int t = 0; t < 2; t++) {
			valid = true;
			for (int j = 1; j < nd; j++) {
				// interpolate
				for (int k = 0; k<18; k++)
					C[k] = path[i-1][k] + ((double)j / (nd-1)) * (path[i][k]-path[i-1][k]);

				retrieve(C, a, q1, q2);
				if (!isRodFeasible(a)) {
					valid = false;
					cout << "a\n";
					break;
				}

				if (!active_chain) {
					if (!calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, ik_sol)) {
						valid = false;
						cout << "q1 at " << j << endl;
						break;
					}
					else
						q2 = get_IK_solution_q2();
				}
				else {
					if (!calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, ik_sol)) {
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

			// If failed, check the alternative active chain if exists
			if (!valid && ik_sol_alter != -1) {
				active_chain = !active_chain;
				ik_sol = ik_sol_alter;
			}
			else
				break;
		}

		if (!valid) {
			cout << "Failed!!!\n";
			exit(1);
		}

		// Now write to file
		for (int j = 1; j < nd; j++) {
			// interpolate
			for (int k = 0; k<18; k++)
				C[k] = path[i-1][k] + ((double)j / (nd-1)) * (path[i][k]-path[i-1][k]);

			retrieve(C, a, q1, q2);
			isRodFeasible(a);
			if (!active_chain) {
				calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, ik_sol);
				q2 = get_IK_solution_q2();
			}
			else {
				calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, ik_sol);
				q1 = get_IK_solution_q1();
			}

			for (int j = 0; j<6; j++) {
				qfile << q1[j] << " ";
			}
			for (int j = 0; j<6; j++) {
				qfile << q2[j] << " ";
			}
			qfile << endl;

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

	// Update file with number of conf.
	copyfiles_withNum("./path/temp.txt", "./path/robot_paths.txt", count);
	copyfiles_withNum("./path/rod_path_temp.txt", "./path/rod_path.txt", count*(get_Points_on_Rod()+1));

	std::remove("./path/temp.txt");
	std::remove("./path/rod_path_temp.txt");
}

int main() {
	fillpath fp;

	return 0;
}
