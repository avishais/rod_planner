
/* Author: Avishai Sintov */

// g++ ./run/runRandom.cpp -o Run -std=c++11

#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include <math.h>

using namespace std;

typedef vector<vector< double >> Matrix;
typedef vector<double> State;



Matrix load() {

	// Load all random confs.
	Matrix Cdb;
	std::ifstream File;
	File.open("./matlab/Benchmark_rand_sg_noObs.txt");
	State c_temp(20);
	int i = 0;
	while(!File.eof() && i < 1438) {
		for (int j=0; j < 20; j++) {
			File >> c_temp[j];
			//cout << c_temp[j] << " ";
		}
		//cout << i << endl;
		Cdb.push_back(c_temp);
		i++;
	}
	File.close();
	Cdb.pop_back();

	return Cdb;
}

void write(State v) {
	std::ofstream ft;
	ft.open("./matlab/Benchmark_rand_sg_noObs_new.txt", ios::app);

	for (int i = 0; i < v.size(); i++)
		ft << v[i] << " ";
	ft << endl;

	ft.close();
}

/*
int main(int argn, char ** args) {
	double runtime = 1000;

	if (argn == 2)
		runtime = atof(args[1]);

	Matrix D = load();

	for (int i = 0; i < D.size(); i++) {
		State v = D[i];
		cout << v[0] << " " << v[1] << " " << v[2] << " " << v[5] << endl;

		if ( v[0] == 500 && (v[1] == 5 || v[1] == 6 || v[1] == 3) && !v[5] ) {
			if (v[1] == 3)
				runtime = 300;
			else
				runtime = 2000;
			string com = "./pln " + std::to_string((int)runtime) + " " + std::to_string(1) + " " + std::to_string(1) + " " + std::to_string((int)v[2]);
			const char *y = com.c_str();
			system(y);
		}
		else
			write(v);
	}

	return 0;
}
*/

// Just run queries in order
int main(int argn, char ** args) {
	double runtime = 1000;

	if (argn == 2)
		runtime = atof(args[1]);


	int N = 100;
	for (int i = 11; i < N; i++) {

		for( int j = 1; j < 2; j++) {
			for (int k = 1; k < 2; k++) {

				if (j==2 && k==4)
					continue;

				// run trial
				string com = "./pln " + std::to_string(runtime) + " " + std::to_string(j) + " " + std::to_string(k) + " " + std::to_string(i);
				const char *y = com.c_str();
				system(y);
			}
		}

		cout << "**************************************" << endl;
		cout << "Completed " << (double)i/N*100 << "%." << endl;
		cout << "**************************************" << endl;
	}

	std::cout << std::endl << std::endl;

	return 0;
}


