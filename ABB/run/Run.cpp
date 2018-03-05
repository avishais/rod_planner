
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


int main(int argn, char ** args) {
	double runtime = 1000;

	if (argn == 2)
		runtime = atof(args[1]);

	vector<int> ms_size = {100, 500, 1000};
	vector<int> knn_size = {2, 3, 4, 5, 6};

	int mode = 3;
	switch (mode) {
	case 1 : {
		int j = 1;
		int k = 2;

		string com = "./pln " + std::to_string(runtime) + " " + std::to_string(ms_size[j]) + " " + std::to_string(knn_size[k]);
		const char *y = com.c_str();
		system(y);

		break;
	}
	case 2 : { // Benchmark with the same scenario and PRMmaps over N trials
		/*std::ofstream ft;
		ft.open("./matlab/Benchmark_poleScene.txt", ios::app);

		int num_nn = 1;
		int ms_size = 100;
		int knn_size = 5;
		string PRMfile = "ms6D_" + std::to_string(ms_size) + "_"  + std::to_string(knn_size) + ".prm";
		cout << "*** Planning with " << PRMfile << endl;

		int N = 100;
		for (int i = 0; i < N; i++) {
			cout << "**************************************" << endl;
			cout << "Completed " << (double)i/N*100 << "%." << endl;
			cout << "**************************************" << endl;
			Plan.plan(c_start, c_goal, runtime, PRMfile, num_nn);

			// Log
			ft << ms_size << "\t" << knn_size << "\t";
			ifstream FromFile;
			FromFile.open("./paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				ft << line << "\t";
			FromFile.close();
			ft << endl;
		}
		ft.close();*/
		break;
	}
	case 3 : { // Benchmark with the same scenario and different PRMmaps over N trials

		int N = 1;
		for (int i = 0; i < N; i++) {

			for( int j = 0; j < ms_size.size(); j++) {
				for (int k = 0; k < knn_size.size(); k++) {

					if (ms_size[j]==1000 && knn_size[k]==6)
						continue;

					// run trial
					string com = "./pln " + std::to_string(runtime) + " " + std::to_string(ms_size[j]) + " " + std::to_string(knn_size[k]);
					const char *y = com.c_str();
					system(y);
				}
			}

			cout << "**************************************" << endl;
			cout << "Completed " << (double)i/N*100 << "%." << endl;
			cout << "**************************************" << endl;
		}
		break;
	}
	}

	return 0;
}


