//#include "../../roadmap/classes/Rod_ODE_class.h"
//#include "../classes/robots_class.h"
//#include "../classes/collisionDetection.h"
#include "../StateValidityChecker.h"

#include <iostream>
#include <fstream>
#include <time.h>       /* time */

#define PI 3.1416

using namespace std;

StateValidityChecker sv;

void sample_a(State &a_rand) {
	for (int i=0; i<6; i++)
		a_rand[i] = -30.0 + ((double)rand() / RAND_MAX)*60; // Random number in [-30,30]
}

int main(int argn, char ** args) {
	int N; // Number of random points to add.
	if (argn == 1)
		N = 5; // sec
	else {
		N = atof(args[1]);
	}
	
	srand (time(NULL));
	
	State a(6);
	State q1(7), q2(7);
	
	ofstream F;
	
	int i = 0;
	while (i<N) {
		sample_a(a);
		if (!sv.generate_feasible_config(a, q1, q2))
			continue;

		F.open ("randomnodes_contGrasp_wall.txt", ios::app);
		for (int j = 0; j < 6; j++)
			F << a[j] << " ";
		for (int j = 0; j < 7; j++)
			F << q1[j] << " ";
		for (int j = 0; j < 7; j++)
			F << q2[j] << " ";
		F << endl;
		F.close();
		
		cout << "Added new node # " << ++i << "." << endl;
	}
}
