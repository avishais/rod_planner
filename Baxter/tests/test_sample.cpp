#include "../validity_checkers/StateValidityChecker.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

void interpolate(StateVector S, double r, StateVector &Si) {

	for (int i = 0; i < S.a.size(); i++) {
		Si.a[i] = S.a[i] + r*(Si.a[i]-S.a[i]);
	}
	for (int i = 0; i < S.q.size(); i++) {
		Si.q[i] = S.q[i] + r*(Si.q[i]-S.q[i]);
	}

}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc;

	State a(6), q(14);
	StateVector S1(7), S2(7);


	cout << "Sampling ... \n";

	svc.GDsample(a, q);
	cout << "Sampled.\n";

	S1.copy(a, q);
	S1.print();

	svc.log_q(a, q);


	return 0;
}

