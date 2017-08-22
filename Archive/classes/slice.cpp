/*
 * Slice.cpp
 *
 *  Created on: Feb 3, 2017
 *      Author: avishai
 */
#include "slice.h"

double round_to_digits(double value, int digits)
{
	if (value == 0.0) // otherwise it will return 'nan' due to the log10() of zero
		return 0.0;

	double factor = pow(10.0, digits - ceil(log10(fabs(value))));
	return round(value * factor) / factor;
}

// --------------------------------------------------------------

// Constructor
Slice::Slice() {
	a_interpolated.resize(6);
	a_start_.resize(6);
	a_goal_.resize(6);

	//FindStable_A_Path();
	//FindStable_A_Path_w_Avg();

	//for (int i = 0; i < a_steps; i++) {
	//	printVector(A_Path[i].a);
		//printMatrix(A_Path[i].P);
		//cout << A_Path[i].s << endl;
	//}
}

void Slice::init_A_Path(int n) {
	A_Path.resize(n);

	for (int i = 0; i < n; i++)
		A_Path[i].alloc(get_Points_on_Rod());
}

//bool flag = false;
// Computes a path in a-Slice based on the edge of stability
void Slice::FindStable_A_Path() {

	int n = max( (int)round( normDistance(a_start_,a_goal_) / resolution ) , 6 ); // if n<6, cs goes to Inf and an error occurs
	cout << "Computing A_path with n = " << n << " nodes..." << endl;
	init_A_Path(n);
	dap1 = 1.0/n;

	// Safety curve
	int n13 = (int)floor(n/3);
	int n23 = (int)floor(2*n/3);
	State cs = {((safety_factor - 1)*(n - n13 - n23 + 1))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			-((safety_factor - 1)*(n*n + n - n13*n13 - n13*n23 - n23*n23 + 1))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			((safety_factor - 1)*(n*n*n13 + n*n*n23 - n*n13*n13 - n*n13*n23 + n*n13 - n*n23*n23 + n*n23 - n13*n13 - n13*n23 + n13 - n23*n23 + n23))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			(n13*n23 + safety_factor*n*n - n13*n23*n23 - n13*n13*n23 + n13*n13*n23*n23 + safety_factor*n*n13*n13 - safety_factor*n*n*n13 + safety_factor*n*n23*n23 - safety_factor*n*n*n23 - n*n13*n23*n23 - n*n13*n13*n23 + n*n*n13*n23 - safety_factor*n*n13 - safety_factor*n*n23 + n*n13*n23 + safety_factor*n*n13*n23)/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1))};

	double SF, l;
	int s, conj_point, j;
	for (int i = 1; i <= n; i++) {
		interpolate(a_start_, a_goal_, (double)(i-1)/(double)(n-1)); // Compute point on line to a_interpolated
		conj_point = conjugate_point(a_interpolated); // Solve ode for a_interpolated

		// Compute scale factor 'l' of a_interpolated
		SF = cs[0]*i*i*i + cs[1]*i*i + cs[2]*i + cs[3];
		s = round_to_digits(SF*conj_point, 3); // was floor
		A_Path[i-1].s = s;
		l = (double)s / (double)get_Points_on_Rod();

		A_Path[i-1].T_end_of_rod = getT(s-1); // Store matrix for end of scaled rod (at s position on original rod)
		A_Path[i-1].P = getPMatrix();

		// Store all feasible matrices
		for (j = 0; j < s; j++)
			A_Path[i-1].T[j] = getT(j);

		// Apply scale factor and store in A_path, and scale T's xyz position
		for (j = 0; j < 3; j++) {
			A_Path[i-1].a[j] = a_interpolated[j]*l;
			A_Path[i-1].a[j+3] = a_interpolated[j+3]*l*l;
			A_Path[i-1].T_end_of_rod[j][3] /= l;
			for (int k = 0; k < A_Path[i-1].T.size(); k++)
				A_Path[i-1].T[k][j][3] /= l;
		}
	}
	a_steps = n;
}

//bool flag = false;
// Computes a path in a-Slice based on the edge of stability
// Here we resolve the scaled 'a' to acquire an exact solution (when scaling a large error occurs)
// Also, we don't store the T matrix along the rod, only the tip matrix
void Slice::FindStable_A_Path_Exact() {

	int n = max( (int)round( normDistance(a_start_,a_goal_) / resolution ) , 6 ); // if n<6, cs goes to Inf and an error occurs
	cout << "Computing A_path with n = " << n << " nodes..." << endl;
	init_A_Path(n);
	dap1 = 1.0/n;
	State a_scaled(6);

	// Safety curve
	int n13 = (int)floor(n/3);
	int n23 = (int)floor(2*n/3);
	State cs = {((safety_factor - 1)*(n - n13 - n23 + 1))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			-((safety_factor - 1)*(n*n + n - n13*n13 - n13*n23 - n23*n23 + 1))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			((safety_factor - 1)*(n*n*n13 + n*n*n23 - n*n13*n13 - n*n13*n23 + n*n13 - n*n23*n23 + n*n23 - n13*n13 - n13*n23 + n13 - n23*n23 + n23))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			(n13*n23 + safety_factor*n*n - n13*n23*n23 - n13*n13*n23 + n13*n13*n23*n23 + safety_factor*n*n13*n13 - safety_factor*n*n*n13 + safety_factor*n*n23*n23 - safety_factor*n*n*n23 - n*n13*n23*n23 - n*n13*n13*n23 + n*n*n13*n23 - safety_factor*n*n13 - safety_factor*n*n23 + n*n13*n23 + safety_factor*n*n13*n23)/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1))};

	double SF, l;
	int s, conj_point, j;
	for (int i = 1; i <= n; i++) {
		interpolate(a_start_, a_goal_, (double)(i-1)/(double)(n-1)); // Compute point on line to a_interpolated
		conj_point = conjugate_point(a_interpolated); // Solve ode for a_interpolated

		// Compute scale factor 'l' of a_interpolated
		SF = cs[0]*i*i*i + cs[1]*i*i + cs[2]*i + cs[3];
		s = round_to_digits(SF*conj_point, 3); // was floor
		A_Path[i-1].s = s;
		l = (double)s / (double)get_Points_on_Rod();

		// Scale a
		for (j = 0; j < 3; j++) {
			a_scaled[j] = a_interpolated[j]*l;
			a_scaled[j+3] = a_interpolated[j+3]*l*l;
		}

		rod_solve(a_scaled);
		A_Path[i-1].a = a_scaled;
		A_Path[i-1].T_end_of_rod = getT(get_Points_on_Rod()-1);
		A_Path[i-1].P = getPMatrix();
	}
	a_steps = n;
}

//bool flag = false;
// Computes a path in a-Slice based on the edge of stability
// This function saves runtime over FindStable_A_Path() by computing every two points and average to find the middle point.
void Slice::FindStable_A_Path_w_Avg() {

	int m = max( (int)round( normDistance(a_start_,a_goal_) / (2*resolution) ) , 6 ); // if n<6, cs goes to Inf and an error occur
	int n = 2*m-1;
	cout << "Computing A_path with n = " << n << " nodes..." << endl;
	init_A_Path(n);
	dap1 = 1.0/n;

	// Safety curve
	int n13 = (int)floor(n/3);
	int n23 = (int)floor(2*n/3);
	State cs = {((safety_factor - 1)*(n - n13 - n23 + 1))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			-((safety_factor - 1)*(n*n + n - n13*n13 - n13*n23 - n23*n23 + 1))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			((safety_factor - 1)*(n*n*n13 + n*n*n23 - n*n13*n13 - n*n13*n23 + n*n13 - n*n23*n23 + n*n23 - n13*n13 - n13*n23 + n13 - n23*n23 + n23))/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1)),
			(n13*n23 + safety_factor*n*n - n13*n23*n23 - n13*n13*n23 + n13*n13*n23*n23 + safety_factor*n*n13*n13 - safety_factor*n*n*n13 + safety_factor*n*n23*n23 - safety_factor*n*n*n23 - n*n13*n23*n23 - n*n13*n13*n23 + n*n*n13*n23 - safety_factor*n*n13 - safety_factor*n*n23 + n*n13*n23 + safety_factor*n*n13*n23)/((n - n13)*(n - n23)*(n13 - 1)*(n23 - 1))};

	double SF, l;
	int s, conj_point, j, k;
	for (int i = 1; i <= n; i+=2) {
		interpolate(a_start_, a_goal_, (double)(i-1)/(double)(n-1)); // Compute point on line to a_interpolated
		conj_point = conjugate_point(a_interpolated); // Solve ode for a_interpolated

		// Compute scale factor 'l' of a_interpolated
		SF = cs[0]*i*i*i + cs[1]*i*i + cs[2]*i + cs[3];
		s = round_to_digits(SF*conj_point, 3); // was floor
		A_Path[i-1].s = s;
		l = (double)s / (double)get_Points_on_Rod();

		// Store matrix for end of scaled rod (at s position on original rod)
		A_Path[i-1].T_end_of_rod = getT(s-1);
		// Store all feasible matrices
		for (j = 0; j < s; j++)
			A_Path[i-1].T[j] = getT(j);

		// Store points on rod
		A_Path[i-1].P = getPMatrix();
		// Cut and scale the rod according to 's'
		for (j = s; j < get_Points_on_Rod(); j++)
			for (k = 0; k < 3; k++)
				A_Path[i-1].P[j][k] = 0;
		//(*A_Path[i-1].P).erase((*A_Path[i-1].P).begin()+s-1, (*A_Path[i-1].P).end());

		// Apply scale factor and store in A_path, and scale T's xyz position
		for (j = 0; j < 3; j++) {
			A_Path[i-1].a[j] = a_interpolated[j]*l;
			A_Path[i-1].a[j+3] = a_interpolated[j+3]*l*l;
			A_Path[i-1].T_end_of_rod[j][3] /= l;
			for (k = 0; k < A_Path[i-1].P.size(); k++)
				A_Path[i-1].P[k][j] /= l;
			for (k = 0; k < A_Path[i-1].T.size(); k++)
				A_Path[i-1].T[k][j][3] /= l;
		}

		// Compute point A_path[i-2] between A_path[i-1] and A_path[i-3] by averaging - reduce runtime
		if (i>1) {
			for (j=0; j<6; j++)
				A_Path[i-2].a[j] = ( A_Path[i-1].a[j] + A_Path[i-3].a[j] ) /2;

			for (j = 0; j < 4; j++)
				for (k = 0; k < 4; k++)
					A_Path[i-2].T_end_of_rod[j][k] = ( A_Path[i-1].T_end_of_rod[j][k] + A_Path[i-3].T_end_of_rod[j][k] ) / 2;

			A_Path[i-2].s = min(A_Path[i-1].s, A_Path[i-3].s);

			for (int t = 0; t < A_Path[i-2].s; t++) {
				for (j = 0; j < 4; j++)
					for (k = 0; k < 4; k++)
						A_Path[i-2].T[t][j][k] = ( A_Path[i-1].T[t][j][k] + A_Path[i-3].T[t][j][k] ) / 2;
			}

			for (j = 0; j < A_Path[i-2].s; j++)
				for (k = 0; k < 3; k++)
					A_Path[i-2].P[j][k] = ( A_Path[i-1].P[j][k] + A_Path[i-3].P[j][k] ) / 2;


		}
	}
	a_steps = n;
}

double Slice::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

void Slice::interpolate(State a1, State a2, double t) {

	for (int i = 0; i < 6; i++) {
		a_interpolated[i] = a1[i]*(1-t) + a2[i]*t;
	}
}



