#include "Rod_ODE_class.h"
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

vector<double> c_global;

// Debugging tool
template <class T>
void o(T a) {
	cout << a << endl;
}


// Constructor
rod_ode::rod_ode() {
	setC(1);
	setL(1);
	setDiscretizationSize(500);

	odes_counter = 0;
	valid_odes_counter = 0;
	odes_time = 0;
}

// ---Setters---

void rod_ode::setDiscretizationSize(int N) {
	points_on_rod = N;

	// Initialize solution variables
	//t_length.clear;
	//P.clear;
	//T.clear;

	t_length.resize(points_on_rod);

	initMatrix(P, points_on_rod, 3);

	initMatrixList(T, points_on_rod, 4, 4);

	initMatrixList(J, points_on_rod, 6, 6);
}

void rod_ode::setC(double stiffness) {
	for (int i = 0; i < 3; i++)
		c.push_back(stiffness); // Should be modified to assign different c components
}

void rod_ode::setL(double length) {
	L = length;
}

// ---Getters---

State rod_ode::getP(int index) {

	State p(P[index]);
	return p;
}

Matrix rod_ode::getPMatrix() {

	return P;
}

Matrix rod_ode::getT(int index) {

	Matrix TT(T[index]);
	return TT;
}

Matrix rod_ode::getJ(int index) {

	Matrix JJ(J[index]);
	return JJ;
}

double rod_ode::get_t_length(int index) {
	return t_length[index];
}

int rod_ode::get_Points_on_Rod() {
	return points_on_rod;
}

State rod_ode::get_conjugate_points_path() {
	return path_conjugate_points;
}

Matrix rod_ode::getJ_from_Jlist(int conf, int index) {

	Matrix JJ(Jlist[conf][index]);
	return JJ;
}

Matrix rod_ode::getT_from_Tlist(int conf, int index) {

	Matrix TT(Tlist[conf][index]);
	return TT;
}

// ---------------

void rod_ode::LogSolutionData(State x, const double t, int counter) {
	int i, j, k;

	// P
	j = 0;
	for (size_t i = 3; i <= 11; i += 4) {
		P[counter][j] = x[i]*1e3;
		j++;
	}

	// T - Transformation matrices along the rod
	k = 0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 4; j++) {
			T[counter][i][j] = x[k];
			k++;
		}
	for (i = 0; i < 3; i++)
		T[counter][i][3] = T[counter][i][3]*1e3;
	T[counter][3][1] = T[counter][3][1] = T[counter][3][2] = 0;
	T[counter][3][3] = 1;

	// t_length
	t_length[counter] = t;

	// If feasibility check is conducted, extract also J
	if (x.size() > 18) {
		// J - Jacobian matrices along the rod
		k = 54;
		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++) {
				J[counter][i][j] = x[k];
				k++;
			}
	}
}

// Logs the data during the ODE solve for the discrete geometry version
void rod_ode::LogSolutionDataG(State x, const double t, int counter) {
	int i, j, k;

	// Mu
	// If we require to solve q using the expm method, then we need to log \mu along the rod
	/*for (size_t i = 0; i < 6; i++)
		Mu[counter][i] = x[i];*/

	// If feasibility check is conducted, extract also J
	if (x.size() > 7) {
		// J - Jacobian matrices along the rod
		k = 42;
		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++) {
				J[counter][i][j] = x[k];
				k++;
			}
	}
}

void rod_ode::LogSolutionDataPath(State x, const double t, int counter) {
	// Plist[rod configuration][position along the rod][x-y-z]
	// Tlist[rod configuration][position along the rod][row][col]

	int l, i, j, k, h;
	int n = x.size()/90;

	for (l = 0; l < n; l++) {
		j = l * 90;

		// P list
		k = 0;
		for (i = j+3; i <= j+11; i += 4) {
			Plist[l][counter][k] = x[i]*1e3;
			k++;
		}

		// T - Transformation matrices along the rod
		k = j;
		for (i = 0; i < 3; i++)
			for (h = 0; h < 4; h++) {
				Tlist[l][counter][i][h] = x[k];
				k++;
			}
		for (i = 0; i < 3; i++)
			Tlist[l][counter][i][3] = Tlist[l][counter][i][3]*1e3;
		Tlist[l][counter][3][1] = Tlist[l][counter][3][1] = Tlist[l][counter][3][2] = 0;
		Tlist[l][counter][3][3] = 1;

		// Jlist
		// If feasibility check is conducted, extract also J
		// J - Jacobian matrices along the rod
		k = j+54;
		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++) {
				Jlist[l][counter][i][j] = x[k];
				k++;
			}
	}

	// t_length
	t_length[counter] = t;
}

State dxdt_func(State x, State u, State mu, State M, State J) {
	// ODE integration
	State dxdt = { x[1] * u[2] - x[2] * u[1], x[2] * u[0] - x[0] * u[2], x[0] * u[1] - x[1] * u[0], x[0], x[5] * u[2] - x[6] * u[1], x[6] * u[0] - x[4] * u[2], x[4] * u[1] - x[5] * u[0], x[4], x[9] * u[2] - x[10] * u[1], x[10] * u[0] - x[8] * u[2], x[8] * u[1] - x[9] * u[0], x[8],
			u[2] * mu[1] - u[1] * mu[2], mu[5] + u[0] * mu[2] - u[2] * mu[0] , -mu[4] + u[1] * mu[0] - u[0] * mu[1] , u[2] * mu[4] - u[1] * mu[5] , u[0] * mu[5] - u[2] * mu[3] , u[1] * mu[3] - u[0] * mu[4], // dmu
			//dM starts here
			-(1 / c_global[1] - 1 / c_global[2])*M[6] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[12] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[7] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[13] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[8] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[14] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[9] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[15] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[10] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[16] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[11] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[17] * mu[1],
			M[30] + (1 / c_global[0] - 1 / c_global[2])*M[0] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[12] * mu[0],
			M[31] + (1 / c_global[0] - 1 / c_global[2])*M[1] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[13] * mu[0],
			M[32] + (1 / c_global[0] - 1 / c_global[2])*M[2] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[14] * mu[0],
			M[33] + (1 / c_global[0] - 1 / c_global[2])*M[3] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[15] * mu[0],
			M[34] + (1 / c_global[0] - 1 / c_global[2])*M[4] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[16] * mu[0],
			M[35] + (1 / c_global[0] - 1 / c_global[2])*M[5] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[17] * mu[0],
			-M[24] - (1 / c_global[0] - 1 / c_global[1])*M[0] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[6] * mu[0],
			-M[25] - (1 / c_global[0] - 1 / c_global[1])*M[1] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[7] * mu[0],
			-M[26] - (1 / c_global[0] - 1 / c_global[1])*M[2] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[8] * mu[0],
			-M[27] - (1 / c_global[0] - 1 / c_global[1])*M[3] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[9] * mu[0],
			-M[28] - (1 / c_global[0] - 1 / c_global[1])*M[4] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[10] * mu[0],
			-M[29] - (1 / c_global[0] - 1 / c_global[1])*M[5] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[11] * mu[0],
			M[24] * u[2] - M[30] * u[1] - (M[6] * mu[5]) / c_global[1] + (M[12] * mu[4]) / c_global[2],
			M[25] * u[2] - M[31] * u[1] - (M[7] * mu[5]) / c_global[1] + (M[13] * mu[4]) / c_global[2],
			M[26] * u[2] - M[32] * u[1] - (M[8] * mu[5]) / c_global[1] + (M[14] * mu[4]) / c_global[2],
			M[27] * u[2] - M[33] * u[1] - (M[9] * mu[5]) / c_global[1] + (M[15] * mu[4]) / c_global[2],
			M[28] * u[2] - M[34] * u[1] - (M[10] * mu[5]) / c_global[1] + (M[16] * mu[4]) / c_global[2],
			M[29] * u[2] - M[35] * u[1] - (M[11] * mu[5]) / c_global[1] + (M[17] * mu[4]) / c_global[2],
			M[30] * u[0] - M[18] * u[2] + (M[0] * mu[5]) / c_global[0] - (M[12] * mu[3]) / c_global[2],
			M[31] * u[0] - M[19] * u[2] + (M[1] * mu[5]) / c_global[0] - (M[13] * mu[3]) / c_global[2],
			M[32] * u[0] - M[20] * u[2] + (M[2] * mu[5]) / c_global[0] - (M[14] * mu[3]) / c_global[2],
			M[33] * u[0] - M[21] * u[2] + (M[3] * mu[5]) / c_global[0] - (M[15] * mu[3]) / c_global[2],
			M[34] * u[0] - M[22] * u[2] + (M[4] * mu[5]) / c_global[0] - (M[16] * mu[3]) / c_global[2],
			M[35] * u[0] - M[23] * u[2] + (M[5] * mu[5]) / c_global[0] - (M[17] * mu[3]) / c_global[2],
			M[18] * u[1] - M[24] * u[0] - (M[0] * mu[4]) / c_global[0] + (M[6] * mu[3]) / c_global[1],
			M[19] * u[1] - M[25] * u[0] - (M[1] * mu[4]) / c_global[0] + (M[7] * mu[3]) / c_global[1],
			M[20] * u[1] - M[26] * u[0] - (M[2] * mu[4]) / c_global[0] + (M[8] * mu[3]) / c_global[1],
			M[21] * u[1] - M[27] * u[0] - (M[3] * mu[4]) / c_global[0] + (M[9] * mu[3]) / c_global[1],
			M[22] * u[1] - M[28] * u[0] - (M[4] * mu[4]) / c_global[0] + (M[10] * mu[3]) / c_global[1],
			M[23] * u[1] - M[29] * u[0] - (M[5] * mu[4]) / c_global[0] + (M[11] * mu[3]) / c_global[1],
			// dJ starts here
			M[0] / c_global[0] + J[6] * u[2] - J[12] * u[1],
			M[1] / c_global[0] + J[7] * u[2] - J[13] * u[1],
			M[2] / c_global[0] + J[8] * u[2] - J[14] * u[1],
			M[3] / c_global[0] + J[9] * u[2] - J[15] * u[1],
			M[4] / c_global[0] + J[10] * u[2] - J[16] * u[1],
			M[5] / c_global[0] + J[11] * u[2] - J[17] * u[1],
			M[6] / c_global[1] - J[0] * u[2] + J[12] * u[0],
			M[7] / c_global[1] - J[1] * u[2] + J[13] * u[0],
			M[8] / c_global[1] - J[2] * u[2] + J[14] * u[0],
			M[9] / c_global[1] - J[3] * u[2] + J[15] * u[0],
			M[10] / c_global[1] - J[4] * u[2] + J[16] * u[0],
			M[11] / c_global[1] - J[5] * u[2] + J[17] * u[0],
			M[12] / c_global[2] + J[0] * u[1] - J[6] * u[0],
			M[13] / c_global[2] + J[1] * u[1] - J[7] * u[0],
			M[14] / c_global[2] + J[2] * u[1] - J[8] * u[0],
			M[15] / c_global[2] + J[3] * u[1] - J[9] * u[0],
			M[16] / c_global[2] + J[4] * u[1] - J[10] * u[0],
			M[17] / c_global[2] + J[5] * u[1] - J[11] * u[0],
			J[24] * u[2] - J[30] * u[1],
			J[25] * u[2] - J[31] * u[1],
			J[26] * u[2] - J[32] * u[1],
			J[27] * u[2] - J[33] * u[1],
			J[28] * u[2] - J[34] * u[1],
			J[29] * u[2] - J[35] * u[1],
			J[12] - J[18] * u[2] + J[30] * u[0],
			J[13] - J[19] * u[2] + J[31] * u[0],
			J[14] - J[20] * u[2] + J[32] * u[0],
			J[15] - J[21] * u[2] + J[33] * u[0],
			J[16] - J[22] * u[2] + J[34] * u[0],
			J[17] - J[23] * u[2] + J[35] * u[0],
			J[18] * u[1] - J[6] - J[24] * u[0],
			J[19] * u[1] - J[7] - J[25] * u[0],
			J[20] * u[1] - J[8] - J[26] * u[0],
			J[21] * u[1] - J[9] - J[27] * u[0],
			J[22] * u[1] - J[10] - J[28] * u[0],
			J[23] * u[1] - J[11] - J[29] * u[0]	};

	return dxdt;
}

// ODE expressions, includes dmu, dM, dJ
State dxdt_funcG(State u, State mu, State M, State J) {
	// ODE integration
	State dxdt = { // dmu
			u[2] * mu[1] - u[1] * mu[2],
			mu[5] + u[0] * mu[2] - u[2] * mu[0],
			-mu[4] + u[1] * mu[0] - u[0] * mu[1],
			u[2] * mu[4] - u[1] * mu[5],
			u[0] * mu[5] - u[2] * mu[3],
			u[1] * mu[3] - u[0] * mu[4],
			//dM
			-(1 / c_global[1] - 1 / c_global[2])*M[6] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[12] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[7] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[13] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[8] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[14] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[9] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[15] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[10] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[16] * mu[1],
			-(1 / c_global[1] - 1 / c_global[2])*M[11] * mu[2] - (1 / c_global[1] - 1 / c_global[2])*M[17] * mu[1],
			M[30] + (1 / c_global[0] - 1 / c_global[2])*M[0] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[12] * mu[0],
			M[31] + (1 / c_global[0] - 1 / c_global[2])*M[1] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[13] * mu[0],
			M[32] + (1 / c_global[0] - 1 / c_global[2])*M[2] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[14] * mu[0],
			M[33] + (1 / c_global[0] - 1 / c_global[2])*M[3] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[15] * mu[0],
			M[34] + (1 / c_global[0] - 1 / c_global[2])*M[4] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[16] * mu[0],
			M[35] + (1 / c_global[0] - 1 / c_global[2])*M[5] * mu[2] + (1 / c_global[0] - 1 / c_global[2])*M[17] * mu[0],
			-M[24] - (1 / c_global[0] - 1 / c_global[1])*M[0] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[6] * mu[0],
			-M[25] - (1 / c_global[0] - 1 / c_global[1])*M[1] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[7] * mu[0],
			-M[26] - (1 / c_global[0] - 1 / c_global[1])*M[2] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[8] * mu[0],
			-M[27] - (1 / c_global[0] - 1 / c_global[1])*M[3] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[9] * mu[0],
			-M[28] - (1 / c_global[0] - 1 / c_global[1])*M[4] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[10] * mu[0],
			-M[29] - (1 / c_global[0] - 1 / c_global[1])*M[5] * mu[1] - (1 / c_global[0] - 1 / c_global[1])*M[11] * mu[0],
			M[24] * u[2] - M[30] * u[1] - (M[6] * mu[5]) / c_global[1] + (M[12] * mu[4]) / c_global[2],
			M[25] * u[2] - M[31] * u[1] - (M[7] * mu[5]) / c_global[1] + (M[13] * mu[4]) / c_global[2],
			M[26] * u[2] - M[32] * u[1] - (M[8] * mu[5]) / c_global[1] + (M[14] * mu[4]) / c_global[2],
			M[27] * u[2] - M[33] * u[1] - (M[9] * mu[5]) / c_global[1] + (M[15] * mu[4]) / c_global[2],
			M[28] * u[2] - M[34] * u[1] - (M[10] * mu[5]) / c_global[1] + (M[16] * mu[4]) / c_global[2],
			M[29] * u[2] - M[35] * u[1] - (M[11] * mu[5]) / c_global[1] + (M[17] * mu[4]) / c_global[2],
			M[30] * u[0] - M[18] * u[2] + (M[0] * mu[5]) / c_global[0] - (M[12] * mu[3]) / c_global[2],
			M[31] * u[0] - M[19] * u[2] + (M[1] * mu[5]) / c_global[0] - (M[13] * mu[3]) / c_global[2],
			M[32] * u[0] - M[20] * u[2] + (M[2] * mu[5]) / c_global[0] - (M[14] * mu[3]) / c_global[2],
			M[33] * u[0] - M[21] * u[2] + (M[3] * mu[5]) / c_global[0] - (M[15] * mu[3]) / c_global[2],
			M[34] * u[0] - M[22] * u[2] + (M[4] * mu[5]) / c_global[0] - (M[16] * mu[3]) / c_global[2],
			M[35] * u[0] - M[23] * u[2] + (M[5] * mu[5]) / c_global[0] - (M[17] * mu[3]) / c_global[2],
			M[18] * u[1] - M[24] * u[0] - (M[0] * mu[4]) / c_global[0] + (M[6] * mu[3]) / c_global[1],
			M[19] * u[1] - M[25] * u[0] - (M[1] * mu[4]) / c_global[0] + (M[7] * mu[3]) / c_global[1],
			M[20] * u[1] - M[26] * u[0] - (M[2] * mu[4]) / c_global[0] + (M[8] * mu[3]) / c_global[1],
			M[21] * u[1] - M[27] * u[0] - (M[3] * mu[4]) / c_global[0] + (M[9] * mu[3]) / c_global[1],
			M[22] * u[1] - M[28] * u[0] - (M[4] * mu[4]) / c_global[0] + (M[10] * mu[3]) / c_global[1],
			M[23] * u[1] - M[29] * u[0] - (M[5] * mu[4]) / c_global[0] + (M[11] * mu[3]) / c_global[1],
			// dJ
			M[0] / c_global[0] + J[6] * u[2] - J[12] * u[1],
			M[1] / c_global[0] + J[7] * u[2] - J[13] * u[1],
			M[2] / c_global[0] + J[8] * u[2] - J[14] * u[1],
			M[3] / c_global[0] + J[9] * u[2] - J[15] * u[1],
			M[4] / c_global[0] + J[10] * u[2] - J[16] * u[1],
			M[5] / c_global[0] + J[11] * u[2] - J[17] * u[1],
			M[6] / c_global[1] - J[0] * u[2] + J[12] * u[0],
			M[7] / c_global[1] - J[1] * u[2] + J[13] * u[0],
			M[8] / c_global[1] - J[2] * u[2] + J[14] * u[0],
			M[9] / c_global[1] - J[3] * u[2] + J[15] * u[0],
			M[10] / c_global[1] - J[4] * u[2] + J[16] * u[0],
			M[11] / c_global[1] - J[5] * u[2] + J[17] * u[0],
			M[12] / c_global[2] + J[0] * u[1] - J[6] * u[0],
			M[13] / c_global[2] + J[1] * u[1] - J[7] * u[0],
			M[14] / c_global[2] + J[2] * u[1] - J[8] * u[0],
			M[15] / c_global[2] + J[3] * u[1] - J[9] * u[0],
			M[16] / c_global[2] + J[4] * u[1] - J[10] * u[0],
			M[17] / c_global[2] + J[5] * u[1] - J[11] * u[0],
			J[24] * u[2] - J[30] * u[1],
			J[25] * u[2] - J[31] * u[1],
			J[26] * u[2] - J[32] * u[1],
			J[27] * u[2] - J[33] * u[1],
			J[28] * u[2] - J[34] * u[1],
			J[29] * u[2] - J[35] * u[1],
			J[12] - J[18] * u[2] + J[30] * u[0],
			J[13] - J[19] * u[2] + J[31] * u[0],
			J[14] - J[20] * u[2] + J[32] * u[0],
			J[15] - J[21] * u[2] + J[33] * u[0],
			J[16] - J[22] * u[2] + J[34] * u[0],
			J[17] - J[23] * u[2] + J[35] * u[0],
			J[18] * u[1] - J[6] - J[24] * u[0],
			J[19] * u[1] - J[7] - J[25] * u[0],
			J[20] * u[1] - J[8] - J[26] * u[0],
			J[21] * u[1] - J[9] - J[27] * u[0],
			J[22] * u[1] - J[10] - J[28] * u[0],
			J[23] * u[1] - J[11] - J[29] * u[0]	};

	return dxdt;
}

// ODE function of the rod for the solve function to set in odeint
void ode_function_solve(State x, State &dxdt, double t) {
	int i;

	// Extract mu
	State mu(6);
	for (i = 0; i < 6; i++)
		mu[i] = x[i + 12];

	// Set u
	State u(3);
	for (i = 0; i < 3; i++)
		u[i] = mu[i] / c_global[i];

	// ODE integration
	dxdt = { x[1] * u[2] - x[2] * u[1], x[2] * u[0] - x[0] * u[2], x[0] * u[1] - x[1] * u[0], x[0], x[5] * u[2] - x[6] * u[1], x[6] * u[0] - x[4] * u[2], x[4] * u[1] - x[5] * u[0], x[4], x[9] * u[2] - x[10] * u[1], x[10] * u[0] - x[8] * u[2], x[8] * u[1] - x[9] * u[0], x[8],
			u[2] * mu[1] - u[1] * mu[2], mu[5] + u[0] * mu[2] - u[2] * mu[0] , -mu[4] + u[1] * mu[0] - u[0] * mu[1] , u[2] * mu[4] - u[1] * mu[5] , u[0] * mu[5] - u[2] * mu[3] , u[1] * mu[3] - u[0] * mu[4] };
}

// ODE function of the rod for the solve function to set in odeint
void ode_function_A_path_solve(State x, State &dxdt, double t) {
	int i, j, k, l;
	int n = x.size()/18;

	// Extract mu
	State mu(6);
	for (i = 0; i < 6; i++)
		mu[i] = x[i + 12];

	// Set u
	State u(3);
	for (i = 0; i < 3; i++)
		u[i] = mu[i] / c_global[i];

	// ODE integration
	dxdt = { x[1] * u[2] - x[2] * u[1], x[2] * u[0] - x[0] * u[2], x[0] * u[1] - x[1] * u[0], x[0], x[5] * u[2] - x[6] * u[1], x[6] * u[0] - x[4] * u[2], x[4] * u[1] - x[5] * u[0], x[4], x[9] * u[2] - x[10] * u[1], x[10] * u[0] - x[8] * u[2], x[8] * u[1] - x[9] * u[0], x[8],
			u[2] * mu[1] - u[1] * mu[2], mu[5] + u[0] * mu[2] - u[2] * mu[0] , -mu[4] + u[1] * mu[0] - u[0] * mu[1] , u[2] * mu[4] - u[1] * mu[5] , u[0] * mu[5] - u[2] * mu[3] , u[1] * mu[3] - u[0] * mu[4] };

	State dxdtT;
	for (k = 1; k < n; k++) {
		j = k * 18;

		// Extract mu
		l = 0;
		for (i = j+12; i < j+18; i++, l++)
			mu[l] = x[i];

		// Set u
		for (i = 0; i < 3; i++)
			u[i] = mu[i] / c_global[i];

		dxdtT = { x[j+1] * u[2] - x[j+2] * u[1], x[j+2] * u[0] - x[j] * u[2], x[j] * u[1] - x[j+1] * u[0], x[j], x[j+5] * u[2] - x[j+6] * u[1], x[j+6] * u[0] - x[j+4] * u[2], x[j+4] * u[1] - x[j+5] * u[0], x[j+4], x[j+9] * u[2] - x[j+10] * u[1], x[j+10] * u[0] - x[j+8] * u[2], x[j+8] * u[1] - x[j+9] * u[0], x[j+8],
				u[2] * mu[1] - u[1] * mu[2], mu[5] + u[0] * mu[2] - u[2] * mu[0] , -mu[4] + u[1] * mu[0] - u[0] * mu[1] , u[2] * mu[4] - u[1] * mu[5] , u[0] * mu[5] - u[2] * mu[3] , u[1] * mu[3] - u[0] * mu[4] };
		dxdt.insert(dxdt.end(), dxdtT.begin(), dxdtT.end());
	}
}

void print_Vector(State a) {
	std::cout << "Vector state: ";
	for (unsigned i = 0; i < a.size(); i++)
		std::cout << a[i] << " ";
	std::cout << std::endl;
}


// ODE function of the rod for the solve function to set in odeint
void ode_function_feasibility(State x, State &dxdt, double t) {
	int i;

	// Extract q
	State q(12);
	for (i = 0; i < 12; i++)
		q[i] = x[i];

	// Extract mu
	State mu(6);
	for (i = 0; i < 6; i++)
		mu[i] = x[i + 12];

	// Set u
	State u(3);
	for (i = 0; i < 3; i++)
		u[i] = mu[i] / c_global[i];

	// Set M
	State M(36);
	for (i = 0; i < 36; i++)
		M[i] = x[i+18];

	// Set J
	State J(36);
	for (i = 0; i < 36; i++)
		J[i] = x[i+54];

	// ODE integration
	dxdt = dxdt_func(q, u, mu, M, J);
}

// ODE function of the rod for the solve function to set in odeint - for the discrete geometry approach
void ode_function_feasibilityG(State x, State &dxdt, double t) {
	int i;

	// Extract mu
	State mu(6);
	for (i = 0; i < 6; i++)
		mu[i] = x[i];

	// Set u
	State u(3);
	for (i = 0; i < 3; i++)
		u[i] = mu[i] / c_global[i];

	// Set M
	State M(36);
	for (i = 0; i < 36; i++)
		M[i] = x[i+6];

	// Set J
	State J(36);
	for (i = 0; i < 36; i++)
		J[i] = x[i+42];

	// ODE integration
	dxdt = dxdt_funcG(u, mu, M, J);
}

// ODE function of the rod for the solve function to set in odeint
void ode_function_feasibility_path(State x, State &dxdt, double t) {
	int i, j, k, l;
	int n = x.size()/90;

	// Extract q
	State q(12);
	for (i = 0; i < 12; i++)
		q[i] = x[i];

	// Extract mu
	State mu(6);
	for (i = 0; i < 6; i++)
		mu[i] = x[i + 12];

	// Set u
	State u(3);
	for (i = 0; i < 3; i++)
		u[i] = mu[i] / c_global[i];

	// Set M
	State M(36);
	for (i = 0; i < 36; i++)
		M[i] = x[i+18];

	// Set J
	State J(36);
	for (i = 0; i < 36; i++)
		J[i] = x[i+54];

	// ODE integration
	dxdt = dxdt_func(q, u, mu, M, J);

	State dxdtT;
	for (k = 1; k < n; k++) {
		j = k * 90;

		// Extract q
		l = 0;
		for (i = j; i < j+12; i++, l++)
			q[l] = x[i];

		// Extract mu
		l = 0;
		for (i = j+12; i < j+18; i++, l++)
			mu[l] = x[i];

		// Set u
		for (i = 0; i < 3; i++)
			u[i] = mu[i] / c_global[i];

		// Set M
		l = 0;
		for (i = j+18; i < j+54; i++, l++)
			M[l] = x[i];

		// Set J
		l = 0;
		for (i = j+54; i < j+90; i++, l++)
			J[l] = x[i];

		dxdtT = dxdt_func(q, u, mu, M, J);
		dxdt.insert(dxdt.end(), dxdtT.begin(), dxdtT.end());

	}
}

// Solve rod and return the rods coordinates and transformation matrices
void rod_ode::rod_solve(State a) {
	// a - configuration of the rod in A-space

	State x = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, a[0], a[1], a[2], a[3], a[4], a[5]};
	const double dt = L / (points_on_rod-1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	stepper_type stepper;
	//clock_t begin = clock();
	for (int i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_solve, x, t, dt);
		LogSolutionData(x, t, i);
	}
	//clock_t end = clock();
	//solution_time = double(end - begin) / CLOCKS_PER_SEC;
}

// Solve rod and return the rods coordinates and transformation matrices
void rod_ode::rod_path_solve(Matrix A) {
	// a - configuration of the rod in A-space
	int i,j,k;
	State q0 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
	State x;
	for (i = 0; i < A.size(); i++) {
		for (j = 0; j < q0.size(); j++)
			x.push_back(q0[j]);
		for (j = 0; j < 6; j++)
			x.push_back(A[i][j]);
	}

	const double dt = L / (points_on_rod-1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	stepper_type stepper;
	//clock_t begin = clock();
	for (i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_A_path_solve, x, t, dt);
		//LogSolutionData(x, t, i);
	}
	//clock_t end = clock();
	//solution_time = double(end - begin) / CLOCKS_PER_SEC;
}

// Is rod configuration feasible
// • Discrete geometry reduces 10%-15%.
// • Scanning stability from the end of the rod saves 25%.
bool rod_ode::isRodFeasible(State a) {
	// a - configuration of the rod in A-space
	// Returns 'True' if NO colllision.

	odes_counter++;
	clock_t begin = clock();

	State x = { a[0], a[1], a[2], a[3], a[4], a[5], 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	for (int i = 0; i < 36; i++)
		x.push_back(0);
	const double dt = L / (points_on_rod - 1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	// Solve extended ODE
	stepper_type stepper;
	for (int i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_feasibilityG, x, t, dt);
		LogSolutionDataG(x, t, i);
	}

	// Scan the determinant from the end of the rod and not from the beginning
	for (int i = points_on_rod-1; i >= 0; i--) {
		if (determinant(J[i]) <= 0) {
			clock_t end = clock();
			odes_time += double(end - begin) / CLOCKS_PER_SEC;
			return false;
		}
	}

	// Only if the rod is stable, then solve the configurations. Can also be done by the expm method, but same timing and less accuracy.
	rod_solve(a);

	// Check for rod collision
	if (Rod_Collision()) {
		clock_t end = clock();
		odes_time += double(end - begin) / CLOCKS_PER_SEC;
		valid_odes_counter++;
		return true;
	}
	else {
		clock_t end = clock();
		odes_time += double(end - begin) / CLOCKS_PER_SEC;
		return false;
	}
}

// Old version - Compute q,\mu,M,J in the same ode.
// Is rod configuration feasible
bool rod_ode::isRodFeasible_old(State a) {
	// a - configuration of the rod in A-space
	// Returns 'True' if NO colllision.

	//cout << "Rod check." << endl;

	odes_counter++;
	clock_t begin = clock();

	State x = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, a[0], a[1], a[2], a[3], a[4], a[5], 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	for (int i = 0; i < 36; i++)
		x.push_back(0);
	const double dt = L / (points_on_rod - 1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	// Solve extended ODE
	stepper_type stepper;
	for (int i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_feasibility, x, t, dt);
		LogSolutionData(x, t, i);
	}

	// Check for stability
	for (int i = 1; i < points_on_rod; i++) {
		if (determinant(J[i]) <= 0) {
			clock_t end = clock();
			odes_time += double(end - begin) / CLOCKS_PER_SEC;
			valid_odes_counter++;
			return false;
		}
	}

	// Check for rod collision
	if (Rod_Collision()) {
		clock_t end = clock();
		odes_time += double(end - begin) / CLOCKS_PER_SEC;
		return true;
	}
	else {
		clock_t end = clock();
		odes_time += double(end - begin) / CLOCKS_PER_SEC;
		return false;
	}
}

// Is rod configuration feasible
int rod_ode::conjugate_point(State a) {
	// a - configuration of the rod in A-space
	// Returns the first conjugate point

	odes_counter++;
	valid_odes_counter++;
	clock_t begin = clock();

	State x = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, a[0], a[1], a[2], a[3], a[4], a[5], 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	for (int i = 0; i < 36; i++)
		x.push_back(0); // J0
	const double dt = L / (points_on_rod - 1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	// Solve extended ODE
	stepper_type stepper;
	for (int i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_feasibility, x, t, dt);
		LogSolutionData(x, t, i);
	}

	// Compute the first conjugate point. Do it coarse and if found a point, fine the search
	// This saves time only while scanning tin the path connected slice
	int stabiliy_point = 0;
	while (stabiliy_point < points_on_rod && determinant(J[stabiliy_point]) > 0) {
		stabiliy_point+=50;
	}
	if (stabiliy_point < points_on_rod) { // do it again with one increment
		stabiliy_point = max(0,stabiliy_point - 51);
		while (stabiliy_point < points_on_rod && determinant(J[stabiliy_point]) > 0) {
			stabiliy_point++;
		}
	}
	else
		stabiliy_point = points_on_rod;

	// Compute the first collision point
	Rod_Collision();

	clock_t end = clock();
	odes_time += double(end - begin) / CLOCKS_PER_SEC;

	return min(collision_point, stabiliy_point);
}

// Is rod configuration feasible
int rod_ode::conjugate_point_old(State a) {
	// a - configuration of the rod in A-space
	// Returns the first conjugate point

	odes_counter++;
	valid_odes_counter++;
	clock_t begin = clock();

	State x = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, a[0], a[1], a[2], a[3], a[4], a[5], 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	for (int i = 0; i < 36; i++)
		x.push_back(0); // J0
	const double dt = L / (points_on_rod - 1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	// Solve extended ODE
	stepper_type stepper;
	for (int i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_feasibility, x, t, dt);
		LogSolutionData(x, t, i);
	}

	// Compute the first conjugate point. Do it coarse and if found a point, fine the search
	int stabiliy_point = 0;
	while (stabiliy_point < points_on_rod && determinant(J[stabiliy_point]) > 0) {
		stabiliy_point++;
	}

	// Compute the first collision point
	Rod_Collision();

	clock_t end = clock();
	odes_time += double(end - begin) / CLOCKS_PER_SEC;

	return min(collision_point, stabiliy_point);
}

// Is rod configuration feasible
void rod_ode::conjugate_point_path(Matrix A) {
	// a - configuration of the rod in A-space
	// Returns the first conjugate point

	clock_t begin = clock();
	int i,j,k;
	State q0 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
	State M0 = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	State x;
	for (i = 0; i < A.size(); i++) {
		for (j = 0; j < q0.size(); j++)
			x.push_back(q0[j]);
		for (j = 0; j < 6; j++)
			x.push_back(A[i][j]);
		for (j = 0; j < M0.size(); j++) // M0
			x.push_back(M0[j]);
		for (j = 0; j < 36; j++) // J0
			x.push_back(0);
	}
	clock_t end = clock();
	cout << "runtime1b: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	const double dt = L / (points_on_rod - 1);

	double t = 0.0;
	c_global = c; // Transfer stiffness c through the global variable

	// Solve extended ODE
	stepper_type stepper;
	initPathLogLists(A.size());
	begin = clock();
	for (int i = 0; i < points_on_rod; ++i, t += dt)
	{
		stepper.do_step(ode_function_feasibility_path, x, t, dt);
		LogSolutionDataPath(x, t, i);
	}
	end = clock();
	cout << "runtime2b: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	begin = clock();
	// Compute the first conjugate point. Do it coarse and if found a point, fine the search
	for (i = 0; i < x.size()/90; i++) {
		// Compute the first conjugate point
		int stabiliy_point = 0;
		while (stabiliy_point < points_on_rod && determinant(Jlist[i][stabiliy_point]) > 0) {
			stabiliy_point++;
		}

		// Compute the first collision point
		Rod_Collision();

		path_conjugate_points[i] = min(collision_point, stabiliy_point);
	}
	end = clock();
	cout << "runtime3b: " << double(end - begin) / CLOCKS_PER_SEC << endl;
}

// This function checks for self - collisions
bool rod_ode::Rod_Collision() {
	// Returns 'true' if no collision
	collision_point = points_on_rod;
	int i, j;

	// Find minimum distance between each point of the rod and its neighbors
	/*State r(points_on_rod-1), rr(points_on_rod);
	for (i = 1; i < points_on_rod; i++)
		r[i - 1] = (P[i][0] - P[i - 1][0])*(P[i][0] - P[i - 1][0]) + (P[i][1] - P[i - 1][1])*(P[i][1] - P[i - 1][1]) + (P[i][2] - P[i - 1][2])*(P[i][2] - P[i - 1][2]);
	r.push_back(r[points_on_rod - 2]);

	rr[0] = r[1];
	for (i = 2; i < points_on_rod; i++)
		rr[i-1] = fmin(r[i], r[i - 1]);
	rr[points_on_rod - 1] = r[points_on_rod - 1];*/

	// Find distances between all points on the rod
	State dis(points_on_rod, 1e9); // Initiate values in cells to be 100
	double temp;
	const int thres = 30;
	for (i=0; i < points_on_rod-thres; i++) {
		for (j = i + thres; j < points_on_rod; j++) {
			temp = (P[i][0] - P[j][0])*(P[i][0] - P[j][0]) + (P[i][1] - P[j][1])*(P[i][1] - P[j][1]) + (P[i][2] - P[j][2])*(P[i][2] - P[j][2]);
			if (temp < dis[i])
				dis[i] = temp;
		}
	}

	// Find number of collisions
	temp = 1e9;
	for (i = 0; i < points_on_rod; i++) {
		//if (dis[i] < temp)
			//temp = dis[i];
		if (dis[i] <= rod_collision_tol) {
			collision_point = i-1;
			return false;
		}
	}
	/*cout << sqrt(temp) << endl;
	i = 0; j = 415;
	for (int k = 0; k < 3; k++)
		cout << P[i][k] << " ";
	for (int k = 0; k < 3; k++)
		cout << P[j][k] << " ";

	cout << endl << sqrt((P[i][0] - P[j][0])*(P[i][0] - P[j][0]) + (P[i][1] - P[j][1])*(P[i][1] - P[j][1]) + (P[i][2] - P[j][2])*(P[i][2] - P[j][2])) << endl;*/
	return true;
}

// -------Misc----------------
// Compute the determinant of a 6x6 matrix with boost - LU decomposition
double rod_ode::determinant(Matrix mat) const {

	// Copy from std::vector matrix to boost matrix to use in the LU-decomposition
	bnu::matrix<double> m(6,6);
	for (unsigned i = 0; i < m.size1() ; ++i) {
		for (unsigned j = 0; j < m.size2() ; ++j) {
			m(i,j) = mat[i][j]; 
		}
	}

	bnu::permutation_matrix<std::size_t> pm(m.size1());
	double det = 1.0;
	if( bnu::lu_factorize(m,pm) ) {
		return 0.0;
	} 
	else {
		for(unsigned i = 0; i < m.size1(); i++) 
			det *= m(i,i); // multiply by elements on diagonal

		int pm_sign=1;
		size_t size = pm.size();
		for (std::size_t i = 0; i < size; ++i)
			if (i != pm(i))
				pm_sign *= -1.0; // swap_rows would swap a pair of rows here, so we change sign
		det = det * pm_sign;
	}
	return det;

}

// Compute the determinant of a 6x6 matrix
double rod_ode::det(int n, Matrix mat)
{
	int i, j, c;
	double d = 0;

	Matrix submat;
	initMatrix(submat, 6, 6);

	if (n == 2)
		return ((mat[0][0] * mat[1][1]) - (mat[1][0] * mat[0][1]));
	else {
		for (c = 0; c < n; c++)
		{
			int subi = 0; //submatrix's i value
			for (i = 1; i < n; i++)
			{
				int subj = 0;
				for (j = 0; j < n; j++)
				{
					if (j == c)
						continue;
					submat[subi][subj] = mat[i][j];
					subj++;
				}
				subi++;

			}
			d = d + (pow(-1, c) * mat[0][c] * det(n - 1, submat));
		}
	}

	return d;
}

void rod_ode::initMatrixList(MatrixList &M, int n, int m, int l) {
	int i, j;

	M.resize(n);
	for (i = 0; i < n; ++i) {
		M[i].resize(m);
		for (j = 0; j < m; ++j) 
			M[i][j].resize(l);
	}
}

void rod_ode::initMatrixListList(MatrixListList &M, int n, int m, int l, int h) {
	int i, j, k;

	M.resize(n);
	for (i = 0; i < n; ++i) {
		M[i].resize(m);
		for (j = 0; j < m; ++j) {
			M[i][j].resize(l);
			for (k = 0; k < l; ++k)
				M[i][j][k].resize(h);
		}
	}
}

void rod_ode::initPathLogLists(int n) {
	initMatrixList(Plist,n,points_on_rod,6);
	initMatrixListList(Tlist, n, points_on_rod, 4, 4);
	initMatrixListList(Jlist, n, points_on_rod, 6, 6);
	path_conjugate_points.resize(n);
}

void rod_ode::print_A_State(State a) const{
	std::cout << "Vector state: ";
	for (unsigned i = 0; i < a.size(); i++)
		std::cout << a[i] << " ";
	std::cout << std::endl;
}

int rod_ode::min(int x, int y)
{
	if (x <= y)
	{
		return x;
	}
	return y;
}



// ====================
/*
int main() {

	rod_ode rod;
	state a = {2.17, -4.2, 2, 1.9, 13.6, -2.4};
	Matrix temp;

	rod.setC(1);
	rod.setL(1);
	rod.setDiscretizationSize(500);
	rod.isRodFeasible(a);
	//rod.solve(a);

	temp = rod.getT(499);

	return 0;
}*/

