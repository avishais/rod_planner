/*
 * slice.h
 *
 *  Created on: Feb 3, 2017
 *      Author: avishai
 */

#ifndef SLICE_H_
#define SLICE_H_

#include <math.h>

#include "Rod_ODE_class.h"


class Slice : public rod_ode
{
protected:

	class A_PATH {
	public:
		A_PATH(){}
		A_PATH(int pointsOnRod){alloc(pointsOnRod);} // Create A_PATH cell and initiate
		void alloc(int pointsOnRod) {
			a.resize(6);

			T_end_of_rod.resize(4);
			for (int i=0; i<4; i++)
				T_end_of_rod[i].resize(4);

			T.resize(pointsOnRod);
			for (int j = 0; j < pointsOnRod; j++) {
				T[j].resize(4);
				for (int i=0; i<4; i++)
					T[j][i].resize(4);
			}

			P.resize(pointsOnRod);
			for (int i=0; i<pointsOnRod; i++)
				P[i].resize(3);
		}
		State a; // Current point a
		Matrix T_end_of_rod; // Rotation matrix at the end of the rod (where the beginning is at (0,0,0))
		MatrixList T;
		Matrix P; // Points forming the rod
		int s;
	};

	const double resolution = 0.3;
	State a_start_, a_goal_;

	vector< A_PATH > A_Path;
	int a_steps = 0; // Number of a-states in A_path
	State a_interpolated;
	const double safety_factor = 0.9;
	double dap1 = 1; // minimum step over state in a_path
	double dap2 = 1; // minimum step over state in the scale direction


public:
	// Constructor
	Slice();
	// Destructor
	~Slice() {};

	void init_A_Path(int n);

	void FindStable_A_Path();
	void FindStable_A_Path_Exact();
	void FindStable_A_Path_w_Avg();

	double normDistance(State, State);
	void interpolate(State, State, double);

	template <class T>
	void printMatrix(T M) {
		for (unsigned i = 0; i < M.size(); i++) {
			for (unsigned j = 0; j < M[i].size(); j++)
				cout << M[i][j] << " ";
			cout << endl;
		}
	}

	template <class T>
	void printVector(T p) {
		cout << "[";
		for (unsigned i = 0; i < p.size(); i++)
			cout << p[i] << " ";
		cout << "]" << endl;
	}
};


#endif /* SLICE_H_ */
