#pragma once
/* This code defines the kinematics (IK and FK) of two ABB IRB120 robotic arms */

#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include <math.h>
#include <boost/numeric/ublas/lu.hpp>

#define PI 3.1416
#define NUM_IK_SOLUTIONS 12

namespace bnu = boost::numeric::ublas;
using namespace std;

typedef vector<double> State;
typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

class two_robots
{
private:
	double b,l1, l2, l3a, l3b, l4, l5, l6, lee; // Links lengths
	double q1minmax, q2minmax, q3min, q3max, q4minmax, q5minmax, q6minmax; // Joint limits
	Vector V_pose_rob_1_o; // The vector [x,y,theta] that describes the position and orientation of robot 1 relative to origin (assume on the same x-y plane)
	Vector V_pose_rob_2_o; // The vector [x,y,theta] that describes the position and orientation of robot 2 relative to origin (assume on the same x-y plane)
	// FK parameters
	Matrix T_fk_solution_1; // FK solution of robot 1
	Vector p_fk_solution_1; // FK solution of robot 1
	Matrix T_fk_solution_2; // FK solution of robot 2
	Vector p_fk_solution_2; // FK solution of robot 2
	// IK parameters
	Vector q_IK_solution_1; // IK solution of robot 1
	Vector q_IK_solution_2; // IK solution of robot 2
	Matrix Q_IK_solutions_1; // All IK solutions of robot 1
	Matrix Q_IK_solutions_2; // All IK solutions of robot 2
	Vector valid_IK_solutions_indices_1; // Indices of the IK solutions to use in the IK_solve function for robot 1
	Vector valid_IK_solutions_indices_2; // Indices of the IK solutions to use in the IK_solve function for robot 2
	int IK_number;
	int countSolutions;

	// Temp variable for time efficiency
	Matrix T_fk_temp;
	Vector p_fk_temp;
	Vector V_pose;
	Matrix T1, T2, T_mult_temp;

public:
	// Constructor
	two_robots(Vector, Vector);

	// Forward kinematics
	void FKsolve_rob(Vector, int);
	Matrix get_FK_solution_T1();
	Vector get_FK_solution_p1();
	Matrix get_FK_solution_T2();
	Vector get_FK_solution_p2();

	// Inverse kinematics
	bool IKsolve_rob(Matrix, int, int);
	Vector get_IK_solution_q1();
	Vector get_IK_solution_q2();
	int get_countSolutions();
	bool calc_specific_IK_solution_R1(Matrix T, Vector q1, int IKsol);
	bool calc_specific_IK_solution_R2(Matrix T, Vector q2, int IKsol);

	// Is robots configurations feasible
	bool IsRobotsFeasible(Matrix, Vector);
	bool IsRobotsFeasible_R2(Matrix T, Vector q);
	int calc_all_IK_solutions_1(Matrix);
	int calc_all_IK_solutions_2(Matrix);

	// Getters
	int get_IK_number();
	Matrix get_T2();
	Vector get_all_IK_solutions_1(int);
	Vector get_all_IK_solutions_2(int);
	int get_valid_IK_solutions_indices_1(int);
	int get_valid_IK_solutions_indices_2(int);

	// Misc
	void initVector1(Vector &, int);
	void initMatrix1(Matrix &, int, int);
	double deg2rad(double);
	void printMatrixR(Matrix);
	void printVectorR(Vector);
	Matrix MatricesMult(Matrix, Matrix);
	bool InvertMatrix(Matrix M, Matrix &Minv); // Inverse of a 4x4 matrix
	double determ(Matrix mat);
	void clearMatrix(Matrix &);
	double normDistance(Vector, Vector);
	void relax_joint_limits();

	// Performance parameters
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}

};
