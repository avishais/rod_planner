/*
 * kdl_class.h
 *
 *  Created on: Feb 27, 2017
 *      Author: avishai
 */

#ifndef KDL_CLASS_H_
#define KDL_CLASS_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#define PI 3.1416

using namespace std;
using namespace KDL;

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class Baxter
{
private:
	double b, l1x, l1y, l1z, l2a, l2b, l3, l4a, l4b, l5, l6a, l6b, l7, lEE;
	State qmin, qmax; // Joint limits
	unsigned int nj; // Number of joints

	// FK parameters
	Matrix T_fk_solution_1; // FK solution of robot 1
	Matrix T_fk_solution_2; // FK solution of robot 2

	// IK parameters
	State q_IK_solution_1; // IK solution of robot 1
	State q_IK_solution_2; // IK solution of robot 2

	// Temp variable for time efficiency
	Matrix T_fk_temp;
	Matrix T_mult_temp;
	Matrix T_pose;

	bool grasp_pose; // false - rod is grasped such that it is continuous to the arm, true - rod is grasped perpendicular to the gripper plane

public:
	// Constructor
	Baxter();

	// KDL declarations
	KDL::Chain chain;
	//ChainFkSolverPos_recursive fksolver;
	KDL::JntArray jointpositions;
	KDL::Frame cartposFK; // Create the frame that will contain the FK results
	KDL::Frame cartposIK; // Create the frame that will contain the IK results

	// Forward kinematics
	void FKsolve_rob(State, int);
	Matrix get_FK_solution_T1();
	Matrix get_FK_solution_T2();

	// Inverse kinematics
	bool IKsolve_rob(Matrix, State, int);
	State get_IK_solution_q1();
	State get_IK_solution_q2();
	bool check_angle_limits(State, int);
	bool calc_specific_IK_solution_R1(Matrix T, State, State);
	bool calc_specific_IK_solution_R2(Matrix T, State, State);

	// Misc
	void initVector1(State &, int);
	void initMatrix1(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix1(Matrix);
	void printVector1(State);
	Matrix MatricesMult(Matrix, Matrix);
	void clearMatrix(Matrix &);
	State generate_random_arm_configuration(int);
	bool InvertMatrix(Matrix M, Matrix &Minv);

	// Performance parameters
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}

	int get_num_joint() {
		return nj;
	}

	bool get_grasp_pose() {
		return grasp_pose;
	}
};



#endif /* KDL_CLASS_H_ */
