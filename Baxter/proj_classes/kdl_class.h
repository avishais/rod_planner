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
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#define PI_ 3.1416

using namespace std;
using namespace KDL;

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class kdl
{
private:
	double b, l1x, l1y, l1z, l2a, l2b, l3, l4a, l4b, l5, l6a, l6b, l7, lEE;
	State qmin, qmax; // Joint limits

	// IK parameters
	State q_solution; // IK solution

	// Temp variable for time efficiency
	Matrix T_pose;

	double L; // Rod length

	bool grasp_pose; // false - rod is grasped such that it is continuous to the arm, true - rod is grasped perpendicular to the gripper plane

public:
	// Constructor
	kdl(double);

	/** KDL declarations */
	//ChainFkSolverPos_recursive fksolver;
	KDL::JntArray jointpositions;
	KDL::Frame cartposFK; // Create the frame that will contain the FK results
	KDL::Frame cartposIK; // Create the frame that will contain the FK results

	/** Newton-Raphson projection onto the constraint surface */
	bool GD(State, Matrix);
	State get_GD_result();

	/** Check the angles limits of the ABB - IK based on a constant trans. matrix */
	bool check_angle_limits(State);

	/**  Forward kinematics of the arm - only for validation */
	void FK(State, Matrix);
	Matrix get_FK_solution();
	Matrix T_fk, T_fk_temp;

	/** Misc */
	void initVector(State &, int);
	void initMatrix(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix(Matrix);
	void printVector(State);
	void clearMatrix(Matrix &);

	/** Log conf. to path.txt file */
	void log_q(State q);

	/** Random configuration in the ambient space that satisfies joint limits */
	State rand_q(int);

	/** Returns ABB's link lengths */
	State get_robots_properties() {
		State P = {b, l1x, l1y, l1z, l2a, l2b, l3, l4a, l4b, l5, l6a, l6b, l7, lEE};
		return P;
	}

	/** Performance parameters */
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}

	bool include_joint_limits = true;
};



#endif /* KDL_CLASS_H_ */
