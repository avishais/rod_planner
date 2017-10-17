/*
 * Checker.cpp
 *
 *  Created on: April 14, 2017
 *      Author: Avishai Sintov
 */

#include "StateValidityChecker.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < 6; i++)
		a[i] = A->values[i]; // Get state of rod
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &q1, State &q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	//const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < Nj; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+Nj]; // Set state of robot1
	}
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++)
		a[i] = A->values[i]; // Set state of rod
	for (unsigned i = 0; i < Nj; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+Nj]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++)
		A->values[i] = a[i];
	for (unsigned i = 0; i < Nj; i++) {
		Q->values[i] = q1[i];
		Q->values[i+Nj]= q2[i];
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < Nj; i++) {
		Q->values[i] = q1[i];
		Q->values[i+Nj]= q2[i];
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < 6; i++)
		A->values[i] = a[i];
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);
	//const ob::RealVectorStateSpace::StateType *AP = C_state->as<ob::RealVectorStateSpace::StateType>(2);

	State a(6), q1(Nj), q2(Nj);

	for (unsigned i = 0; i < 6; i++)
		a[i] = A->values[i]; // Set state of rod
	for (unsigned i = 0; i < Nj; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+Nj]; // Set state of robot1
	}

	cout << "a: "; printVector(a);
	cout << "q1: "; printVector(q1);
	cout << "q2: "; printVector(q2);
}

bool StateValidityChecker::close_chain(const ob::State *state, int active_chain) {
	// c is a 20 dimensional vector composed of [a q1 q2]
	// active_chain = 0 or 1.

	State a(6), q1(Nj), q2(Nj), q1_temp;
	retrieveStateVector(state, a, q1, q2);

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	bool valid = false;
	if (!active_chain) {
		//q2 = generate_random_arm_configuration(2);
		FKsolve_rob(q1, 1);
		Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
		T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
		for (int i = 0; i < 100 && !valid; i++) // Try to close the chain 100 times
			valid = IKsolve_rob(T2, q2, 2);
		if (!valid)
			return false;
		q2 = get_IK_solution_q2();
		cout << "******* Found IK\n";
	}

	if (active_chain || !valid) {
		q1 = generate_random_arm_configuration(1);
		Matrix Tinv = Q;
		InvertMatrix(Q, Tinv); // Invert matrix
		FKsolve_rob(q2, 2);
		Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
		T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
		for (int i = 0; i < 100 && !valid; i++) // Try to close the chain 100 times
			valid = IKsolve_rob(T1, q1, 1);
		if (!valid)
			return false;
		q1 = get_IK_solution_q1();
	}

	updateStateVector(state, q1, q2);
	return true;
}


bool StateValidityChecker::close_chain(const ob::State *state, int active_chain, Matrix Q) {
	// c is a 20 dimensional vector composed of [a q1 q2 ik]

	State a(6), q1(Nj), q2(Nj), q1_temp;
	retrieveStateVector(state, a, q1, q2);

	bool valid = false;
	if (!active_chain) {
		FKsolve_rob(q1, 1);
		Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
		T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
		for (int i = 0; i < 100 && !valid; i++) // Try to close the chain 100 times
			valid = IKsolve_rob(T2, generate_random_arm_configuration(2), 2);
		if (!valid)
			return false;
		q2 = get_IK_solution_q2();
	}

	if (active_chain || !valid) {
		Matrix Tinv = Q;
		InvertMatrix(Q, Tinv); // Invert matrix
		FKsolve_rob(q2, 2);
		Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
		T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
		for (int i = 0; i < 100 && !valid; i++) // Try to close the chain 100 times
			valid = IKsolve_rob(T1, generate_random_arm_configuration(1), 1);
		if (!valid)
			return false;
		q1 = get_IK_solution_q1();
	}

	updateStateVector(state, q1, q2);
	return true;
}

bool StateValidityChecker::generate_feasible_config(const ob::State *state) {
	
	State a(6), q1(Nj), q2(Nj);
	retrieveStateVector(state, a, q1, q2);
	
	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);
	Matrix P = getPMatrix();
	
	for (int i = 0; i < 100; i++) {
		bool valid = false;
		State q1 = generate_random_arm_configuration(1);
		FKsolve_rob(q1, 1);
		Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
		T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
		for (int i = 0; i < 100 && !valid; i++) // Try to close the chain 100 times
			valid = IKsolve_rob(T2, generate_random_arm_configuration(2), 2);
		if (valid) {
			q2 = get_IK_solution_q2();
			if (collision_state(P, q1, q2))
				continue;
			updateStateVector(state, q1, q2);
			cout << "Found configuration for the rod." << endl;
			return true;
		}
	}
	
	cout << "Failed to find configuration for the rod." << endl;
	return false;
}

bool StateValidityChecker::generate_feasible_config(State a, State &q1, State &q2) {

	//q1.resize(Nj);
	//q2.resize(Nj);

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);
	Matrix P = getPMatrix();

	for (int i = 0; i < 60; i++) {
		bool valid = false;
		q1 = generate_random_arm_configuration(1);
		FKsolve_rob(q1, 1);
		Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
		T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
		for (int i = 0; i < 16 && !valid; i++) // Try to close the chain 100 times
			valid = IKsolve_rob(T2, generate_random_arm_configuration(2), 2);
		if (valid) {
			q2 = get_IK_solution_q2();
			if (collision_state(P, q1, q2))
				continue;
			//cout << "Found configuration for the rod." << endl;
			return true;
		}
	}

	//cout << "Failed to find configuration for the rod." << endl;
	return false;
}


// ------------------- Distance functions

double StateValidityChecker::AngleDistance(const ob::State *st_a, const ob::State *st_b) {
	State q1_a(Nj), q2_a(Nj), q1_b(Nj), q2_b(Nj);
	retrieveStateVector(st_a, q1_a, q2_a);
	retrieveStateVector(st_b, q1_b, q2_b);

	double sum = 0;
	for (int i=0; i < Nj; i++)
		sum += pow(q1_a[i]-q1_b[i], 2) + pow(q2_a[i]-q2_b[i], 2);
	return sqrt(sum);
}

double StateValidityChecker::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

double StateValidityChecker::StateDistance(const ob::State *s1, const ob::State *s2) {

	State aa(6), qa1(Nj), qa2(Nj);
	State ab(6), qb1(Nj), qb2(Nj);

	retrieveStateVector(s1, aa, qa1, qa2);
	retrieveStateVector(s2, ab, qb1, qb2);

	double sum = 0;
	for (int i=0; i < aa.size(); i++)
		sum += pow(aa[i]-ab[i], 2);
	for (int i=0; i < qa1.size(); i++)
		sum += pow(qa1[i]-qb1[i], 2) + pow(qa2[i]-qb2[i], 2);
	return sqrt(sum);
}

// ------------------- Validity check

bool StateValidityChecker::check_rigid_motion(const ob::State *st1, const ob::State *st2, NODE aPRM1, NODE aPRM2) {

	printStateVector(st1);
	printStateVector(st2);

	int active_chain = rand() % 2;

	double d = AngleDistance(st1, st2);
	int nd = d/0.05;

	Matrix Q = aPRM1.index_indicator ? ms.T_end[aPRM1.index] : subms.T_end[aPRM1.index];
	Matrix P = aPRM1.index_indicator ? ms.P[aPRM1.index] : subms.P[aPRM1.index];

	// temporary storage for the checked state
	ob::State *test = mysi_->allocState();
	State q1(6), q2(6), q1_prev(6), q2_prev(6);

	for (int active_chain = 0; active_chain < 2; active_chain++) {
		retrieveStateVector(st1, q1_prev, q2_prev);

		bool valid = true;
		for (int i = 1; i < nd; i++) {
			stateSpace_->interpolate(st1, st2, (double)i / (double)(nd-1), test);
			retrieveStateVector(test, q1, q2);

			switch (active_chain) {
			case 0:
				if (calc_specific_IK_solution_R1(Q, q1, q2)) {
					q2 = get_IK_solution_q2();
					if (!check_angles_offset(q2, q2_prev))
						valid = false;

					if (collision_state(P, q1, q2))
						valid = false;
				}
				else
					valid = false;
				break;
			case 1:
				if (calc_specific_IK_solution_R2(Q, q2, q1)) {
					q1 = get_IK_solution_q1();
					if (!check_angles_offset(q1, q1_prev))
						valid = false;

					if (collision_state(P, q1, q2))
						valid = false;
				}
				else
					valid = false;
			}
			if (!valid)
				break;

			q1_prev = q1;
			q2_prev = q2;
		}
		if (valid)
			return true;
	}
	return false;
}

bool StateValidityChecker::check_angles_offset(ob::State *dstate, State q1, State q2, int active_chain) {

	State q1n(6), q2n(6);

	//retrieveStateVector(nstate, a, q1, q2);
	retrieveStateVector(dstate, q1n, q2n);

	double lim = 0.6;

	if (!active_chain) {
		for (int i=0; i < q2.size(); i++) {
			if (fabs(q2[i]-q2n[i]) > lim)
				return false;
		}
	}
	else {
		for (int i=0; i < q1.size(); i++) {
			if (fabs(q1[i]-q1n[i]) > lim)
				return false;
		}
	}

	return true;
}

bool StateValidityChecker::check_angles_offset(State qa, State qb) {

	double lim = 0.6;

	for (int i=0; i < qa.size(); i++) {
		if (fabs(qa[i]-qb[i]) > lim)
			return false;
	}

	return true;
}

