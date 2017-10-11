/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

/*
myStateValidityCheckerClass::myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) {

}*/

#include "StateValidityChecker.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Get state of rod
	for (unsigned i = 0; i < nq; i++)
		q[i] = Q->values[i]; // Get state of robots
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Get state of rod
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Set state of rod
	for (unsigned i = 0; i < nq/2; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+nq/2]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a, State q) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		A->values[i] = a[i];

	for (unsigned i = 0; i < nq; i++)
		Q->values[i] = q[i];
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		A->values[i] = a[i];
	for (unsigned i = 0; i < nq/2; i++) {
		Q->values[i] = q1[i];
		Q->values[i+nq/2]= q2[i];
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < na; i++)
		A->values[i] = a[i];
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	State a(na), q1(nq/2), q2(nq/2);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Set state of rod
	for (unsigned i = 0; i < nq/2; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+nq/2]; // Set state of robot1
	}

	cout << "a: "; kdl::printVector(a);
	cout << "q1: "; kdl::printVector(q1);
	cout << "q2: "; kdl::printVector(q2);
}

// ----------------------- v GD functions v ----------------------------

bool StateValidityChecker::GDsample(ob::State *st) {

	State a(na), q(nq), q1(nq/2), q2(nq/2);

	if (!GDsample(a, q))
		return false;
cout << __LINE__ << endl;
	updateStateVector(st, a, q);
	return true;
}

bool StateValidityChecker::GDsample(State &a, State &q) {

	bool flag = true;
	while (flag) {
		// Random rod configuration
		for (int i = 0; i < na; i++)
			a[i] = ((double) rand() / (RAND_MAX)) * 2 * 30 - 30;
		if (!isRodFeasible(a))
			continue;
		Matrix Q = getT(get_Points_on_Rod()-1);

		for (int k = 0; k < 10; k++) {

			// Random joint angles
			for (int i = 0; i < nq; i++)
				q[i] = ((double) rand() / (RAND_MAX)) * 2 * PI_ - PI_;

			if (!GD(q, Q)) // GD checks for joint limits
				continue;
			q = get_GD_result();

			if (withObs && collision_state(getPMatrix(), q))
				continue;

			flag = false;
			break;
		}
	}

	return true;
}

bool StateValidityChecker::GDproject(ob::State *st) {

	State a(na), q(nq);

	// Check that 'a' on the random state is feasible
	retrieveStateVector(st, a, q);

	bool valid = GDproject(a, q);

	if (valid) {
		updateStateVector(st, a, q);
		return true;
	}

	return false;
}

bool StateValidityChecker::GDproject(State a, State &q) {

	kdl::IK_counter++;
	clock_t sT = clock();

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	if (!GD(q, Q)) { // GD checks for joint limits
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	q = get_GD_result();

	if (withObs && collision_state(getPMatrix(), q)) {
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
	return true;
}

bool StateValidityChecker::GDproject(State &q, Matrix Q) {

	State q1(nq/2), q2(nq/2);

	kdl::IK_counter++;
	clock_t sT = clock();

	if (!GD(q, Q)) { // GD checks for joint limits
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	q = get_GD_result();

	kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
	return true;
}

// ----------------------- ^ GD functions ^ ----------------------------


// ---------------------------------------------------------------

void StateValidityChecker::log_q(const ob::State *st) {
	State a(na), q(nq);

	retrieveStateVector(st, a, q);
	log_q(a, q);
}

void StateValidityChecker::log_q(State a, State q) {
	std::ofstream qfile, afile, pfile, ai;
	qfile.open("../paths/path.txt");
	afile.open("../paths/afile.txt");
	pfile.open("../paths/rod_path.txt");

	qfile << 1 << endl;
	pfile << 501 << endl;

	for (int j = 0; j < na; j++)
		afile << a[j] << " ";
	for (int j = 0; j < nq; j++)
		qfile << q[j] << " ";

	rod_solve(a);
	State temp(3);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	qfile.close();
	afile.close();
	pfile.close();
}

void StateValidityChecker::log_q(Matrix A, Matrix M) {
	std::ofstream qfile, afile, pfile, ai;
	qfile.open("./paths/path.txt");
	afile.open("./paths/afile.txt");
	pfile.open("./paths/rod_path.txt");

	qfile << M.size() << endl;
	pfile << M.size()*501 << endl;

	for (int i = 0; i < M.size(); i++) {
		for (int j = 0; j < nq; j++)
			qfile << M[i][j] << " ";
		for (int j = 0; j < na; j++)
			afile << A[i][j] << " ";
		qfile << endl;
		afile << endl;

		rod_solve(A[i]);
		State temp(3);
		// Log points on rod to file
		for (int k = 0; k < get_Points_on_Rod(); k++) {
			temp = getP(k);
			pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
		}
		pfile << endl;
	}

	qfile.close();
	afile.close();
	pfile.close();
}

// ---------------------------------------------------------------

double StateValidityChecker::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

State StateValidityChecker::join_Vectors(State q1, State q2) {

	State q(q1.size()+q2.size());

	for (int i = 0; i < q1.size(); i++) {
		q[i] = q1[i];
		q[i+q1.size()] = q2[i];
	}

	return q;
}

void StateValidityChecker::seperate_Vector(State q, State &q1, State &q2) {

	for (int i = 0; i < q.size()/2; i++) {
		q1[i] = q[i];
		q2[i] = q[i+q.size()/2];
	}
}

double StateValidityChecker::StateDistance(const ob::State *s1, const ob::State *s2) {

	State aa(na), qa(nq);
	State ab(na), qb(nq);

	retrieveStateVector(s1, aa, qa);
	retrieveStateVector(s2, ab, qb);

	double sum = 0;
	for (int i=0; i < aa.size(); i++)
		sum += pow(aa[i]-ab[i], 2);
	for (int i=0; i < qa.size(); i++)
		sum += pow(qa[i]-qb[i], 2);
	return sqrt(sum);
}

double StateValidityChecker::AngleDistance(const ob::State *st1, const ob::State *st2) {
	State q1(nq), q2(nq), a_dummy(na);
	retrieveStateVector(st1, a_dummy, q1);
	retrieveStateVector(st2, a_dummy, q2);

	double sum = 0;
	for (int i = 0; i < nq; i++)
		sum += pow(q1[i]-q2[i], 2);
	return sqrt(sum);
}


// ====================== Check validity ===============================

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(StateVector &S) {

	isValid_counter++;

	if (GDproject(S.a, S.q))
		return true;

	return false;
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(ob::State *st) {

	isValid_counter++;

	return GDproject(st);
}

bool StateValidityChecker::check_angles_offset(State qn, State q) {

	double lim = 0.4;

	for (int i=0; i < q.size(); i++) {
		if (fabs(q[i]-qn[i]) > lim)
			return false;
	}

	return true;
}

bool StateValidityChecker::check_angles_offset(ob::State *nstate, State q) {

	State qn(q.size()), a(na);

	retrieveStateVector(nstate, a, qn);

	double lim = 0.4;

	for (int i=0; i < q.size(); i++) {
		if (fabs(q[i]-qn[i]) > lim)
			return false;
	}

	return true;
}

bool StateValidityChecker::check_rigid_motion(const ob::State *st1, const ob::State *st2, NODE aPRM1, NODE aPRM2) {

	q_rigid_path.clear();

	double d = AngleDistance(st1, st2);
	if (d < RBS_tol)
		return true;
	int nd = d / RBS_tol;

	Matrix Q = aPRM1.index_indicator ? ms.T_end[aPRM1.index] : subms.T_end[aPRM1.index];
	Matrix P = aPRM1.index_indicator ? ms.P[aPRM1.index] : subms.P[aPRM1.index];

	// temporary storage for the checked state
	ob::State *test = mysi_->allocState();
	State a_dummy(na), q(nq), q1(nq/2), q2(nq/2), q_prev(nq);
	retrieveStateVector(st1, a_dummy, q_prev);

	bool valid = true;
	for (int i = 1; i < nd; i++) {
		stateSpace_->interpolate(st1, st2, (double)i / (double)(nd-1), test);
		retrieveStateVector(test, a_dummy, q);

		if (!GDproject(q, Q))
			valid = false;

		if (collision_state(P, q) || !check_angles_offset(q, q_prev))
			valid = false;

		if (!valid)
			break;

		q_rigid_path.push_back(q);

		q_prev = q;
	}

	return valid;
}

// =============================== LOG ====================================

void StateValidityChecker::LogPerf2file() {

	std::ofstream myfile;
	myfile.open("./paths/perf_log.txt");

	myfile << final_solved << endl;
	myfile << PlanDistance << endl; // Distance between nodes 1
	myfile << total_runtime << endl; // Overall planning runtime 2
	myfile << kdl::get_IK_counter() << endl; // How many IK checks? 5
	myfile << kdl::get_IK_time() << endl; // IK computation time 6
	myfile << get_collisionCheck_counter() << endl; // How many collision checks? 7
	myfile << get_collisionCheck_time() << endl; // Collision check computation time 8
	myfile << get_isValid_counter() << endl; // How many nodes checked 9
	myfile << nodes_in_path << endl; // Nodes in path 10
	myfile << nodes_in_trees << endl; // 11
	myfile << get_odes_counter() << endl;
	myfile << get_valid_odes_counter() << endl;
	myfile << get_odes_time() << endl;
	myfile << get_add_startNgoal_runtime() << endl;
	myfile << load_runtime << endl;

	myfile.close();
}
