/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "../roadmap/classes/Rod_ODE_class.h"
#include "./classes/robots_class.h"
#include "./classes/collisionDetection.h"
#include "../roadmap/prm_gen.hpp"

#include <iostream>

namespace ob = ompl::base;
using namespace std;

class StateValidityChecker :  public PRMGenerator, public two_robots, public collisionDetection { //public rod_ode, 
public:
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), two_robots({-1085.85/2, 0, 0 }, {1085.85/2, 0, PI}), collisionDetection(1085.85,0,0,0)
{
		q_temp.resize(6);
		close_chain_return_IK.resize(2);
}; //Constructor // Avishai
	StateValidityChecker() : two_robots({-1085.85/2, 0, 0 }, {1085.85/2, 0, PI}), collisionDetection(1085.85,0,0,0) {};


	// Validity check
	bool close_chain(const ob::State *state, int);
	bool close_chain(const ob::State *state, int, Matrix);
	bool isValid(const ob::State *state, int active_chain, int IK_sol);
	bool checkMotion(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol);
	bool check_neighbors(ob::State *dstate, ob::State *nstate, int active_chain);
	bool check_angles_offset(ob::State *dstate, Vector, Vector, int active_chain);
	bool check_angles_offset(Vector qa, Vector qb);
	Vector identify_state_ik(const ob::State *state);
	Vector identify_state_ik(const ob::State *state, Matrix Q);
	bool check_rigid_motion(const ob::State *st1, const ob::State *st2, VectorInt ik1, VectorInt ik2, NODE aPRM1, NODE aPRM2);

	// Retrieve and update
	void retrieveStateVector(const ob::State *state, Vector &a);
	void retrieveStateVector(const ob::State *state, Vector &q1, Vector &q2);
	void retrieveStateVector(const ob::State *state, Vector &a, Vector &q1, Vector &q2);
	void updateStateVector(const ob::State *state, Vector a, Vector q1, Vector q2);
	void updateStateVector(const ob::State *state, Vector q1, Vector q2);
	void updateStateVector(const ob::State *state, Vector a);
	void printStateVector(const ob::State *state);

	void defaultSettings();
	double normDistance(Vector, Vector);
	double StateDistance(const ob::State *s1, const ob::State *s2);
	double AngleDistance(const ob::State *st_a, const ob::State *st_b);

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	Vector get_close_chain_return_IK() {
		return close_chain_return_IK;
	}

	// Performance parameters
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}
	double check_rigid_time;
private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	Vector q_temp;
	int valid_solution_index;
	Vector close_chain_return_IK;

};





#endif /* CHECKER_H_ */
