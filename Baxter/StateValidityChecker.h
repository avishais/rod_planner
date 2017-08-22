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

class StateValidityChecker :  public PRMGenerator, public Baxter, public collisionDetection { //public rod_ode,
public:
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get())//, Baxter(), collisionDetection()
{
		q_temp.resize(6);
		close_chain_return_IK.resize(2);

		Nj = get_num_joint();
}; //Constructor // Avishai
	StateValidityChecker() {Nj = get_num_joint();};

	// Validity check
	bool close_chain(const ob::State *state, int);
	bool close_chain(const ob::State *state, int, Matrix);
	bool generate_feasible_config(const ob::State *state);
	bool generate_feasible_config(State a, State &q1, State &q2);
	bool isValid(const ob::State *state, int active_chain, int IK_sol);
	bool checkMotion(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol);
	bool check_neighbors(ob::State *dstate, ob::State *nstate, int active_chain);
	bool check_angles_offset(ob::State *dstate, State, State, int active_chain);
	bool check_angles_offset(State qa, State qb);
	bool check_rigid_motion(const ob::State *st1, const ob::State *st2, NODE aPRM1, NODE aPRM2);

	// Retrieve and update
	void retrieveStateVector(const ob::State *state, State &a);
	void retrieveStateVector(const ob::State *state, State &q1, State &q2);
	void retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2);
	void updateStateVector(const ob::State *state, State a, State q1, State q2);
	void updateStateVector(const ob::State *state, State q1, State q2);
	void updateStateVector(const ob::State *state, State a);
	void printStateVector(const ob::State *state);

	void defaultSettings();
	double normDistance(State, State);
	double StateDistance(const ob::State *s1, const ob::State *s2);
	double AngleDistance(const ob::State *st_a, const ob::State *st_b);

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	State get_close_chain_return_IK() {
		return close_chain_return_IK;
	}

	// Performance parameters
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}

	int Nj; // Number of joints in each arm
private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	State q_temp;
	int valid_solution_index;
	State close_chain_return_IK;

};





#endif /* CHECKER_H_ */
