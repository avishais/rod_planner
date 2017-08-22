/*
 * plan_C_space.h
 *
 *  Created on: Nov 10, 2016
 *      Author: avishai
 */

#ifndef PLAN_C_SPACE_H_
#define PLAN_C_SPACE_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

//#include "mySampler.h"

// Built-in planners
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/rrt/RRT.h>
//#include <ompl/geometric/planners/rrt/LazyRRT.h>
//#include <ompl/geometric/planners/rrt/BiTRRT.h>
//#include <ompl/geometric/planners/sbl/SBL.h>
//#include <ompl/geometric/planners/sbl/pSBL.h>
//#include <ompl/geometric/planners/prm/PRM.h>
//#include <ompl/geometric/planners/prm/LazyPRM.h>

// Modified and custom planners
#include "myRRTConnectBaxter.h"
//#include "decoupled_rod.h"
//#include "testplanner.h"

#include "StateValidityCheckerBaxter.h"
//#include "Rod_ODE_class.h"

// Standard libraries
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

//rod_ode rod;

//StateValidityChecker svcc; // The checker class

bool isStateValidC(const ob::State *state);

// Prototypes
class plan_C //: public StateValidityChecker
{
public:

	plan_C() {};

	void plan(State c_start, State c_goal, double runtime, string, int);

	bool solved_bool;
	double total_runtime;
	int ode_count;
};

#endif /* PLAN_C_SPACE_H_ */
