/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Avishai Sintov, Ioan Sucan */

#include "plan.h"
#include <time.h>       /* time */

bool isStateValidC(const ob::State *state)
{
	return true;
}

void plan_C::plan(State c_start, State c_goal, double runtime, string PRMfile, int sNg_nn) {

	// construct the state space we are planning in
	ob::CompoundStateSpace *cs = new ob::CompoundStateSpace(); // Compound R^12 configuration space
	ob::StateSpacePtr A(new ob::RealVectorStateSpace(6)); // A-space - state space of the rod - R^6
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(12)); // Angles of Robot 1 & 2 - R^12
	cs->addSubspace(A, 1.0);
	cs->addSubspace(Q, 1.0);

	// set the bounds for the A=R^6
	ob::RealVectorBounds Abounds(6);
	Abounds.setLow(-30);
	Abounds.setHigh(30);

	// set the bounds for the Q=R^12 part of 'Cspace'
	ob::RealVectorBounds Qbounds(12);
	Qbounds.setLow(0, -2.88); // Robot 1
	Qbounds.setHigh(0, 2.88);
	Qbounds.setLow(1, -1.919);
	Qbounds.setHigh(1, 1.919);
	Qbounds.setLow(2, -1.919);
	Qbounds.setHigh(2, 1.22);
	Qbounds.setLow(3, -2.79);
	Qbounds.setHigh(3, 2.79);
	Qbounds.setLow(4, -2.09);
	Qbounds.setHigh(4, 2.09);
	Qbounds.setLow(5, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it - this impacts the sampler
	Qbounds.setHigh(5, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it
	Qbounds.setLow(6, -2.88); // Robot 2
	Qbounds.setHigh(6, 2.88);
	Qbounds.setLow(7, -1.919);
	Qbounds.setHigh(7, 1.919);
	Qbounds.setLow(8, -1.919);
	Qbounds.setHigh(8, 1.22);
	Qbounds.setLow(9, -2.79);
	Qbounds.setHigh(9, 2.79);
	Qbounds.setLow(10, -2.09);
	Qbounds.setHigh(10, 2.09);
	Qbounds.setLow(11, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it
	Qbounds.setHigh(11, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it

	// set the bound for the compound space
	cs->as<ob::RealVectorStateSpace>(0)->setBounds(Abounds);
	cs->as<ob::RealVectorStateSpace>(1)->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(cs);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	//si->setValidStateSamplerAllocator(allocMyValidStateSampler);

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValidC, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.05); // 3% ???

	// create start state
	ob::ScopedState<ob::CompoundStateSpace> start(Cspace);
	for (int i = 0; i < 6; i++)
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_start[i]; // Access the first component of the start a-state
	for (int i = 0; i < 12; i++)
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
	for (int i = 0; i < 12; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner(new og::CBiRRT(si, PRMfile, sNg_nn));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	cout << "Runtime: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		/*ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		std::ofstream myfile;
		myfile.open("pathRRTC.txt");
		og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		pog.printAsMatrix(myfile); // Print as matrix to file
		myfile.close();*/
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}


int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime, map_index, num_nn;
	int env = 1;

	if (argn < 5) {
		cout << "Not enough inputs.\n";
		return 1;
	}

	runtime = atof(args[1]);
	int j = atoi(args[2]);
	int k = atoi(args[3]);
	int query = atoi(args[4]);

	srand( time(NULL) );

	plan_C Plan;

	// Tests the success rate of the all feasible queries found in case 4.
	// Remember to disable poles collisions before running this !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	vector<int> ms_size = {100, 500, 1000};
	vector<int> knn_size = {2, 3, 4, 5, 6};
	num_nn = 1;

	// Load all random confs.
	Matrix Cdb;
	std::ifstream File;
	File.open("./data/abb_samples_data.txt");
	State c_temp(18);
	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 18; j++) {
			File >> c_temp[j];
		}
		Cdb.push_back(c_temp);
		i++;
	}
	File.close();
	Cdb.pop_back();

	// Load all random queries.
	Matrix I;
	File.open("./data/feasible_queries.txt");
	State i_temp(4);
	i = 0;
	while(!File.eof()) {
		for (int j = 0; j < 4; j++) {
			File >> i_temp[j];
		}
		I.push_back(i_temp);
		i++;
	}
	File.close();
	I.pop_back();

	std::ofstream ft;
	ft.open("./matlab/Benchmark_rand_sg_noObs.txt", ios::app);

	int i1 = I[query][0];
	int i2 = I[query][1];
	State c_start = Cdb[i1];
	State c_goal  = Cdb[i2];

	string PRMfile = "ms6D_" + std::to_string(ms_size[j]) + "_"  + std::to_string(knn_size[k]) + ".prm";
	cout << "*** Planning with " << PRMfile << endl;

	Plan.plan(c_start, c_goal, runtime, PRMfile, num_nn);

	// Log
	ft << ms_size[j] << "\t" << knn_size[k] << "\t" << query << "\t" << i1 << "\t" << i2 << "\t";
	ifstream FromFile;
	FromFile.open("./paths/perf_log.txt");
	string line;
	while (getline(FromFile, line))
		ft << line << "\t";
	FromFile.close();
	ft << endl;


	cout << "**************************************" << endl;
	//cout << "Completed " << (double)i/N*100 << "%." << endl;
	cout << "**************************************" << endl;
	ft.close();

	std::cout << std::endl << std::endl;

	return 0;
}


