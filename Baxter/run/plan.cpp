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

void plan_C::plan(State c_start, State c_goal, double runtime, string PRMfile, int sNg_nn, int iter_bound_num) {

	// construct the state space we are planning in
	ob::CompoundStateSpace *cs = new ob::CompoundStateSpace(); // Compound R^12 configuration space
	ob::StateSpacePtr A(new ob::RealVectorStateSpace(6)); // A-space - state space of the rod - R^6
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(14)); // Angles of Robot 1 & 2 - R^12
	cs->addSubspace(A, 1.0);
	cs->addSubspace(Q, 1.0);

	// set the bounds for the A=R^6
	ob::RealVectorBounds Abounds(6);
	Abounds.setLow(-30);
	Abounds.setHigh(30);

	// set the bounds for the Q=R^14 part of 'Cspace'
	ob::RealVectorBounds Qbounds(14);
	Qbounds.setLow(0, -2.461); // S0
	Qbounds.setHigh(0, 0.89);
	Qbounds.setLow(1, -2.147); // S1
	Qbounds.setHigh(1, +1.047);
	Qbounds.setLow(2, -3.028); // E0
	Qbounds.setHigh(2, +3.028);
	Qbounds.setLow(3, -1.6232); // E1
	Qbounds.setHigh(3, 1.0472);
	Qbounds.setLow(4, -3.059); // W0
	Qbounds.setHigh(4, 3.059);
	Qbounds.setLow(5, -1.571); // W1
	Qbounds.setHigh(5, 2.094);
	Qbounds.setLow(6, -3.059); // W2
	Qbounds.setHigh(6, 3.059);
	// Left arm
	Qbounds.setLow(7, -0.89); // S0
	Qbounds.setHigh(7, 2.462);
	Qbounds.setLow(8, -2.147); // S1
	Qbounds.setHigh(8, +1.047);
	Qbounds.setLow(9, -3.028); // E0
	Qbounds.setHigh(9, +3.028);
	Qbounds.setLow(10, -1.6232); // E1
	Qbounds.setHigh(10, 1.0472);
	Qbounds.setLow(11, -3.059); // W0
	Qbounds.setHigh(11, 3.059);
	Qbounds.setLow(12, -1.571); // W1
	Qbounds.setHigh(12, 2.094);
	Qbounds.setLow(13, -3.059); // W2
	Qbounds.setHigh(13, 3.059);

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
	for (int i = 0; i < c_start.size()-6; i++)
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
	for (int i = 0; i < c_goal.size()-6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner(new og::CBiRRT(si, PRMfile, sNg_nn, iter_bound_num));

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
	double runtime, map_index, num_nn, iter_bound_num;
	int env = 1;

	if (argn == 1)
		runtime = 1; // sec
	else
		runtime = atof(args[1]);

	if (argn>2) {
		map_index = atoi(args[2]);
		num_nn = atoi(args[3]);
		iter_bound_num = atoi(args[4]);
	}
	else {
		map_index = 1;
		num_nn = 2;
		iter_bound_num = 1e9;
	}


	plan_C Plan;

	// PRM data
	//vector<string> PRMfile = {"ms6D_100_2.prm", "ms6D_1000_2.prm","ms6D_2000_2.prm","ms6D_5000_2.prm"};
	//vector<string> PRMfile = {"ms6D_100_2.prm", "ms6D_300_2.prm", "ms6D_750_2.prm", "ms6D_1000_2.prm","ms6D_1500_2.prm","ms6D_2000_2.prm"};
	//vector<string> PRMfile = {"ms6D_100_3.prm", "ms6D_750_3.prm", "ms6D_1500_3.prm","ms6D_300_3.prm"};
	//vector<double> load_time = {2, 4, 8, 9, 13, 13}; // Average time required to load the map

	//vector<string> PRMfile = {"ms6D_100_2.prm","ms6D_100_3.prm","ms6D_100_4.prm","ms6D_100_5.prm","ms6D_100_6.prm"};
	//vector<double> load_time = {1.5, 3, 3.5, 4, 5}; // Average time required to load the map
	//vector<string> PRMfile = {"ms6D_500_2.prm","ms6D_500_3.prm","ms6D_500_4.prm","ms6D_500_5.prm","ms6D_500_6.prm"};
	//vector<double> load_time = {4, 6.5, 10, 12, 15}; // Average time required to load the map
	//vector<string> PRMfile = {"ms6D_1000_2.prm","ms6D_1000_3.prm","ms6D_1000_4.prm","ms6D_1000_5.prm","ms6D_100_2.prm","ms6D_100_3.prm","ms6D_100_4.prm","ms6D_100_5.prm","ms6D_100_6.prm"};
	//vector<double> load_time = {8, 15, 16, 20,1.5, 3, 3.5, 4, 5}; // Average time required to load the map

	vector<string> PRMfile = {"ms6D_500_4.prm", "ms6D_1000_4.prm"};
	vector<double> load_time = {10, 16};

	State c_start, c_goal;
	if (env == 1) { // Example 1
		//c_start = {-0.228674, -5.79388, 0.376973, -12.1839, 0.804729, 7.27214, -0.350352, -1.5366, 2.87881, -0.453977, -1.50315, 1.44179, -0.604059, -0.429175, -1.34472, 1.10585, -0.78678, 2.76589, -0.860165, -1.67489};
		//c_goal = {1.13317, -4.08401, 2.74606, 6.78602, 11.6337, -5.10359, 0.862361, -0.234115, -2.1993, -0.763042, 2.05378, 0.149986, 2.24325, 0.615206, -0.584064, -1.06856, -1.15379, 1.68501, -0.847547, -1.82553};
		c_start = {0, -4.6, 0, 0, 0, 0, 0, -0.1, 0, -1, -1.5, -1.4, -0.5, 1.61661, 0.494671, -1.15783, 0.499994, 0.968899, -0.318459, 2.78032};
		c_goal = {2.3, -5.5, 0, 0, 0, 0, -0.9, -0.9, 0.8, 0.7, -2.6, 0.9, -0.3, -0.176201, -0.610284, -0.57566, -0.603606, 2.87572, -0.0423817, -2.10024};
	}
	else if (env == 2) {

	}

	int mode = 1;
	switch (mode) {
	case 1 : {
		map_index = 0;
		num_nn = 1;
		iter_bound_num = 1e9;
		Plan.plan(c_start, c_goal, runtime+load_time[map_index], PRMfile[map_index], num_nn, iter_bound_num);

		break;
	}
	case 2 : { // Benchmark with the same scenario and PRMmaps over N trials
		std::ofstream ft;
		ft.open("./matlab/time_sameScenario_30_750_2.txt", ios::app);


		map_index = 0;
		num_nn = 2;
		iter_bound_num = 1e9;
		int N = 100;
		for (int i = 0; i < N; i++) {
			cout << "**************************************" << endl;
			cout << "Completed " << (double)i/N*100 << "%." << endl;
			cout << "**************************************" << endl;
			Plan.plan(c_start, c_goal, runtime+load_time[map_index], PRMfile[map_index], num_nn, iter_bound_num);

			// Log
			ifstream FromFile;
			FromFile.open("./paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				ft << line << "\t";
			FromFile.close();
			ft << endl;
		}
		ft.close();
		break;
	}
	case 3 : { // Benchmark with the same scenario and different PRMmaps over N trials
		vector<int> prmk = {2,3,4,5,6};
		int k = 2;
		iter_bound_num = 1e9;
		std::ofstream ft;

		for( int j = 2; j < 3/*PRMfile.size()*/; j++) {

			int N = 100;
			for (int i = 0; i < N; i++) {
				cout << "**************************************" << endl;
				cout << "Completed " << (double)i/N*100 << "%." << endl;
				cout << "**************************************" << endl;
				Plan.plan(c_start, c_goal, runtime+load_time[j], PRMfile[j], k, iter_bound_num);

				// Log
				ft.open("./matlab/time_poleScene_MS1000_new.txt", ios::app);
				ft << prmk[j] << "\t";
				ifstream FromFile;
				FromFile.open("./paths/perf_log.txt");
				string line;
				while (getline(FromFile, line))
					ft << line << "\t";
				FromFile.close();
				ft << endl;
				ft.close();
			}
		}
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}


