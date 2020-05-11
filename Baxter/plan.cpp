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

bool plan_C::plan(State c_start, State c_goal, double runtime, string PRMfile, int sNg_nn) {

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

	// set the bounds for the Q=R^12 part of 'Cspace'
	ob::RealVectorBounds Qbounds(14);
	// Right arm
	Qbounds.setLow(0, -2.461); // S0
	Qbounds.setHigh(0, 0.89);
	Qbounds.setLow(1, -2.147); // S1
	Qbounds.setHigh(1, +1.047);
	Qbounds.setLow(2, -3.028); // E0
	Qbounds.setHigh(2, +3.028);
	Qbounds.setLow(3, -0.052); // E1
	Qbounds.setHigh(3, 2.618);
	Qbounds.setLow(4, -3.059); // W0
	Qbounds.setHigh(4, 3.059);
	Qbounds.setLow(5, -1.571); // W1
	Qbounds.setHigh(5, 2.094);
	Qbounds.setLow(6, -3.059); // W2
	Qbounds.setHigh(6, 3.059);
	// Left arm
	Qbounds.setLow(7, -0.89); // S0
	Qbounds.setHigh(7, 2.461);
	Qbounds.setLow(8, -2.147); // S1
	Qbounds.setHigh(8, +1.047);
	Qbounds.setLow(9, -3.028); // E0
	Qbounds.setHigh(9, +3.028);
	Qbounds.setLow(10, -0.052); // E1
	Qbounds.setHigh(10, 2.618);
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
	for (int i = 0; i < 7; i++) {
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i+7] = c_start[i+7+6];//q2[i];
	}

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
	for (int i = 0; i < 7; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i+7] = c_goal[i+7+6];//q2[i];
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner(new og::RRTConnect(si, PRMfile, sNg_nn));
	//ob::PlannerPtr planner(new og::RRT(si));
	//ob::PlannerPtr planner(new og::BiTRRT(si));
	//ob::	PlannerPtr planner(new og::LazyRRT(si));
	//ob::PlannerPtr planner(new og::pSBL(si));
	//ob::PlannerPtr planner(new og::PRM(si));
	//ob::PlannerPtr planner(new og::LazyPRM(si));
	//ob::PlannerPtr planner(new ompl::decoupled_rod(si));
	//ob::PlannerPtr planner(new ompl::testplanner(si));

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
	total_runtime = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Runtime: " << total_runtime << endl;

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

	return solved_bool;
}



void load_random_nodes(Matrix &Cdb) {

	std::ifstream File;
	File.open("./gen_random_conf/randomnodes_contGrasp.txt");

	State c_temp(20);

	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 20; j++) {
			File >> c_temp[j];
			//cout << c_temp[j] << " ";
		}
		//cout << endl;
		Cdb.push_back(c_temp);
		i++;
	}
	File.close();
	Cdb.pop_back();
}

void load_query_points(Matrix &C) {
	State c_temp(18);

	ifstream fq;
	fq.open("query.txt");
	int i = 0;
	while(!fq.eof()) {
		for (int j=0; j < 18; j++) {
			fq >> c_temp[j];
		}
		C.push_back(c_temp);
		i++;
	}
	fq.close();
}


double distance(State a, State b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}

bool extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("./path/perf_log.txt");

	bool success = false;
	string line;
	int j = 1;
	while (getline(FromFile, line)) {
		ToFile << line << "\t";
		if (j==1 && stoi(line))
			success = true;
		j++;
	}

	FromFile.close();

	return success;
}


int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime, map_index, num_nn;

	srand (time(NULL));

	if (argn == 1)
		runtime = 1; // sec
	else
		runtime = atof(args[1]);

	if (argn==4) {
		map_index = atof(args[2]);
		num_nn = atof(args[3]);
	}
	else {
		map_index = 1;
		num_nn = 2;
	}


	plan_C Plan;

	Matrix Cdb;
	load_random_nodes(Cdb);

	// PRM data
	//vector<string> PRMfile = {"ms6D_100_2.prm", "ms6D_1000_2.prm","ms6D_2000_2.prm","ms6D_5000_2.prm"};
	vector<string> PRMfile = {"ms6D_100_2.prm", "ms6D_300_2.prm", "ms6D_750_2.prm", "ms6D_1000_2.prm","ms6D_1500_2.prm","ms6D_2000_2.prm"};
	vector<double> load_time = {1.5, 4, 8, 9, 13, 13}; // Average time required to load the map

	int mode = 3;
	switch (mode) {
	case 1 : {
		std::ofstream ft;
		ft.open("./stats/random_trials_Perp.txt", ios::app);

		int p1, p2;
		for (int j = 0; j < 200; j++) {
			do {
				p1 = rand() % Cdb.size();
				p2 = rand() % Cdb.size();
			} while (p1==p2);

			cout << p1 << " " << p2 << endl;

			map_index = 0;//rand() % 6;
			num_nn = 2;
			if (Plan.plan(Cdb[p1], Cdb[p2], runtime+load_time[map_index], PRMfile[map_index], num_nn)) {
				cout << "Found solution for random query.\n";
				//break;
			}

			// Log
			ft << p1 << "\t" << p2 << "\t" << map_index << "\t";
			bool suc = extract_from_perf_file(ft);
			ft << endl;
		}
		ft.close();
		break;
	}
	case 2 : {

		//State c_start = {2.17, -4.2, 2, 1.9, 9.6, -2.4, 0.0873, 0.1745,  0, 0, 0.1745, 0, 0.972532, 0.309214, 0.505241, 0.848896, -1.59516, 0.0524666};
		//State c_goal = {0, -4, 0, -5, 0, 3, 1.9, 0.6726, 0.1745, -1.5707, 0.7, -2.6, -0.9826, 1.1060, 0.4769, -0.5696, -1.5875, -0.9615};
		//State c_start = {0.5911,   -1.2856,   -5.8199 ,  -0.7626 ,   0.3748  ,  0.0214, -0.390328,0.612012,-0.281784,-2.10627,-1.01849,-2.05768,-0.00197797,0.888097,-0.686747,-0.664729,-0.661837,-1.75615 };
		//State c_goal = {-0.5938  , -2.3700 ,  -5.0451 ,  -0.2548  ,  0.8103 ,  -0.4187, 0.0444899,0.441007,-0.275984,-2.51569,-0.948493,-1.57105,0.0492653,0.884583,-0.607802,-0.627877,-0.618528,-1.85671};

		//State c_start = {2.17,-4.2,2,1.9,9.6,-2.4,   0.163367,-2.07083,-0.0491402,0.63468,-1.72675,0.654779,1.20807,   -0.451364,-0.918904,0.75261,-0.388954,-2.50482,1.30017,-1.83258};
		//State c_start = {2.17,-4.2,2,1.9,9.6,-2.4,   -0.146017, 0.218054, -1.73545, 0.448191, -0.743789, 1.93503, -2.85199, 1.12108, 0.428385, -3.00426, 0.489287, -0.316586, 0.73614, -2.21643};
		//State c_goal = {0,-4,0,-5,0,3,   -0.744613,0.475527,2.25127,0.26767,-1.33456,1.87308,-1.11176,   1.38322,-0.200921,0.261213,-0.0107912,2.21554,-1.12716,0.272123};

		//State c_start = {0, 3.2, 0, 0, 0, 0,   -0.146017, 0.218054, -1.73545, 0.448191, -0.743789, 1.93503, -2.85199, 1.12108, 0.428385, -3.00426, 0.489287, -0.316586, 0.73614, -2.21643};
		//State c_goal = {0, -4, 0, -5, 0, 3,   -0.744613,0.475527,2.25127,0.26767,-1.33456,1.87308,-1.11176,   1.38322,-0.200921,0.261213,-0.0107912,2.21554,-1.12716,0.272123};

		State c_start = Cdb[55];//{4.99721, 2.57765 ,-0.266556, 3.62154, -16.8713, 18.0325, 0.272898, -0.660043, 0.571277, 0.231013, 0.100897 ,-0.0446562, -2.57338, 0.950578, -1.25627 ,-0.163598, 0.535093, 2.95328 ,0.6372, 0.784733}; ;
		State c_goal = Cdb[19];//{0.616185, 3.66523, -1.91427, 2.00706 ,-10.1803, 11.9301, -0.275121, -0.514129, 0.565935, 0.150455, 0.744461, 0.891834, -2.29583, 0.823761, -0.100183, -1.10103, -0.00924622, 2.96437, 0.15459, 0.501461};

		map_index = rand() % 5;
		num_nn = 2;
		Plan.plan(c_start, c_goal, runtime+load_time[map_index], PRMfile[map_index], num_nn);
		break;
	}
	case 3 : {
		//std::ofstream ft;
		//ft.open("./stats/time_wrapScenario.txt", ios::app);

		//State c_start = Cdb[42];
		//State c_start = {0.133974, 3.28699, -0.114511, -0.0914271, 0.0355165, 0.161795, -0.293665, -0.332455, -0.118274, 0.0251658, -0.375827, -1.22614, -1.38004, 0.277725, -0.198784, -0.698157, 0.296816, 2.97746, 1.50569, -0.981951};
		//State c_start = {-0.0310325, 3.34544, -0.131529, -0.124919, -0.0158914, -0.225185, -0.249251, -0.36438, -0.0465314, 0.0357443, -0.376082, -1.20598, -1.27285, 0.331411, -0.331415, -0.593757, 0.428278, 3.04315, 1.62352, -0.959364};
		State c_start = {-0.228674, -5.79388, 0.376973, -12.1839, 0.804729, 7.27214, -0.92767, -0.610358, 0.819103, 0.211475, 0.646579, 1.57925, -1.88655, 1.82996, 0.150483, -0.21126, -0.0391787, 0.292372, 0.861397, 2.65933};

		// 1
		//State c_goal = { 0, 3.5, 0, 0, 0, 0, -0.27, -0.35, -0.1, 0, -0.32, -1.17, -1.38, -0.151493, -0.308546, -0.435896, 0.187272, 2.52019, 1.60744, -1.12429};
		// 2
		//State c_goal = {0, 3.5, 0, 0, 0, 0, 0, -0.45, -0.1, 0.3, 0, -1.4, -1.5, -0.22298, -0.443116, -0.00200849, 0.436657, -0.544159, -1.54183, 1.56446};
		// 3
		State c_goal = {0, 3.5, 0, 0, 0, 0, 0, -0.8, -0.1, 0.6, 0, -1.3, -1.5, 0.0196822, -0.807394, -0.275271, 0.715919, -0.475965, -1.49528, 1.81508};


		runtime = 30;
		int N = 1;//Cdb.size();
		for (int i = 0; i < N; i++) {
			cout << "**************************************" << endl;
			cout << "Completed " << (double)i/N*100 << "%." << endl;
			cout << "**************************************" << endl;
			bool success = Plan.plan(c_start, c_goal, runtime+load_time[2], "ms6D_50_4.prm", 2);

			if (success) {
				cout << "Solution " << i << ".\n";
				break;
			}

			// Log
			//extract_from_perf_file(ft);
			//ft << endl;
		}

		std::ofstream ft;
		ft.open("./stats/stats_ms6D_20_4.txt", ios::app);
		ft << 50 << " B " << Plan.solved_bool << " " << Plan.total_runtime << endl;
		ft.close();

		//ft.close();
		break;
	}
	case 4 : {
		vector<int> prmsize = {100,500,1000,1500,2000};

		for( int j = 2; j < PRMfile.size(); j++) {
			for (int k = 1; k < 2; k++) {
				std::ofstream ft;
				ft.open("./stats/time_sameScenario_sg_30_" + std::to_string(prmsize[j]) + "_" + std::to_string(k) + ".txt", ios::app);

				State c_start = {0, 3.2, 0, 0, 0, 0, 0, -0.50,0.440898,-0,-1.57,0,0,-0.000695474163865839,-0.00798116494726853,0,-1.56222150355732,0};
				State c_goal = {0, -4, 0, -5, 0, 3,-1.4,0.419512,0.064,1.2,1.0,-2.6,-0.588374911336135,-0.277496816097365,-0.384651761569099,-2.16861949996107,-1.72941957387009,3.04121873510677};

				int N = 50;
				for (int i = 0; i < N; i++) {
					cout << "**************************************" << endl;
					cout << "Completed " << (double)i/N*100 << "%." << endl;
					cout << "**************************************" << endl;
					Plan.plan(c_start, c_goal, runtime+load_time[j], PRMfile[j], k);

					// Log
					extract_from_perf_file(ft);
					ft << endl;
				}
				ft.close();
			}

		}
		break;
	}
	case 5 : {
		Matrix C;
		load_query_points(C);
		Plan.plan(C[0], C[1], runtime+load_time[map_index], PRMfile[map_index], num_nn);
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}


