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

bool extract_from_perf_file(ofstream &ToFile);

bool isStateValidC(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, string PRMfile, int sNg_nn, int iter_bound_num)
{
	return std::make_shared<og::RRTConnect>(si, PRMfile, sNg_nn, iter_bound_num); 
}

void plan_C::plan(Vector c_start, Vector c_goal, double runtime, string PRMfile, int sNg_nn, int iter_bound_num) {

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
	for (int i = 0; i < 6; i++) {
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_start[i]; // Access the first component of the start a-state
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i+6] = c_start[i+12];//q2[i];
	}

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i+6] = c_goal[i+12];//q2[i];
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	// pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner = allocatePlanner(si, PRMfile, sNg_nn, iter_bound_num);

	// ob::PlannerPtr planner(new og::RRTConnect(si, PRMfile, sNg_nn, iter_bound_num));
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
	// solved_bool = false;
	// clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	// clock_t end = clock();
	// total_runtime = double(end - begin) / CLOCKS_PER_SEC;
	// cout << "Runtime: " << total_runtime << endl;
	solved_bool = (bool)solved;

	// if (solved) {
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
		// std::cout << "Found solution" << std::endl;
		// solved_bool = true;
	// }
	// else {
		// std::cout << "No solutions found" << std::endl;
	// }
	// std::cout << "Out" << std::endl;
}



void load_random_nodes(Matrix &Cdb) {
	cout << "Getting random nodes." << endl;

	std::ifstream File;
	File.open("/home/avishai/Documents/workspace/rod_planner_vanilla/validity_checkers/randomnodes.txt");
	//File.open("randomnodes.txt");

	Vector c_temp(18);

	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 18; j++) {
			File >> c_temp[j];
			// cout << c_temp[j] << " ";
		}
		// cout << endl;
		Cdb.push_back(c_temp);
		i++;
		// cout << i << endl;
	}
	File.close();
	Cdb.pop_back();

}

void load_query_points(Matrix &C) {
	Vector c_temp(18);

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


double distance(Vector a, Vector b) {
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
	double runtime, map_index, num_nn, iter_bound_num;

	if (argn == 1)
		runtime = 10; // sec
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

	vector<string> PRMfile = {"ms6D_50_4.prm", "ms6D_100_4.prm"};
	vector<double> load_time = {10, 10};

	int mode = 1;
	switch (mode) {
	case 1 : {
		srand (time(NULL));

		Matrix Cdb;
		load_random_nodes(Cdb);

		cout << Cdb.size() << endl;

		std::ofstream ft;
		ft.open("timesC.txt", ios::app);

		int p1, p2;
		for (int j = 47; j < 100; j++) {
			// do {
			// 	p1 = rand() % Cdb.size();
			// 	p2 = rand() % Cdb.size();

			// } while (p1==p2);
			p1 = j;
			p2 = j + 1;

			cout << p1 << " " << p2 << endl;

			map_index = 1;
			num_nn = 3;
			runtime = 200;
			try {
				Plan.plan(Cdb[p1], Cdb[p2], runtime+load_time[map_index], PRMfile[map_index], num_nn, iter_bound_num);
			} catch (std::exception& e){
				std::cout << " a standard exception was caught, with message '" << e.what() << "'\n";
			}

			// Log
			ft << p1 << "\t" << p2 << "\t";
			bool suc = extract_from_perf_file(ft);
			ft << endl;

			if (suc) {
				cout << "Found solution for random query.\n";
			}
		}
		ft.close();
		break;
	}
	case 2 : {

		//Vector c_start = {2.17, -4.2, 2, 1.9, 9.6, -2.4, 0.0873, 0.1745,  0, 0, 0.1745, 0, 0.972532, 0.309214, 0.505241, 0.848896, -1.59516, 0.0524666};
		//Vector c_goal = {0, -4, 0, -5, 0, 3, 1.9, 0.6726, 0.1745, -1.5707, 0.7, -2.6, -0.9826, 1.1060, 0.4769, -0.5696, -1.5875, -0.9615};
		//Vector c_start = {0.5911,   -1.2856,   -5.8199 ,  -0.7626 ,   0.3748  ,  0.0214, -0.390328,0.612012,-0.281784,-2.10627,-1.01849,-2.05768,-0.00197797,0.888097,-0.686747,-0.664729,-0.661837,-1.75615 };
		//Vector c_goal = {-0.5938  , -2.3700 ,  -5.0451 ,  -0.2548  ,  0.8103 ,  -0.4187, 0.0444899,0.441007,-0.275984,-2.51569,-0.948493,-1.57105,0.0492653,0.884583,-0.607802,-0.627877,-0.618528,-1.85671};

		//Vector c_start = {0, 3.2, 0, 0, 0, 0, 0, -0.50,0.440898,-0,-1.57,0,0,-0.000695474163865839,-0.00798116494726853,0,-1.56222150355732,0};
		//Vector c_goal = {0, -4, 0, -5, 0, 3,-1.4,0.419512,0.064,1.2,1.0,-2.6,-0.588374911336135,-0.277496816097365,-0.384651761569099,-2.16861949996107,-1.72941957387009,3.04121873510677};

		// Example - with pole obstacles
		Vector c_start = {1.13317, -4.08401, 2.74606, 6.786018, 11.63367, -5.103594, -0.209439510239320, 0.122173047639603,	0.174532925199433, 1.30899693899575, 0.261799387799149, 0.698131700797732, -0.106584015572764, 1.06335198985049, 0.282882132165777, -0.115210802424076, -1.95829181139617, -1.35961844319303};
		Vector c_goal = {1.8708,-1.3245,2.944,3.7388,6.5021,-0.01924,0.4,0.3,1,-0.1,-0.57118,-0.4,-0.84385,0.73392,0.2169,0.52291,-1.1915,2.6346};
		//1.8708,-1.3245,2.944,3.7388,6.5021,-0.01924,1.2557,0.8,-0.46087,-0.74742,0.57118,1.7,-0.95019,1.4717,-0.94615,0.73362,-1.7091,2.6137};

		map_index = 0;
		num_nn = 2;
		runtime = 30;
		iter_bound_num = 1e9;
		Plan.plan(c_start, c_goal, runtime+load_time[map_index], PRMfile[map_index], num_nn, iter_bound_num);

		cout << "Back\n";

		std::ofstream ft;
		ft.open("./stats/stats_rm.txt", ios::app);
		ft << 100 << " A ";
		ft << Plan.solved_bool << " " << Plan.total_runtime << " ";
		ft.close();

		break;
	}
	case 3 : {
		std::ofstream ft;
		//ft.open("./stats/time_sameScenario_30_750_2.txt", ios::app);
		ft.open("./stats/time_sameScenario_collisionCounter7.txt", ios::app);

		Matrix Cdb;
		load_random_nodes(Cdb);

		//Vector c_start = {0, 3.2, 0, 0, 0, 0, 0, -0.50,0.440898,-0,-1.57,0,0,-0.000695474163865839,-0.00798116494726853,0,-1.56222150355732,0};
		//Vector c_goal = {0, -4, 0, -5, 0, 3,-1.4,0.419512,0.064,1.2,1.0,-2.6,-0.588374911336135,-0.277496816097365,-0.384651761569099,-2.16861949996107,-1.72941957387009,3.04121873510677};

		Vector c_start = Cdb[224];
		Vector c_goal = Cdb[307];

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
			extract_from_perf_file(ft);
			ft << endl;
		}
		ft.close();
		break;
	}
	case 4 : {
		vector<int> prmk = {2,3,4,5,6};
		int k = 2;
		iter_bound_num = 1e9;
		std::ofstream ft;

		for( int j = 2; j < 3/*PRMfile.size()*/; j++) {

			//Vector c_start = {0, 3.2, 0, 0, 0, 0, 0, -0.50,0.440898,-0,-1.57,0,0,-0.000695474163865839,-0.00798116494726853,0,-1.56222150355732,0};
			//Vector c_goal = {0, -4, 0, -5, 0, 3,-1.4,0.419512,0.064,1.2,1.0,-2.6,-0.588374911336135,-0.277496816097365,-0.384651761569099,-2.16861949996107,-1.72941957387009,3.04121873510677};

			// Example - with pole obstacles
			Vector c_start = {1.13317, -4.08401, 2.74606, 6.786018, 11.63367, -5.103594, -0.209439510239320, 0.122173047639603,	0.174532925199433, 1.30899693899575, 0.261799387799149, 0.698131700797732, -0.106584015572764, 1.06335198985049, 0.282882132165777, -0.115210802424076, -1.95829181139617, -1.35961844319303};
			Vector c_goal = {1.8708,-1.3245,2.944,3.7388,6.5021,-0.01924,0.4,0.3,1,-0.1,-0.57118,-0.4,-0.84385,0.73392,0.2169,0.52291,-1.1915,2.6346};

			int N = 100;
			for (int i = 0; i < N; i++) {
				cout << "**************************************" << endl;
				cout << "Completed " << (double)i/N*100 << "%." << endl;
				cout << "**************************************" << endl;
				Plan.plan(c_start, c_goal, runtime+load_time[j], PRMfile[j], k, iter_bound_num);

				// Log
				ft.open("./stats/time_poleScene_MS1000_new.txt", ios::app);
				ft << prmk[j] << "\t";
				extract_from_perf_file(ft);
				ft << endl;
				ft.close();
			}
		}
		break;
	}
	case 5 : {
		Matrix C;

		//map_index = 2;
		num_nn = 2;
		iter_bound_num = 1e10;

		load_query_points(C);
		Plan.plan(C[0], C[1], runtime+load_time[map_index], PRMfile[map_index], num_nn, iter_bound_num);
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}


