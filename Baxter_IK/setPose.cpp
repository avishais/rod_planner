/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Avishai Sintov */

//#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "myRRTConnect.h" // Avishai

// Debugging tool
template <class T>
void o(T a) {
	cout << a << endl;
}

ompl::geometric::RRTConnect::RRTConnect(const base::SpaceInformationPtr &si, string PRMfile, int sNg_nn) : base::Planner(si, "RRTConnect"), StateValidityChecker(si)
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	specs_.directed = true;

	maxDistance_ = 0.0;

	Planner::declareParam<double>("range", this, &RRTConnect::setRange, &RRTConnect::getRange, "0.:1.:10000.");
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);

	defaultSettings(); // Avishai

	Range = 0.5; // As tested

	prmfile = PRMfile;
	sNg_k = sNg_nn;

}

ompl::geometric::RRTConnect::~RRTConnect()
{
	freeMemory();
}

void ompl::geometric::RRTConnect::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!tStart_)
		tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	if (!tGoal_)
		tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	tStart_->setDistanceFunction(std::bind(&RRTConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2)); //activeDistance
	tGoal_->setDistanceFunction(std::bind(&RRTConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2)); //distanceFunction
}

void ompl::geometric::RRTConnect::freeMemory()
{
	std::vector<Motion*> motions;

	if (tStart_)
	{
		tStart_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}

	if (tGoal_)
	{
		tGoal_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void ompl::geometric::RRTConnect::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (tStart_)
		tStart_->clear();
	if (tGoal_)
		tGoal_->clear();
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
}


ompl::base::PlannerStatus ompl::geometric::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
	// -------- Temp --------

	const base::State *st = pis_.nextGoal();
	State a(6), q1(Nj), q2(Nj);
	//printStateVector(st);

	// ** Start
	//State C = {0, 0, 5, -10, 8, 0, 0.305935, -1.03039, -0.727934, 0.319778, 0.458636, 0.883846, 0.34995, 0.702649, -1.077, -0.337511, 0.296043, -2.92456, -0.816651, 0.358031};

	// ** Goal
	State C = {0.471018, -0.63155, -0.205713, 5.50456, -3.37723, -23.831, -1.36857, -0.686826, 1.80753, 0.0183914, 0.0374246, 1.22959, -1.1895, 1.27828, -0.765959, -1.46411, 0.51644, 1.37573, -1.24053, -0.693005};

	for (int i = 0; i < 6; i++)
		a[i] = C[i];
	for (int i = 0; i < 7; i++) {
		q1[i] = C[i+6];
		q2[i] = C[i+6+7];
	}

	// Close sides
	if (!isRodFeasible(a)) {
		cout << "Instable rod.\n";
		exit(1);
	}
	Matrix Q = getT(get_Points_on_Rod()-1);

	FKsolve_rob(q1, 1);
	Matrix T1 = get_FK_solution_T1();
	//T1 = {{1,0,0,T1[0][3]},{0,0,1,T1[1][3]},{0,-1,0,T1[2][3]},{0,0,0,1}};

	int c = 0, index;
	double angZ = 0;
	double step = 10;
	do {
		cout << "T1: \n"; printMatrix(T1);
		if (!IKsolve_rob(T1, q1, 1))
			cout << "Failed IK for robot 1.\n";
		q1 = get_IK_solution_q1();
		updateStateVector(st, a, q1, q2);
		close_chain(st, 0);

		retrieveStateVector(st, a, q1, q2);
		cout << "Collision check: " << collision_state(getPMatrix(), q1, q2) << endl;
		cout << "Check angles: " << (check_angle_limits(q1, 1) && check_angle_limits(q2, 2)) << endl;

		printStateVector(st);
		saveState2file(st);

		/*cout << "------------------------------------\n";
		cout << "Current position: [" << T1[0][3] << ", " << T1[1][3] << ", " << T1[2][3] << "]\n";
		cout << "pose index [1-3] (x,y,z): ";
		cin >> index;
		cout << "Current coordinate is " << T1[index-1][3] << ", enter new value: ";
		cin >> T1[index-1][3];*/

		cout << "Current pose: [" << T1[0][3] << ", " << T1[1][3] << ", " << T1[2][3] << ", " << angZ << "]\n";
		cout << "Use arrow keys to move rod.\n";
		cin >> index;
		//index = std::getchar();
		if (index == 8)
			T1[0][3] += step;
		if (index == 2)
			T1[0][3] -= step;
		if (index == 6)
			T1[1][3] += step;
		if (index == 4)
			T1[1][3] -= step;
		if (index == 7)
			T1[2][3] += step;
		if (index == 1)
			T1[2][3] -= step;
		if (index == 9)
			angZ = 0.2;
		else if (index == 3)
			angZ = -0.2;
		else angZ = 0;

		if (index == 10) {
			cout << "{";
			for (int i = 0; i < 6; i++)
				cout << a[i] << ", ";
			for (int i = 0; i < 7; i++)
				cout << q1[i] << ", ";
			for (int i = 0; i < 7; i++) {
				cout << q2[i];
				if (i < 6)
					cout << ", ";
			}
			cout << "}" << endl;
		}
		if (index == 11) {
			cout << "Insert new step value: ";
			cin >> step;
		}

		Matrix Rz = {{cos(angZ), -sin(angZ), 0, 0}, {sin(angZ), cos(angZ), 0, 0},{0,0,1,0},{0,0,0,1}};

		Q = MatricesMult(Rz, Q);
		T1 = MatricesMult(Rz, T1);

		c++;
	} while (c < 10000);

	return base::PlannerStatus::EXACT_SOLUTION;
}




void ompl::geometric::RRTConnect::saveState2file(const base::State *st) {

	State a(6), q1(7), q2(7);

	// Open a_path file
	std::ofstream myfile, afile, pfile, ai;
	myfile.open("./path/robot_paths.txt");
	afile.open("./path/afile.txt");
	pfile.open("./path/rod_path.txt");

	myfile << 1 << endl;
	pfile << 501 << endl;

	retrieveStateVector(st, a, q1, q2);

	for (int j = 0; j<6; j++) {
		afile << a[j] << " ";
	}
	for (int j = 0; j<7; j++) {
		myfile << q1[j] << " ";
	}
	for (int j = 0; j<7; j++) {
		myfile << q2[j] << " ";
	}
	myfile << endl;
	afile << endl;

	State temp;
	rod_solve(a);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;


	myfile.close();
	afile.close();
	pfile.close();
}

// ** Start
//a ={0, -4.6, 0, 0, 0, 0};
//qq1 = {0, -0.2, 0, -1, -1.7, -1.4, 0};
//qq2 = {0.4, -0.6, 0, -0.4, 0, 0, 0};
//c_start = {0, -4.6, 0, 0, 0, 0, 0, -0.1, 0, -1, -1.5, -1.4, -0.5, 1.61661, 0.494671, -1.15783, 0.499994, 0.968899, -0.318459, 2.78032};

// ** Goal
/*aa =  { 2.3, -5.5, 0, 0, 0, 0};
qq1 = {-0.8, -1.2, 0.8, 0.6, -2.6, 0.7, 0}; //-0.8 -0.8 0.8 0.7 -2.6 1 0.2
qq2 = {0, -0.294859, -1.29833, -0.5, -0.000638515, -0.485917, 2.675};
//c_goal = {2.3, -5.5, 0, 0, 0, 0, -0.9, -0.9, 0.8, 0.7, -2.6, 0.9, -0.3, -0.176201, -0.610284, -0.57566, -0.603606, 2.87572, -0.0423817, -2.10024};
 */
