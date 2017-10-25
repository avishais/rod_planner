#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "MatVec.h"
#include <iostream>
#include <string>
#include <vector>

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > State;

class collisionDetection
{
public:
	collisionDetection();

	void load_models();
	int collision_state(Matrix M, State q);
	PQP_Model base, link1, link2, link3, link4, link5, link6, link7;
	PQP_Model base2, link12, link22, link32, link42, link52, link62, link72;
	PQP_Model rod_EE1, rod_EE2, EE, EE2, rod, table, ped, route, obs;

	// Performance parameters
	int collisionCheck_counter;
	double collisionCheck_time;
	int get_collisionCheck_counter() {
		return collisionCheck_counter;
	}
	double get_collisionCheck_time() {
		return collisionCheck_time;
	}

	bool grasp_pose; // false - rod is grasped such that it is continuous to the arm, true - rod is grasped perpendicular to the gripper plane
	bool withObs = true;
};

#endif
