#include "kdl_class.h"

// Constructor for the robots
kdl::kdl(double D) {
	b = 0;
	l1x= 0.024645e3+0.055695e3-0.02e3;
	l1y=-0.25e3;
	l1z=0.118588e3+0.011038e3;
	l2a=0.073e3;
	l2b=0.245e3;
	l3=0.102e3;
	l4a=0.069e3;
	l4b=0.26242e3-0.015e3;
	l5=0.1e3;
	l6a=0.01e3;
	l6b=0.2707e3;
	l7=0.16e3;
	lEE=0.15e3;

	L = 1;

	// Joint limits
	qmin.resize(7);
	qmax.resize(7);
	qmin[0] = deg2rad(-141); // S0 - Flip for Arm 2
	qmax[0] = deg2rad(51);
	qmin[1] = deg2rad(-123); // S1
	qmax[1] = deg2rad(60);
	qmin[2] = deg2rad(-173.5); // E0 - Flip for Arm 2
	qmax[2] = deg2rad(173.5);
	qmin[3] = deg2rad(-3); // E1
	qmax[3] = deg2rad(150);
	qmin[4] = deg2rad(-175.25); // W0 - Flip for Arm 2
	qmax[4] = deg2rad(175.25);
	qmin[5] = deg2rad(-90); // W1
	qmax[5] = deg2rad(120);
	qmin[6] = deg2rad(-175.25); // W2 - Flip for Arm 2
	qmax[6] = deg2rad(175.25);

	initMatrix(T_fk, 4, 4);

	initMatrix(T_pose, 4, 4);
	T_pose = {{-1, 0, 0, 0}, {0, -1, 0, D}, {0, 0, 1, 0}, {0, 0, 0, 1}};

	// Create joint array
	unsigned int nj = 14; // Number of joints for two ABB arms
	jointpositions = JntArray(nj);
	initVector(q_solution, nj);

	grasp_pose = false;

	cout << "Initiated chain with " << nj << " joints.\n";
}

// ----- Descend -------

bool kdl::GD(State q_init_flip, Matrix Q) {

	bool valid = true;

	// Flip robot two vector
	State q_init(q_init_flip.size());
	for (int i = 0; i < 7; i++)
		q_init[i] = q_init_flip[i];
	for (int i = 13, j = 7; i >= 7; i--,j++)
		q_init[j] = q_init_flip[i];
	q_init[13] = -q_init[13];

	State q(14);

	IK_counter++;
	clock_t begin = clock();

	// Initiate chain with given rod configuration
	KDL::Chain chain;
	// Arm 1
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l2a,0.0,l2b))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4b,0.0,-l4a))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(0,0,1,0,1,0,-1,0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l5,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l6b,0.0,-l6a))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l7,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lEE,0.0,0.0))));
	//if (grasp_pose)
	//	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // EE - grasp non-continuous to the arm
	// Rod
	Rotation r( Q[0][0],Q[0][1],Q[0][2], Q[1][0],Q[1][1],Q[1][2], Q[2][0],Q[2][1],Q[2][2] );
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(Q[0][3],Q[1][3],Q[2][3])))); // Rod translation
	chain.addSegment(Segment(Joint(Joint::None),Frame(r))); // Rod rotation
	// Arm 2
	//if (grasp_pose)
	//	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // EE - grasp non-continuous to the arm
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(lEE,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l7,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l6b,0.0,l6a))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l5,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(0,0,1,0,1,0,-1,0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l4b,0.0,l4a))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l3,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l2a,0.0,-l2b))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0.0,0))));

	for (int i = 0; i < 3; i++) {
		cartposIK.p(i) = T_pose[i][3];
		for (int j = 0; j < 3; j++)
			cartposIK.M(i,j) = T_pose[i][j];
	}

	// KDL
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain); 	// Create solver based on kinematic chain
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,10000,1e-5);//Maximum 10000 iterations, stop at accuracy 1e-5

	//Creation of jntarrays:
	JntArray qKDL(chain.getNrOfJoints());
	JntArray qInit(chain.getNrOfJoints());

	for (int i = 0; i < chain.getNrOfJoints(); i++)
		qInit(i) = q_init[i];

	//Set destination frame
	KDL::Frame F_dest = cartposIK;//Frame(Vector(1.0, 1.0, 0.0));
	int ret = iksolver.CartToJnt(qInit, F_dest, qKDL);

	bool result = false;
	if (ret >= 0) {

		for (int i = 0; i < 14; i++)
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);

		for (int i = 0; i < q.size(); i++) {
			q[i] = fmod(q[i], 2*PI_);
			if (q[i]>PI_)
				q[i] -= 2*PI_;
			if (q[i]<-PI_)
				q[i] += 2*PI_;
		}

		for (int i = 0; i < 7; i++)
			q_solution[i] = q[i];
		for (int i = 13, j = 7; i >= 7; i--,j++)
			q_solution[j] = q[i];
		q_solution[7] = -q_solution[7];

		if (include_joint_limits && !check_angle_limits(q_solution))
			result = false;
		else
			result = true;
	}

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return result;
}

bool kdl::check_angle_limits(State q) {

	// Arm 1
	if (q[0] < qmin[0] || q[0] > qmax[0])
		return false;
	for (int i = 1; i < 7; i++)
		if (q[i] < qmin[i] || q[i] > qmax[i])
			return false;

	// Arm 2
	if (q[7] < qmin[0] || q[7] > qmax[0])
		return false;
	for (int i = 8; i < q.size(); i++)
		if (q[i] < qmin[i-7] || q[i] > qmax[i-7])
			return false;
	return true;
}

State kdl::get_GD_result() {
	return q_solution;
}

// -----FK-------

// This is only for validation. There is no use for this function in terms of closed chain kinematics
void kdl::FK(State q, Matrix Q) {

	// Flip robot two vector
	State q_flip(q.size());
	for (int i = 0; i < 7; i++)
		q_flip[i] = q[i];
	for (int i = 13, j = 7; i >= 7; i--,j++)
		q_flip[j] = q[i];
	q_flip[13] = -q_flip[13];
	q = q_flip;
	printVector(q);

	// Initiate chain with given rod configuration
	KDL::Chain chain;
	// Arm 1
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l2a,0.0,l2b))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4b,0.0,-l4a))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(0,0,1,0,1,0,-1,0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l5,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l6b,0.0,-l6a))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l7,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lEE,0.0,0.0))));
	//if (grasp_pose)
	//	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // EE - grasp non-continuous to the arm
	// Rod
	Rotation r( Q[0][0],Q[0][1],Q[0][2], Q[1][0],Q[1][1],Q[1][2], Q[2][0],Q[2][1],Q[2][2] );
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(Q[0][3],Q[1][3],Q[2][3])))); // Rod translation
	chain.addSegment(Segment(Joint(Joint::None),Frame(r))); // Rod rotation
	// Arm 2
	//if (grasp_pose)
	//	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // EE - grasp non-continuous to the arm
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(lEE,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l7,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l6b,0.0,l6a))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l5,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(0,0,1,0,1,0,-1,0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l4b,0.0,l4a))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l3,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l2a,0.0,-l2b))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0.0,0))));

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	for (int i = 0; i < q.size(); i++)
		jointpositions(i) = q[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions, cartposFK);

	for (int i = 0; i < 3; i++) {
		T_fk[i][3] = fabs(cartposFK.p(i)) < 1e-4 ? 0 : cartposFK.p(i);
		for (int j = 0; j < 3; j++)
			T_fk[i][j] = fabs(cartposFK.M(i,j)) < 1e-4 ? 0 : cartposFK.M(i,j);
	}
	T_fk[3][0] = T_fk[3][1] = T_fk[3][2] = 0;
	T_fk[3][3] = 1;
}

Matrix kdl::get_FK_solution() {

	//printMatrix(T_fk_solution_1);

	return T_fk;
}

//------------------------

// Misc
void kdl::initMatrix(Matrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

void kdl::initVector(State &V, int n) {
	V.resize(n);
}

double kdl::deg2rad(double deg) {
	return deg * PI_ / 180.0;
}

void kdl::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void kdl::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}


void kdl::clearMatrix(Matrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

void kdl::log_q(State q) {
	std::ofstream myfile;
	myfile.open("./paths/path.txt");

	myfile << 1 << endl;

	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;

	myfile.close();
}

State kdl::rand_q(int nj) {

	State q(nj);

	for (int i = 0; i < 7; i++) {
		q[i] = (double)rand()/RAND_MAX * (qmax[i]-qmin[i]) - qmin[i];
		q[i+7] = (double)rand()/RAND_MAX * (qmax[i]-qmin[i]) - qmin[i];
	}

	return q;
}
