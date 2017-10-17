#include "robots_class.h"

// Constructor for the robots
Baxter::Baxter() {

	grasp_pose = true;

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

	// Joint limits
	qmin.resize(7);
	qmax.resize(7);
	qmin[0] = deg2rad(-141); // S0 - Flip for Arm 2
	qmax[0] = deg2rad(51);
	qmin[1] = deg2rad(-123); // S1
	qmax[1] = deg2rad(60);
	qmin[2] = deg2rad(-173.5); // E0 - Flip for Arm 2
	qmax[2] = deg2rad(173.5);
	qmin[3] = deg2rad(-93); // E1
	qmax[3] = deg2rad(60);
	qmin[4] = deg2rad(-175.25); // W0 - Flip for Arm 2
	qmax[4] = deg2rad(175.25);
	qmin[5] = deg2rad(-90); // W1
	qmax[5] = deg2rad(120);
	qmin[6] = deg2rad(-175.25); // W2 - Flip for Arm 2
	qmax[6] = deg2rad(175.25);

	initMatrix1(T_fk_temp, 4, 4);
	initMatrix1(T_mult_temp, 4, 4);
	initMatrix1(T_pose, 4, 4);

	//Definition of a kinematic chain & add segments to the chain
	//chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,b)))); // Base
	//chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l1x,l1y,l1z)))); // Torso
	/*chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.0)))); // Link 1
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l2a,0.0,l2b)))); //
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(1,0,0,0,0,1,0,-1,0)))); // ^Link 2
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l3,0.0,0.0)))); //
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Rotation(1,0,0,0,0,-1,0,1,0)))); //
	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // ^Link 3
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l4a,0.0,l4b)))); //
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(1,0,0,0,0,-1,0,1,0)))); // ^Link 4
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l5,0.0,0)))); //
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Rotation(1,0,0,0,0,1,0,-1,0)))); //
	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // ^Link 5
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l6a,0.0,l6b)))); //
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(0,0,-1,0,1,0,1,0,0)))); //
	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(1,0,0,0,0,-1,0,1,0)))); // ^Link 6
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l7,0.0,0.0)))); //
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Rotation(1,0,0,0,0,-1,0,1,0)))); //
	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // ^Link 7
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,lEE)))); // EE

	if (grasp_pose)
		chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,-1,0,1,0,1,0,0)))); // EE - grasp continuous to the arm
	 */
	// Arm 1
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l2a,0.0,l2b))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4b,0.0,-l4a))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation(0,0,1,0,1,0,-1,0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(l5,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l6b,0.0,-l6a))));
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l7,0.0,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lEE,0.0,0.0))));
	if (grasp_pose)
		chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(0,0,1,0,1,0,-1,0,0)))); // EE - grasp non-continuous to the arm

	// Create joint array
	nj = chain.getNrOfJoints();
	jointpositions = JntArray(nj);
}

// -----FK-------


void Baxter::FKsolve_rob(State q, int robot_num) {

	double L1y;
	L1y = l1y;
	if (robot_num == 2)
		L1y = -L1y;
	T_pose = {{1,0,0,l1x},{0,1,0,L1y},{0,0,1,b+l1z},{0,0,0,1}};

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	for (int i = 0; i < nj; i++)
		jointpositions(i) = q[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions,cartposFK);

	for (int i = 0; i < 3; i++) {
		T_fk_temp[i][3] = cartposFK.p(i);
		for (int j = 0; j < 3; j++)
			T_fk_temp[i][j] = cartposFK.M(i,j);
	}
	T_fk_temp[3][0] = T_fk_temp[3][1] = T_fk_temp[3][2] = 0;
	T_fk_temp[3][3] = 1;

	T_fk_temp = MatricesMult(T_pose, T_fk_temp);

	if (robot_num == 1) {
		T_fk_solution_1 = T_fk_temp;
	}
	else {
		T_fk_solution_2 = T_fk_temp;
	}
}

Matrix Baxter::get_FK_solution_T1() {

	//printMatrix(T_fk_solution_1);

	return T_fk_solution_1;
}

Matrix Baxter::get_FK_solution_T2() {

	//printMatrix(T_fk_solution_2);

	return T_fk_solution_2;
}

// -------IK----------

bool Baxter::IKsolve_rob(Matrix T, State q_init, int robot_num) {
	IK_counter++;
	clock_t begin = clock();

	State q(nj);

	double L1y;
	L1y = l1y;
	if (robot_num == 2)
		L1y = -L1y;

	//T = T_pose ^-1 * T;
	Matrix T_pose_inv = {{1,0,0,-l1x}, {0,1,0,-L1y}, {0,0,1,-b-l1z}, {0,0,0,1}};
	T = MatricesMult(T_pose_inv, T);

	for (int i = 0; i < 3; i++) {
		cartposIK.p(i) = T[i][3];
		for (int j = 0; j < 3; j++)
			cartposIK.M(i,j) = T[i][j];
	}

	// KDL
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain); 	// Create solver based on kinematic chain
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,1000,1e-5);//Maximum 1000 iterations, stop at accuracy 1e-6

	//Creation of jntarrays:
	JntArray qKDL(nj);
	JntArray qInit(nj);

	for (int i = 0; i < nj; i++)
		qInit(i) = q_init[i];

	//Set destination frame
	KDL::Frame F_dest = cartposIK;//Frame(Vector(1.0, 1.0, 0.0));
	int ret = iksolver.CartToJnt(qInit, F_dest, qKDL);

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	if (ret >= 0) {
		for (int i = 0; i < 7; i++) {
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);
			q[i] = fmod(q[i], 2*PI);
			if (q[i]>PI) q[i]-=2*PI;
			if (q[i]<-PI) q[i]+=2*PI;
		}

		if (check_angle_limits(q, robot_num)) {
			if (robot_num == 1)
				q_IK_solution_1 = q;
			else
				q_IK_solution_2 = q;
			return true;
		}
	}

	return false;

}

bool Baxter::check_angle_limits(State q, int robot_num) {

	if (robot_num==1) {
		if (q[0] < qmin[0] || q[0] > qmax[0])
			return false;
	}
	else {
		if (q[0] < -qmax[0] || q[0] > -qmin[0])
			return false;
	}

	for (int i = 1; i < q.size(); i++)
		if (q[i] < qmin[i] || q[i] > qmax[i])
			return false;

	return true;
}

State Baxter::get_IK_solution_q1() {
	return q_IK_solution_1;
}

State Baxter::get_IK_solution_q2() {
	return q_IK_solution_2;
}

bool Baxter::calc_specific_IK_solution_R1(Matrix T_end, State q_active, State q_init) {
	FKsolve_rob(q_active, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), T_end); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	if (!IKsolve_rob(T2, q_init, 2))
		return false;
	return true;
}

bool Baxter::calc_specific_IK_solution_R2(Matrix T_end, State q_active, State q_init) {
	Matrix Tinv = T_end;
	InvertMatrix(T_end, Tinv); // Invert matrix
	FKsolve_rob(q_active, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	if (!IKsolve_rob(T1, q_init, 1))
		return false;
	return true;
}

//------------------------

// Misc
void Baxter::initMatrix1(Matrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

void Baxter::initVector1(State &V, int n) {
	V.resize(n);
}

double Baxter::deg2rad(double deg) {
	return deg * PI / 180.0;
}

void Baxter::printMatrix1(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void Baxter::printVector1(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}

Matrix Baxter::MatricesMult(Matrix M1, Matrix M2) {
	clearMatrix(T_mult_temp);
	for(unsigned i = 0; i < 4; ++i)
		for(unsigned j = 0; j < 4; ++j)
			for(int k = 0; k < 4; ++k)
			{
				T_mult_temp[i][j] += M1[i][k] * M2[k][j];
			}
	return T_mult_temp;
}

void Baxter::clearMatrix(Matrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

State Baxter::generate_random_arm_configuration(int robot_num) {
	State q(nj);

	if (robot_num == 1) {
		for (int i = 0; i < nj; i++) {
			double r = rand() / double(RAND_MAX); // Random fraction [0,1]
			q[i] = r * (qmax[i]-qmin[i]) + qmin[i];
		}
	}
	else {
		for (int i = 0; i < nj; i++) {
			double r = rand() / double(RAND_MAX); // Random fraction [0,1]
			q[i] = r * (-qmin[i]-(-qmax[i])) - qmax[i];
		}
	}

	return q;
}

bool Baxter::InvertMatrix(Matrix M, Matrix &Minv) {
	State m(16),  inv(16);

	int k = 0;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++, k++)
			m[k] = M[j][i];

	double det;
	int i;

	inv[0] = m[5]  * m[10] * m[15] -
			m[5]  * m[11] * m[14] -
			m[9]  * m[6]  * m[15] +
			m[9]  * m[7]  * m[14] +
			m[13] * m[6]  * m[11] -
			m[13] * m[7]  * m[10];

	inv[4] = -m[4]  * m[10] * m[15] +
			m[4]  * m[11] * m[14] +
			m[8]  * m[6]  * m[15] -
			m[8]  * m[7]  * m[14] -
			m[12] * m[6]  * m[11] +
			m[12] * m[7]  * m[10];

	inv[8] = m[4]  * m[9] * m[15] -
			m[4]  * m[11] * m[13] -
			m[8]  * m[5] * m[15] +
			m[8]  * m[7] * m[13] +
			m[12] * m[5] * m[11] -
			m[12] * m[7] * m[9];

	inv[12] = -m[4]  * m[9] * m[14] +
			m[4]  * m[10] * m[13] +
			m[8]  * m[5] * m[14] -
			m[8]  * m[6] * m[13] -
			m[12] * m[5] * m[10] +
			m[12] * m[6] * m[9];

	inv[1] = -m[1]  * m[10] * m[15] +
			m[1]  * m[11] * m[14] +
			m[9]  * m[2] * m[15] -
			m[9]  * m[3] * m[14] -
			m[13] * m[2] * m[11] +
			m[13] * m[3] * m[10];

	inv[5] = m[0]  * m[10] * m[15] -
			m[0]  * m[11] * m[14] -
			m[8]  * m[2] * m[15] +
			m[8]  * m[3] * m[14] +
			m[12] * m[2] * m[11] -
			m[12] * m[3] * m[10];

	inv[9] = -m[0]  * m[9] * m[15] +
			m[0]  * m[11] * m[13] +
			m[8]  * m[1] * m[15] -
			m[8]  * m[3] * m[13] -
			m[12] * m[1] * m[11] +
			m[12] * m[3] * m[9];

	inv[13] = m[0]  * m[9] * m[14] -
			m[0]  * m[10] * m[13] -
			m[8]  * m[1] * m[14] +
			m[8]  * m[2] * m[13] +
			m[12] * m[1] * m[10] -
			m[12] * m[2] * m[9];

	inv[2] = m[1]  * m[6] * m[15] -
			m[1]  * m[7] * m[14] -
			m[5]  * m[2] * m[15] +
			m[5]  * m[3] * m[14] +
			m[13] * m[2] * m[7] -
			m[13] * m[3] * m[6];

	inv[6] = -m[0]  * m[6] * m[15] +
			m[0]  * m[7] * m[14] +
			m[4]  * m[2] * m[15] -
			m[4]  * m[3] * m[14] -
			m[12] * m[2] * m[7] +
			m[12] * m[3] * m[6];

	inv[10] = m[0]  * m[5] * m[15] -
			m[0]  * m[7] * m[13] -
			m[4]  * m[1] * m[15] +
			m[4]  * m[3] * m[13] +
			m[12] * m[1] * m[7] -
			m[12] * m[3] * m[5];

	inv[14] = -m[0]  * m[5] * m[14] +
			m[0]  * m[6] * m[13] +
			m[4]  * m[1] * m[14] -
			m[4]  * m[2] * m[13] -
			m[12] * m[1] * m[6] +
			m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
			m[1] * m[7] * m[10] +
			m[5] * m[2] * m[11] -
			m[5] * m[3] * m[10] -
			m[9] * m[2] * m[7] +
			m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
			m[0] * m[7] * m[10] -
			m[4] * m[2] * m[11] +
			m[4] * m[3] * m[10] +
			m[8] * m[2] * m[7] -
			m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
			m[0] * m[7] * m[9] +
			m[4] * m[1] * m[11] -
			m[4] * m[3] * m[9] -
			m[8] * m[1] * m[7] +
			m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
			m[0] * m[6] * m[9] -
			m[4] * m[1] * m[10] +
			m[4] * m[2] * m[9] +
			m[8] * m[1] * m[6] -
			m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
		return false;

	det = 1.0 / det;

	k = 0;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++, k++)
			Minv[j][i] = inv[k] * det;

	return true;
}

/*
int main() {

	cout << "KDL for Baxter" << endl;

	Baxter K;

	//State q = {-PI/4,PI/2,PI/6,-PI/5,PI/3,1,-1.2};
	State q = {0.272898, -0.660043, 0.571277, 0.231013, 0.100897, -0.0446562, -2.57338};
	K.printVector1(q);

	//K.FKsolve_rob({0,0,0,0,0,0,0},1);
	K.FKsolve_rob(q,1);
	Matrix T1 = K.get_FK_solution_T1();
	K.printMatrix1(T1);

	//K.FKsolve_rob(q,2);
	//Matrix T2 = K.get_FK_solution_T2();
	K.printMatrix1(T2);
	q = {0,0,0,0,0,0,0};
	cout << K.IKsolve_rob(T1, q, 1) << endl;
	State qIK = K.get_IK_solution_q1();
	K.printVector1(qIK);
	K.FKsolve_rob(qIK,1);
	K.printMatrix1(K.get_FK_solution_T1());

	//cout << K.check_angle_limits(q) << endl;
	//cout << K.check_angle_limits(qIK) << endl;

	//K.printVector(K.generate_random_arm_configuration());
	//K.printVector(K.generate_random_arm_configuration());

	return 0;
}
 */
