#include "collisionDetection.h"

#define CADLINK "/home/avishai/Downloads/omplapp/ompl/Workspace/precomputation/Baxter/simulator/"

collisionDetection::collisionDetection() {

	// load the models
	load_models();

	grasp_pose = true;

}

void pm(PQP_REAL M[][3], std::string str) {
	std::cout << str << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (fabs(M[i][j])>1e-4)
				std::cout << M[i][j] << " ";
			else
				std::cout << 0 << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
}

void pv(PQP_REAL V[3], std::string str) {
	std::cout << str << std::endl;

	for (int i = 0; i < 3; i++)
		std::cout << V[i] << " ";
	std::cout << std::endl;
}

int collisionDetection::collision_state(Matrix M, State q)
// Returns 0 if no collision
{
	collisionCheck_counter++;
	clock_t begin = clock();

	double rot1 = q[0];
	double rot2 = q[1];
	double rot3 = q[2];
	double rot4 = q[3];
	double rot5 = q[4];
	double rot6 = q[5];
	double rot7 = q[6];
	double rot12 = q[7];
	double rot22 = q[8];
	double rot32 = q[9];
	double rot42 = q[10];
	double rot52 = q[11];
	double rot62 = q[12];
	double rot72 = q[13];

	collisionDetection::rod.BeginModel();

	for (int i=0; i<M.size()/3; i++){  //for each point, want for every 3 points
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = M[3*i][0]; p1[1] = M[3*i][1]; p1[2] = M[3*i][2];  //P1
		p2[0] = M[3*i+1][0]; p2[1] = M[3*i+1][1]; p2[2] = M[3*i+1][2];  //P2
		p3[0] = M[3*i+2][0]; p3[1] = M[3*i+2][1]; p3[2] = M[3*i+2][2];  //P3
		collisionDetection::rod.AddTri(p1,p2,p3,i);
	}

	collisionDetection::rod.EndModel();

	collisionDetection::rod_EE1.BeginModel();

	int EE_buffer = 7;

	for (int i=EE_buffer; i<(M.size()/3); i++){  //for each point, not first 21 points
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = M[3*i][0]; p1[1] = M[3*i][1]; p1[2] = M[3*i][2];  //P1
		p2[0] = M[3*i+1][0]; p2[1] = M[3*i+1][1]; p2[2] = M[3*i+1][2];  //P2
		p3[0] = M[3*i+2][0]; p3[1] = M[3*i+2][1]; p3[2] = M[3*i+2][2];  //P3
		collisionDetection::rod_EE1.AddTri(p1,p2,p3,i);
	}

	collisionDetection::rod_EE1.EndModel();

	collisionDetection::rod_EE2.BeginModel();

	for (int i=0; i<(M.size()/3-EE_buffer*3); i++){  //for each point, not last 21 points
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = M[3*i][0]; p1[1] = M[3*i][1]; p1[2] = M[3*i][2];  //P1
		p2[0] = M[3*i+1][0]; p2[1] = M[3*i+1][1]; p2[2] = M[3*i+1][2];  //P2
		p3[0] = M[3*i+2][0]; p3[1] = M[3*i+2][1]; p3[2] = M[3*i+2][2];  //P3
		collisionDetection::rod_EE2.AddTri(p1,p2,p3,i);
	}

	collisionDetection::rod_EE2.EndModel();


	// make items for transformations
	PQP_REAL R0[3][3],R1[3][3],R2[3][3],T0[3],T1[3],T2[3];
	PQP_REAL R3[3][3],R4[3][3],R5[3][3],T3[3],T4[3],T5[5];
	PQP_REAL R6[3][3],R7[3][3],T6[3],T7[3],T2_t[3],Ti[3];
	PQP_REAL R8[3][3],T8[3][3],TP[3],MP[3][3],REE[3][3],REE2[3][3];

	PQP_REAL M0[3][3],M1[3][3],M2[3][3],M3[3][3],M4[3][3],Trod[3];
	PQP_REAL M5[3][3],M6[3][3],M7[3][3],M8[3][3],TEE[3],TEE2[3];

	PQP_REAL R02[3][3],R12[3][3],R22[3][3],T02[3],T12[3],T22[3];
	PQP_REAL R32[3][3],R42[3][3],R52[3][3],T32[3],T42[3],T52[5];
	PQP_REAL R62[3][3],R72[3][3],T62[3],T72[3],T2_t2[3];
	PQP_REAL R82[3][3],T82[3][3],RInt[3][3],MInt[3][3],Mrod[3][3],Rrod[3][3], Rrod_v[3][3];

	PQP_REAL M02[3][3],M12[3][3],M22[3][3],M32[3][3],M42[3][3];
	PQP_REAL M52[3][3],M62[3][3],M72[3][3],M82[3][3];

	// rotation matrix
	MRotZ(M0,0);     //base rotate Z
	MxM(R0,M0,M0);

	MRotZ(M1,rot1);  //link 1 rotate Z
	MxM(R1,R0,M1);

	MRotY(M2,rot2);  //link 2 rotate Z
	MxM(RInt,R1,M2);
	MRotX(MInt,-3.1415926/2);
	MxM(R2,RInt,MInt);

	MRotX(M3,rot3+3.1415926/2);  //link 3 rotate Z
	MxM(RInt,R2,M3);
	MRotY(MInt,3.1415926/2);
	MxM(R3,RInt,MInt);

	MRotY(M4,rot4);  //link 4 rotate Z
	MxM(RInt,R3,M4);
	MRotX(MInt,3.1415926/2);
	MxM(R4,RInt,MInt);

	MRotX(M5,rot5-3.1415926/2);  //link 5 rotate Z
	MxM(RInt,R4,M5);
	MRotY(MInt,3.1415926/2);
	MxM(R5,RInt,MInt);

	MRotY(M6,rot6-3.1415926/2);  //link 6 rotate Z
	MxM(RInt,R5,M6);
	MRotX(MInt,3.1415926/2);
	MxM(R6,RInt,MInt);

	MRotX(M7,rot7+3.1415926/2);  // link 7 rotate Z
	MxM(RInt,R6,M7);
	MRotY(MInt,3.1415926/2);
	MxM(R7,RInt,MInt);

	MxM(REE,R7,M0);

	//define kinematics

	T0[0] =  0;
	T0[1] =  0;
	T0[2] =  0;

	MxV(T0,R0,T0);

	TP[0] = 0;
	TP[1] = 0;
	TP[2] = -850;

	T1[0] =  (0.024645+0.055695 - 0.02)*1000.;
	T1[1] =  ( -.25)*1000.;
	T1[2] =  (.118588+0.011038)*1000.;

	MxV(T1,R0,T1);
	VpV(T1,T0,T1);

	T2[0] =  0.073*1000.;
	T2[1] =  0.;
	T2[2] =  0.245*1000.;

	//MxV(T2,R1,T2);
	T2_t[0] = R1[0][0]*T2[0]+R1[0][1]*T2[1]+R1[0][2]*T2[2];
	T2_t[1] = R1[1][0]*T2[0]+R1[1][1]*T2[1]+R1[1][2]*T2[2];
	T2_t[2] = R1[2][0]*T2[0]+R1[2][1]*T2[1]+R1[2][2]*T2[2];

	VpV(T2_t,T1,T2_t);

	T2[0] =  0.102*1000.;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T3,R2,T2);
	VpV(T3,T2_t,T3);

	T2[0] =  (0.069)*1000.;
	T2[1] =  0;
	T2[2] =  (.26242-.015)*1000.;

	MxV(T4,R3,T2);
	VpV(T4,T4,T3);

	T2[0] =  (0.10)*1000.;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T5,R4,T2);
	VpV(T5,T4,T5);

	T2[0] =  (0.01)*1000.;
	T2[1] =  0;
	T2[2] =  (.2707)*1000.;

	MxV(T6,R5,T2);
	VpV(T6,T5,T6);

	T2[0] =  (.16)*1000.;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T7,R6,T2);
	VpV(T7,T6,T7);

	T2[0] =  0;
	T2[1] =  0;
	T2[2] =  0.05*1000.;

	MxV(TEE,R7,T2);
	VpV(TEE,T7,TEE);

	//ROBOT 2
	// rotation matrix
	MRotZ(M02,3.14159265/2);     //base rotate Z
	MxM(R02,M02,M02);

	MRotZ(M12,rot12-3.14159265);  //link 1 rotate Z
	MxM(R12,R02,M12);

	MRotY(M22,rot22);  //link 2 rotate Z
	MxM(RInt,R12,M22);
	MRotX(MInt,-3.1415926/2);
	MxM(R22,RInt,MInt);

	MRotX(M32,rot32+3.1415926/2);  //link 3 rotate Z
	MxM(RInt,R22,M32);
	MRotY(MInt,3.1415926/2);
	MxM(R32,RInt,MInt);

	MRotY(M42,rot42);  //link 4 rotate Z
	MxM(RInt,R32,M42);
	MRotX(MInt,3.1415926/2);
	MxM(R42,RInt,MInt);

	MRotX(M52,rot52-3.1415926/2);  //link 5 rotate Z
	MxM(RInt,R42,M52);
	MRotY(MInt,3.1415926/2);
	MxM(R52,RInt,MInt);

	MRotY(M62,rot62-3.1415926/2);  //link 6 rotate Z
	MxM(RInt,R52,M62);
	MRotX(MInt,3.1415926/2);
	MxM(R62,RInt,MInt);

	MRotX(M72,rot72+3.1415926/2); //link 7 rotate Z
	MxM(RInt,R62,M72);
	MRotY(MInt,3.1415926/2);
	MxM(R72,RInt,MInt);

	MxM(REE2,R72,M0);

	//define kinematics

	T02[0] =  0;
	T02[1] =  0;
	T02[2] =  0;

	//MxV(T02,R02,T02);

	T12[0] =  -(0.024645+0.055695 - 0.02)*1000;
	T12[1] =  (0 -.25)*1000;
	T12[2] =  (.118588+0.011038)*1000;

	MxV(T12,R02,T12);
	VpV(T12,T02,T12);

	T22[0] =  0.073*1000;
	T22[1] =  0;
	T22[2] =  0.245*1000;

	//MxV(T22,R12,T22);
	T2_t2[0] = R12[0][0]*T22[0]+R12[0][1]*T22[1]+R12[0][2]*T22[2];
	T2_t2[1] = R12[1][0]*T22[0]+R12[1][1]*T22[1]+R12[1][2]*T22[2];
	T2_t2[2] = R12[2][0]*T22[0]+R12[2][1]*T22[1]+R12[2][2]*T22[2];
	VpV(T2_t2,T12,T2_t2);

	T22[0] =  0.102*1000;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T32,R22,T22);
	VpV(T32,T2_t2,T32);

	T22[0] =  (0.069)*1000;
	T22[1] =  0;
	T22[2] =  (.26242-.015)*1000;

	MxV(T42,R32,T22);
	VpV(T42,T42,T32);

	T22[0] =  (0.10)*1000;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T52,R42,T22);
	VpV(T52,T42,T52);

	T22[0] =  (0.01)*1000;
	T22[1] =  0;
	T22[2] =  (.2707)*1000;

	MxV(T62,R52,T22);
	VpV(T62,T52,T62);

	Ti[0]=0;Ti[1]=0;Ti[2]=0;

	T22[0] =  (.16)*1000;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T72,R62,T22);
	VpV(T72,T62,T72);

	T22[0] =  0;
	T22[1] =  0;
	T22[2] =  0.05*1000.;

	MxV(TEE2,R72,T22);
	VpV(TEE2,T72,TEE2);

	// ----- Reposition rod at center of EE1

	T2[0] =  0;
	T2[1] =  0;//$.1*1000;
	T2[2] =  0.1*1000;

	MxV(Trod,REE,T2);
	VpV(Trod,Trod,TEE);

	double pose_angle; // true - rod is grasped such that it is continuous to the arm, false - rod is grasped perpendicular to the gripper plane
	if (!grasp_pose)
		pose_angle = -3.1415926/2; // Continuous to the arm
	else
		pose_angle = 3.1415926; // Perpendicular to the gripper plane

	MRotY(Mrod,pose_angle);
	MxM(Rrod, REE, Mrod);

	// Fix based on updated kinematics in kdl_class.cpp from 10/10/2017
	MRotX(Mrod,-3.1415926);
	MxM(Rrod_v, Rrod, Mrod);

	/*pm(REE, "REE");
	pm(Rrod, "Rrod");
	pm(Rrod_v, "Rrod_v");
	pv(Trod, "Trod");*/

	// ------------------------------------

	// perform tolerance query

	PQP_REAL tolerance = 10.0;
	PQP_ToleranceResult res[124];

	// robot 1 collision
	PQP_Tolerance(&res[0],R0,TP,&ped,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[1],R0,TP,&ped,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[2],R0,TP,&ped,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[3],R0,TP,&ped,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[4],R0,T0,&base,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[5],R0,T0,&base,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[6],R0,T0,&base,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[7],R0,T0,&base,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[8],R0,T0,&base,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[9],R1,T1,&link1,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[10],R1,T1,&link1,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[11],R1,T1,&link1,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[12],R1,T1,&link1,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[13],R2,T2_t,&link2,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[14],R2,T2_t,&link2,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[15],R2,T2_t,&link2,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[16],R3,T3,&link3,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[17],R3,T3,&link3,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[18],R3,T3,&link3,REE,TEE,&EE,tolerance);

	// robot 2 collision
	PQP_Tolerance(&res[19],R0,TP,&ped,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[20],R0,TP,&ped,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[21],R0,TP,&ped,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[22],R0,TP,&ped,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[23],R0,T0,&base,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[24],R0,T0,&base,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[25],R0,T0,&base,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[26],R0,T0,&base,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[27],R0,T0,&base,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[28],R12,T12,&link12,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[29],R12,T12,&link12,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[30],R12,T12,&link12,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[31],R12,T12,&link12,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[32],R22,T2_t2,&link22,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[33],R22,T2_t2,&link22,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[34],R22,T2_t2,&link22,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[35],R32,T32,&link32,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[36],R32,T32,&link32,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[37],R32,T32,&link32,REE2,TEE2,&EE2,tolerance);

	// inter-robot collision 
	PQP_Tolerance(&res[38],R1,T1,&link1,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[39],R1,T1,&link1,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[40],R1,T1,&link1,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[41],R1,T1,&link1,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[42],R2,T2,&link2,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[43],R2,T2,&link2,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[44],R2,T2,&link2,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[45],R12,T12,&link12,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[46],R12,T12,&link12,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[47],R12,T12,&link12,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[48],R12,T12,&link12,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[49],R22,T22,&link22,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[50],R22,T22,&link22,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[51],R22,T22,&link22,REE,TEE,&EE,tolerance);

	PQP_Tolerance(&res[52],R3,T3,&link3,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[53],R3,T3,&link3,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[54],R3,T3,&link3,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[55],R3,T3,&link3,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[56],R3,T3,&link3,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[57],R3,T3,&link3,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[58],R4,T4,&link4,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[59],R4,T4,&link4,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[60],R4,T4,&link4,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[61],R4,T4,&link4,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[62],R4,T4,&link4,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[63],R4,T4,&link4,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[64],R5,T5,&link5,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[65],R5,T5,&link5,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[66],R5,T5,&link5,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[67],R5,T5,&link5,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[68],R5,T5,&link5,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[69],R5,T5,&link5,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[70],R6,T6,&link6,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[71],R6,T6,&link6,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[72],R6,T6,&link6,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[73],R6,T6,&link6,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[74],R6,T6,&link6,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[75],R6,T6,&link6,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[76],R7,T7,&link7,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[77],R7,T7,&link7,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[78],R7,T7,&link7,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[79],R7,T7,&link7,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[80],R7,T7,&link7,R72,T72,&link72,tolerance);
	PQP_Tolerance(&res[81],R7,T7,&link7,REE2,TEE2,&EE2,tolerance);

	PQP_Tolerance(&res[82],R32,T32,&link32,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[83],R42,T42,&link42,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[84],R52,T52,&link52,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[85],R62,T62,&link62,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[86],R72,T72,&link72,REE,TEE,&EE,tolerance);

	// robot collision with rod
	PQP_Tolerance(&res[87],Rrod_v,Trod,&rod,R0,TP,&ped,tolerance);
	PQP_Tolerance(&res[88],Rrod_v,Trod,&rod,R1,T1,&link1,tolerance);
	PQP_Tolerance(&res[89],Rrod_v,Trod,&rod,R2,T2_t,&link2,tolerance);
	PQP_Tolerance(&res[90],Rrod_v,Trod,&rod,R3,T3,&link3,tolerance);
	PQP_Tolerance(&res[91],Rrod_v,Trod,&rod,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[92],Rrod_v,Trod,&rod,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[93],Rrod_v,Trod,&rod,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[94],Rrod_v,Trod,&rod,R7,T7,&link7,tolerance);
	PQP_Tolerance(&res[95],Rrod_v,Trod,&rod,R0,T0,&base,tolerance);
	PQP_Tolerance(&res[96],Rrod_v,Trod,&rod,R12,T12,&link12,tolerance);
	PQP_Tolerance(&res[97],Rrod_v,Trod,&rod,R22,T2_t2,&link22,tolerance);
	PQP_Tolerance(&res[98],Rrod_v,Trod,&rod,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[99],Rrod_v,Trod,&rod,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[100],Rrod_v,Trod,&rod,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[101],Rrod_v,Trod,&rod,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[102],Rrod_v,Trod,&rod,R72,T72,&link72,tolerance);

	// rod collisions with EE's with rod not counting buffer from parent EE
	PQP_Tolerance(&res[103],REE,TEE,&EE,REE2,TEE2,&EE2,tolerance);
	PQP_Tolerance(&res[104],Rrod_v,Trod,&rod_EE1,REE,TEE,&EE,tolerance);
	PQP_Tolerance(&res[105],Rrod_v,Trod,&rod_EE2,REE2,TEE2,&EE2,tolerance);

	int num_links2check = 105;

	//Collision with obstacles
	if(withObs) {
		PQP_REAL Ttable[3], Twall[3];
		PQP_REAL Rtable[3][3], Rwall[3][3];

		// Table
		MRotZ(Rtable,3.1415926/2);
		Ttable[0]=1000; Ttable[1]=0; Ttable[2]=20;

		PQP_Tolerance(&res[106],Rtable,Ttable,&table,REE,TEE,&EE,tolerance);
		PQP_Tolerance(&res[107],Rtable,Ttable,&table,R5,T5,&link5,tolerance);
		PQP_Tolerance(&res[108],Rtable,Ttable,&table,R6,T6,&link6,tolerance);
		PQP_Tolerance(&res[109],Rtable,Ttable,&table,R7,T7,&link7,tolerance);
		PQP_Tolerance(&res[110],Rtable,Ttable,&table,REE2,TEE2,&EE2,tolerance);
		PQP_Tolerance(&res[111],Rtable,Ttable,&table,R52,T52,&link52,tolerance);
		PQP_Tolerance(&res[112],Rtable,Ttable,&table,R62,T62,&link62,tolerance);
		PQP_Tolerance(&res[113],Rtable,Ttable,&table,R72,T72,&link72,tolerance);
		PQP_Tolerance(&res[114],Rtable,Ttable,&table,Rrod_v,Trod,&rod,tolerance);


		// Wall
		/*MRotZ(Rwall,0*3.1415926/180);
		//Twall[0] = 980; Twall[1] = -125; Twall[2] = 50;
		Twall[0] = 1010; Twall[1] = -20; Twall[2] = 170;

		PQP_Tolerance(&res[115],Rwall,Twall,&wall,REE,TEE,&EE,tolerance);
		PQP_Tolerance(&res[116],Rwall,Twall,&wall,R5,T5,&link5,tolerance);
		PQP_Tolerance(&res[117],Rwall,Twall,&wall,R6,T6,&link6,tolerance);
		PQP_Tolerance(&res[118],Rwall,Twall,&wall,R7,T7,&link7,tolerance);
		PQP_Tolerance(&res[119],Rwall,Twall,&wall,REE2,TEE2,&EE2,tolerance);
		PQP_Tolerance(&res[120],Rwall,Twall,&wall,R52,T52,&link52,tolerance);
		PQP_Tolerance(&res[121],Rwall,Twall,&wall,R62,T62,&link62,tolerance);
		PQP_Tolerance(&res[122],Rwall,Twall,&wall,R72,T72,&link72,tolerance);
		PQP_Tolerance(&res[123],Rwall,Twall,&wall,Rrod_v,Trod,&rod,tolerance);*/


		// Obs1
		/*MRotZ(Rwall,0);
		//Twall[0] = 980; Twall[1] = -125; Twall[2] = 50;
		Twall[0] = 900; Twall[1] = -175; Twall[2] = -100;

		PQP_Tolerance(&res[115],Rwall,Twall,&obs,REE,TEE,&EE,tolerance);
		PQP_Tolerance(&res[116],Rwall,Twall,&obs,R5,T5,&link5,tolerance);
		PQP_Tolerance(&res[117],Rwall,Twall,&obs,R6,T6,&link6,tolerance);
		PQP_Tolerance(&res[118],Rwall,Twall,&obs,R7,T7,&link7,tolerance);
		PQP_Tolerance(&res[119],Rwall,Twall,&obs,REE2,TEE2,&EE2,tolerance);
		PQP_Tolerance(&res[120],Rwall,Twall,&obs,R52,T52,&link52,tolerance);
		PQP_Tolerance(&res[121],Rwall,Twall,&obs,R62,T62,&link62,tolerance);
		PQP_Tolerance(&res[122],Rwall,Twall,&obs,R72,T72,&link72,tolerance);
		PQP_Tolerance(&res[123],Rwall,Twall,&obs,Rrod,Trod,&rod,tolerance);*/

		num_links2check = 114;
	}

	clock_t end = clock();
	collisionCheck_time += double(end - begin) / CLOCKS_PER_SEC;

	// if any parts in collision
	for (int i = 0; i <= num_links2check; ++i) {
		if (res[i].CloserThanTolerance() == 1){
			//std::cout << "Collision in " << i << std::endl;
			return 1;
		}
	}

	// if rod was not correctly inputted
	if (res[38].CloserThanTolerance() != 1 && res[38].CloserThanTolerance() != 0){
		//std::cout << "Error " << std::endl;
		return 2;
	}

	// else not in collision
	return 0;
}

void collisionDetection::load_models(){
	FILE *fp;
	int i, ntris;

	// initialize the base

	fp = fopen(CADLINK "base_link.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open base_link.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::base.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::base.AddTri(p1,p2,p3,i);
	}
	collisionDetection::base.EndModel();
	fclose(fp);

	// initialize link 1

	fp = fopen(CADLINK "S0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open S0.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link1.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link1.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link1.EndModel();
	fclose(fp);

	// initialize link2

	fp = fopen(CADLINK "S1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open S1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link2.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link2.EndModel();
	fclose(fp);

	// initialize link3

	fp = fopen(CADLINK "E0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open E0.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link3.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link3.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link3.EndModel();
	fclose(fp);

	// initialize link4

	fp = fopen(CADLINK "E1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open E1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link4.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link4.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link4.EndModel();
	fclose(fp);

	// initialize link5

	fp = fopen(CADLINK "W0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W0.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link5.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link5.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link5.EndModel();
	fclose(fp);

	// initialize link6

	fp = fopen(CADLINK "W1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link6.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link6.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link6.EndModel();
	fclose(fp);

	// initialize link7

	fp = fopen(CADLINK "W2.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W2.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link7.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link7.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link7.EndModel();
	fclose(fp);

	// initialize EE

	fp = fopen(CADLINK "gripper.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open gripper.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::EE.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::EE.AddTri(p1,p2,p3,i);
	}
	collisionDetection::EE.EndModel();
	fclose(fp);

	//ROBOT 2

	// initialize link 1

	fp = fopen(CADLINK "S0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open S0.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link12.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link12.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link12.EndModel();
	fclose(fp);

	// initialize link2

	fp = fopen(CADLINK "S1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open S1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link22.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link22.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link22.EndModel();
	fclose(fp);

	// initialize link3

	fp = fopen(CADLINK "E0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open E0.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link32.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link32.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link32.EndModel();
	fclose(fp);

	// initialize link4

	fp = fopen(CADLINK "E1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open E1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link42.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link42.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link42.EndModel();
	fclose(fp);

	// initialize link5

	fp = fopen(CADLINK "W0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W0.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link52.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link52.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link52.EndModel();
	fclose(fp);

	// initialize link6

	fp = fopen(CADLINK "W1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link62.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link62.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link62.EndModel();
	fclose(fp);

	// initialize link7

	fp = fopen(CADLINK "W2.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W1.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link72.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link72.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link72.EndModel();
	fclose(fp);

	// initialize EE

	fp = fopen(CADLINK "gripper.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open W2.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::EE2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::EE2.AddTri(p1,p2,p3,i);
	}
	collisionDetection::EE2.EndModel();
	fclose(fp);

	// initialize pedestal

	fp = fopen(CADLINK "pedestal.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open pedestal.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::ped.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::ped.AddTri(p1,p2,p3,i);
	}
	collisionDetection::ped.EndModel();
	fclose(fp);

	if (withObs) {

		// Table
		fp = fopen(CADLINK "table.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open pedestal.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		collisionDetection::table.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			collisionDetection::table.AddTri(p1,p2,p3,i);
		}
		collisionDetection::table.EndModel();
		fclose(fp);


		// Wall
		fp = fopen(CADLINK "wall4s.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open pedestal.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		collisionDetection::wall.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			collisionDetection::wall.AddTri(p1,p2,p3,i);
		}
		collisionDetection::wall.EndModel();
		fclose(fp);

		// Wall
		fp = fopen(CADLINK "obs.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open obs.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		collisionDetection::obs.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			collisionDetection::obs.AddTri(p1,p2,p3,i);
		}
		collisionDetection::obs.EndModel();
		fclose(fp);

	}
}


