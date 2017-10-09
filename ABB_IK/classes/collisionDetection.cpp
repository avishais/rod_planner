#include "collisionDetection.h"

#define CADLINK "/home/avishai/Downloads/omplapp/ompl/Workspace/precomputation/ABB/simulator/"

int collisionDetection::collision_state(Matrix M, Vector q1, Vector q2, PQP_REAL tolerance)
// Returns 0 if no collision
{
	collisionCheck_counter++;
	clock_t begin = clock();

	double rot1 = q1[0];
	double rot2 = q1[1];
	double rot3 = q1[2];
	double rot4 = q1[3];
	double rot5 = q1[4];
	double rot6 = q1[5];
	double rot12 = q2[0];
	double rot22 = q2[1];
	double rot32 = q2[2];
	double rot42 = q2[3];
	double rot52 = q2[4];
	double rot62 = q2[5];

	int rs = 7; // Rod skip buffer

	collisionDetection::rod.BeginModel();

	for (int i=0; i<M.size()/rs; i++){  //for each point, want for every 3 points
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = M[rs*i][0]; p1[1] = M[rs*i][1]; p1[2] = M[rs*i][2];  //P1
		p2[0] = M[rs*i+1][0]; p2[1] = M[rs*i+1][1]; p2[2] = M[rs*i+1][2];  //P2
		p3[0] = M[rs*i+2][0]; p3[1] = M[rs*i+2][1]; p3[2] = M[rs*i+2][2];  //P3
		collisionDetection::rod.AddTri(p1,p2,p3,i);
	}

	collisionDetection::rod.EndModel();

	collisionDetection::rod_EE1.BeginModel();

	int EE_buffer = 7;

	for (int i=EE_buffer; i<(M.size()/rs); i++){  //for each point, not first 21 points
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = M[rs*i][0]; p1[1] = M[rs*i][1]; p1[2] = M[rs*i][2];  //P1
		p2[0] = M[rs*i+1][0]; p2[1] = M[rs*i+1][1]; p2[2] = M[rs*i+1][2];  //P2
		p3[0] = M[rs*i+2][0]; p3[1] = M[rs*i+2][1]; p3[2] = M[rs*i+2][2];  //P3
		collisionDetection::rod_EE1.AddTri(p1,p2,p3,i);
	}

	collisionDetection::rod_EE1.EndModel();

	collisionDetection::rod_EE2.BeginModel();

	for (int i=0; i<(M.size()-EE_buffer)/rs; i++){  //for each point, not last 21 points
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = M[rs*i][0]; p1[1] = M[rs*i][1]; p1[2] = M[rs*i][2];  //P1
		p2[0] = M[rs*i+1][0]; p2[1] = M[rs*i+1][1]; p2[2] = M[rs*i+1][2];  //P2
		p3[0] = M[rs*i+2][0]; p3[1] = M[rs*i+2][1]; p3[2] = M[rs*i+2][2];  //P3
		collisionDetection::rod_EE2.AddTri(p1,p2,p3,i);
	}

	collisionDetection::rod_EE2.EndModel();

	// make items for transformations
	PQP_REAL R0[3][3],R1[3][3],R2[3][3],T0[3],T1[3],T2[3];
	PQP_REAL R3[3][3],R4[3][3],R5[3][3],T3[3],T4[3],T5[5];
	PQP_REAL R6[3][3],R7[3][3],T6[3],T7[3],T2_t[3];

	PQP_REAL M0[3][3],M1[3][3],M2[3][3],M3[3][3],M4[3][3];
	PQP_REAL M5[3][3],M6[3][3],M7[3][3];

	double oglm[16];

	// rotation matrix
	MRotZ(M0,0);     //base rotate Z
	MxM(R0,M0,M0);

	MRotZ(M1,rot1);  //link 1 rotate Z
	MxM(R1,R0,M1);

	MRotY(M2,rot2);  //link 2 rotate Y
	MxM(R2,R1,M2);

	MRotY(M3,rot3);  //link 3 rotate Y
	MxM(R3,R2,M3);

	MRotX(M4,rot4);  //link 4 rotate X
	MxM(R4,R3,M4);

	MRotY(M5,rot5);  //link 5 rotate Y
	MxM(R5,R4,M5);

	MRotX(M6,rot6);  //link 6 rotate X
	MxM(R6,R5,M6);

	MRotY(M7,3.1415926/2);
	MxM(R7,R6,M7);

	//define kinematics

	T0[0] =  -offsetX/2;
	T0[1] =  -offsetY/2;
	T0[2] =  -offsetZ/2;

	MxV(T0,R0,T0);

	T1[0] =  0;
	T1[1] =  0;
	T1[2] =  145*2;

	MxV(T1,R0,T1);
	VpV(T1,T0,T1);

	T2[0] =  0;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T2,R1,T2);
	VpV(T2,T1,T2);

	T2_t[0] = T2[0];
	T2_t[1] = T2[1];
	T2_t[2] = T2[2];

	T2[0] =  0;  // Was T3 in Steve's code
	T2[1] =  0;
	T2[2] =  270;

	MxV(T3,R2,T2);
	VpV(T3,T2_t,T3); // Was T2 (not T2_t) in Steve's code

	T2[0] =  134;
	T2[1] =  0;
	T2[2] =  70;

	MxV(T4,R3,T2);
	VpV(T4,T4,T3);

	T2[0] =  168;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T5,R4,T2);
	VpV(T5,T4,T5);

	T2[0] =  72-13/2;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T6,R5,T2);
	VpV(T6,T5,T6);

	T2[0] =  13/2+60;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T7,R6,T2);
	VpV(T7,T6,T7);

	// ROBOT 2

	PQP_REAL R02[3][3],R12[3][3],R22[3][3],T02[3],T12[3],T22[3];
	PQP_REAL R32[3][3],R42[3][3],R52[3][3],T32[3],T42[3],T52[5];
	PQP_REAL R62[3][3],R72[3][3],T62[3],T72[3],T2_t2[3];

	PQP_REAL M02[3][3],M12[3][3],M22[3][3],M32[3][3],M42[3][3];
	PQP_REAL M52[3][3],M62[3][3],M72[3][3],MI[3][3];
	MI[0][0]=MI[1][1]=MI[2][2] =1;

	// rotation matrix
	MRotZ(M02,3.14159265/2+offsetRot/2);     //base rotate Z
	MxM(R02,M02,M02);

	MRotZ(M12,rot12);  //link 1 rotate Z
	MxM(R12,R02,M12);

	MRotY(M22,rot22);  //link 2 rotate Y
	MxM(R22,R12,M22);

	MRotY(M32,rot32);  //link 3 rotate Y
	MxM(R32,R22,M32);

	MRotX(M42,rot42);  //link 4 rotate X
	MxM(R42,R32,M42);

	MRotY(M52,rot52);  //link 5 rotate Y
	MxM(R52,R42,M52);

	MRotX(M62,rot62);  //link 6 rotate X
	MxM(R62,R52,M62);

	MRotY(M72,3.1415826/2);
	MxM(R72,R62,M72);

	//define kinematics

	T02[0] =  offsetX/2;
	T02[1] =  offsetY/2;
	T02[2] =  offsetZ/2;

	//MxV(T02,R02,T02);

	T12[0] =  0;
	T12[1] =  0;
	T12[2] =  145*2;

	MxV(T12,R02,T12);
	VpV(T12,T02,T12);

	T22[0] =  0;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T22,R12,T22);
	VpV(T22,T12,T22);

	T2_t2[0] = T22[0];
	T2_t2[1] = T22[1];
	T2_t2[2] = T22[2];

	T22[0] =  0;
	T22[1] =  0;
	T22[2] =  270;

	MxV(T32,R22,T22);
	VpV(T32,T2_t2,T32);

	T22[0] =  134;
	T22[1] =  0;
	T22[2] =  70;

	MxV(T42,R32,T22);
	VpV(T42,T42,T32);

	T22[0] =  168;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T52,R42,T22);
	VpV(T52,T42,T52);

	T22[0] =  72-13/2;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T62,R52,T22);
	VpV(T62,T52,T62);

	T22[0] =  13/2+60;
	T22[1] =  0.0;
	T22[2] =  0;

	MxV(T72,R62,T22);
	VpV(T72,T62,T72);

	// perform tolerance query
	PQP_ToleranceResult res[98];

	// robot 1 collision
	PQP_Tolerance(&res[0],R0,T0,&base,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[1],R1,T1,&link1,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[2],R2,T2_t,&link2,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[3],R0,T0,&base,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[4],R1,T1,&link1,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[5],R2,T2_t,&link2,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[6],R0,T0,&base,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[7],R1,T1,&link1,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[8],R2,T2_t,&link2,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[9],R0,T0,&base,R3,T3,&link3,tolerance);
	PQP_Tolerance(&res[10],R0,T0,&base,R6,T7,&EE,tolerance);
	PQP_Tolerance(&res[11],R1,T1,&link1,R6,T7,&EE,tolerance);
	PQP_Tolerance(&res[12],R2,T2_t,&link2,R6,T7,&EE,tolerance);

	// robot 2 collision
	PQP_Tolerance(&res[13],R02,T02,&base2,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[14],R12,T12,&link12,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[15],R22,T2_t2,&link22,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[16],R02,T02,&base2,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[17],R12,T12,&link12,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[18],R22,T2_t2,&link22,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[19],R02,T02,&base2,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[20],R12,T12,&link12,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[21],R22,T2_t2,&link22,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[22],R02,T02,&base2,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[23],R02,T02,&base2,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[24],R12,T12,&link12,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[25],R22,T2_t2,&link22,R62,T72,&EE2,tolerance);

	// inter-robot collision  link2 and up for all robot
	PQP_Tolerance(&res[26],R3,T3,&link3,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[27],R3,T3,&link3,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[28],R3,T3,&link3,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[29],R3,T3,&link3,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[30],R4,T4,&link4,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[31],R4,T4,&link4,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[32],R4,T4,&link4,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[33],R4,T4,&link4,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[34],R5,T5,&link5,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[35],R5,T5,&link5,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[36],R5,T5,&link5,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[37],R5,T5,&link5,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[38],R6,T6,&link6,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[39],R6,T6,&link6,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[40],R6,T6,&link6,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[41],R6,T6,&link6,R62,T62,&link62,tolerance);

	//inter EE collisions
	PQP_Tolerance(&res[42],R3,T3,&link3,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[43],R4,T4,&link4,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[44],R5,T5,&link5,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[45],R6,T6,&link6,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[46],R6,T7,&EE,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[47],R6,T7,&EE,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[48],R6,T7,&EE,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[49],R6,T7,&EE,R62,T62,&link62,tolerance);

	// robot collision with rod
	PQP_Tolerance(&res[50],R6,T7,&rod,R0,T0,&base,tolerance);
	PQP_Tolerance(&res[51],R6,T7,&rod,R1,T1,&link1,tolerance);
	PQP_Tolerance(&res[52],R6,T7,&rod,R2,T2_t,&link2,tolerance);
	PQP_Tolerance(&res[53],R6,T7,&rod,R3,T3,&link3,tolerance);
	PQP_Tolerance(&res[54],R6,T7,&rod,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[55],R6,T7,&rod,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[56],R6,T7,&rod,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[57],R6,T7,&rod,R02,T02,&base2,tolerance);
	PQP_Tolerance(&res[58],R6,T7,&rod,R12,T12,&link12,tolerance);
	PQP_Tolerance(&res[59],R6,T7,&rod,R22,T2_t2,&link22,tolerance);
	PQP_Tolerance(&res[60],R6,T7,&rod,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[61],R6,T7,&rod,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[62],R6,T7,&rod,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[63],R6,T7,&rod,R62,T62,&link62,tolerance);

	// rod collisions with EE's with rod not counting buffer from parent EE
	PQP_Tolerance(&res[64],R6,T7,&rod_EE1,R6,T7,&EE,tolerance);
	PQP_Tolerance(&res[65],R6,T7,&rod_EE2,R62,T72,&EE2,tolerance);
	PQP_Tolerance(&res[66],R6,T7,&EE,R62,T72,&EE2,tolerance);

	// robot collision with ground
	PQP_Tolerance(&res[67],R0,T0,&table,R2,T2_t,&link2,tolerance);
	PQP_Tolerance(&res[68],R0,T0,&table,R3,T3,&link3,tolerance);
	PQP_Tolerance(&res[69],R0,T0,&table,R4,T4,&link4,tolerance);
	PQP_Tolerance(&res[70],R0,T0,&table,R5,T5,&link5,tolerance);
	PQP_Tolerance(&res[71],R0,T0,&table,R6,T6,&link6,tolerance);
	PQP_Tolerance(&res[72],R0,T0,&table,R6,T7,&EE,tolerance);
	PQP_Tolerance(&res[73],R0,T0,&table,R2,T2_t2,&link22,tolerance);
	PQP_Tolerance(&res[74],R0,T0,&table,R32,T32,&link32,tolerance);
	PQP_Tolerance(&res[75],R0,T0,&table,R42,T42,&link42,tolerance);
	PQP_Tolerance(&res[76],R0,T0,&table,R52,T52,&link52,tolerance);
	PQP_Tolerance(&res[77],R0,T0,&table,R62,T62,&link62,tolerance);
	PQP_Tolerance(&res[78],R0,T0,&table,R62,T72,&EE2,tolerance);

	//rod collision with ground
	PQP_Tolerance(&res[79],R0,T0,&table,R6,T7,&rod,tolerance);

	int num_links2check = 79;

	//Collision with obstacles
	if (withObs) {
		PQP_REAL Robs[3][3], Tobs1[3], Tobs2[3];

		// Obs 1
		MRotZ(Robs,0);
		//MxM(R0,Mobs,Mobs);

		Tobs1[0] =  -60;
		Tobs1[1] =  -220;
		Tobs1[2] =  0;

		Tobs2[0] =  0;
		Tobs2[1] =  400;
		Tobs2[2] =  0;

		PQP_Tolerance(&res[80],Robs,Tobs1,&obs1,R4,T4,&link4,tolerance);
		PQP_Tolerance(&res[81],Robs,Tobs1,&obs1,R5,T5,&link5,tolerance);
		PQP_Tolerance(&res[82],Robs,Tobs1,&obs1,R6,T6,&link6,tolerance);
		PQP_Tolerance(&res[83],Robs,Tobs1,&obs1,R6,T7,&EE,tolerance);
		PQP_Tolerance(&res[84],Robs,Tobs1,&obs1,R42,T42,&link42,tolerance);
		PQP_Tolerance(&res[85],Robs,Tobs1,&obs1,R52,T52,&link52,tolerance);
		PQP_Tolerance(&res[86],Robs,Tobs1,&obs1,R62,T62,&link62,tolerance);
		PQP_Tolerance(&res[87],Robs,Tobs1,&obs1,R62,T72,&EE2,tolerance);
		PQP_Tolerance(&res[88],Robs,Tobs1,&obs1,R6,T7,&rod,tolerance);

		PQP_Tolerance(&res[89],Robs,Tobs2,&obs2,R4,T4,&link4,tolerance);
		PQP_Tolerance(&res[90],Robs,Tobs2,&obs2,R5,T5,&link5,tolerance);
		PQP_Tolerance(&res[91],Robs,Tobs2,&obs2,R6,T6,&link6,tolerance);
		PQP_Tolerance(&res[92],Robs,Tobs2,&obs2,R6,T7,&EE,tolerance);
		PQP_Tolerance(&res[93],Robs,Tobs2,&obs2,R42,T42,&link42,tolerance);
		PQP_Tolerance(&res[94],Robs,Tobs2,&obs2,R52,T52,&link52,tolerance);
		PQP_Tolerance(&res[95],Robs,Tobs2,&obs2,R62,T62,&link62,tolerance);
		PQP_Tolerance(&res[96],Robs,Tobs2,&obs2,R62,T72,&EE2,tolerance);
		PQP_Tolerance(&res[97],Robs,Tobs2,&obs2,R6,T7,&rod,tolerance);

		num_links2check = 97;
	}

	clock_t end = clock();
	collisionCheck_time += double(end - begin) / CLOCKS_PER_SEC;

	// if any parts in collision
	for (int i = 0; i <= num_links2check; ++i) {
		if (res[i].CloserThanTolerance() == 1){
			//std::cout << "----------------Collision in " << i << std::endl;
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

	fp = fopen(CADLINK "Base_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Base_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link1_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link1_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link2_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link2_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link3_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link3_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link4_r2.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link4_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link5_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link5_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link6.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link6_r.tris\n"); exit(-1); }
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

	// initialize EE

	fp = fopen(CADLINK "EE_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open EE_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Base_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Base_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::base2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::base2.AddTri(p1,p2,p3,i);
	}
	collisionDetection::base2.EndModel();
	fclose(fp);

	// initialize link 1

	fp = fopen(CADLINK "Link1_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link1_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link2_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link2_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link3_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link3_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link4_r2.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link4_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link5_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link5_r.tris\n"); exit(-1); }
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

	fp = fopen(CADLINK "Link6.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link6_r.tris\n"); exit(-1); }
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

	// initialize EE

	fp = fopen(CADLINK "EE_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open EE_r.tris\n"); exit(-1); }
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

	// initialize table

	fp = fopen(CADLINK "table.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open table.tris\n"); exit(-1); }
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

	if (withObs) {

		// initialize obs1

		fp = fopen(CADLINK "obs.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open obs.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		collisionDetection::obs1.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			collisionDetection::obs1.AddTri(p1,p2,p3,i);
		}
		collisionDetection::obs1.EndModel();
		fclose(fp);

		// initialize obs2

		fp = fopen(CADLINK "obs.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open obs.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		collisionDetection::obs2.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			collisionDetection::obs2.AddTri(p1,p2,p3,i);
		}
		collisionDetection::obs2.EndModel();
		fclose(fp);
	}
}

collisionDetection::collisionDetection(double X, double Y, double Z, double Rot){

	// load the models
	load_models();
	collisionDetection::offsetX = X;
	collisionDetection::offsetY = Y;
	collisionDetection::offsetZ = Z;
	collisionDetection::offsetRot = Rot;

}
