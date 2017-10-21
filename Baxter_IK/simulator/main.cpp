#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "MatVec.h"
#include <iostream>
#include <vector>
#include <unistd.h>

PQP_Model base, link1, link2, link3, link4, link5, link6, link7, ped;
Model *base_to_draw, *link1_to_draw, *link2_to_draw, *link3_to_draw, \
*link4_to_draw, *link5_to_draw, *link6_to_draw, *link7_to_draw, *ped_to_draw, *clamps_to_draw, *clamps_to_draw2, *table_to_draw, *obs1_to_draw, *wall_to_draw;

PQP_Model base2, link12, link22, link32, link42, link52, link62, link72, rod, EE, EE2, clmp, clmp2;
PQP_Model table, obs1, wall;
Model *link1_to_draw2, *link2_to_draw2, *link3_to_draw2, \
*link4_to_draw2, *link5_to_draw2, *link6_to_draw2, *link7_to_draw2, \
*rod_to_draw, *EE_to_draw, *EE_to_draw2;

PQP_REAL R0[3][3],R1[3][3],R2[3][3],T0[3],T1[3],T2[3];
PQP_REAL R3[3][3],R4[3][3],R5[3][3],T3[3],T4[3],T5[5];
PQP_REAL R6[3][3],R7[3][3],T6[3],T7[3],T2_t[3],Ti[3];
PQP_REAL R8[3][3],T8[3][3],TP[3],MP[3][3],REE[3][3],REE2[3][3], Rrod_v[3][3];

PQP_REAL M0[3][3],M1[3][3],M2[3][3],M3[3][3],M4[3][3],Trod[3], Trod2[3];
PQP_REAL M5[3][3],M6[3][3],M7[3][3],M8[3][3],TEE[3],TEE2[3];

int step;
bool withObs = true;
bool grasp_pose = true; // false - rod is grasped such that it is continuous to the arm, true - rod is grasped perpendicular to the gripper plane

double oglm[16];

double Xc = 650, Yc = 380;

bool step_sim = false;
int sim_velocity;

PQP_REAL R02[3][3],R12[3][3],R22[3][3],T02[3],T12[3],T22[3];
PQP_REAL R32[3][3],R42[3][3],R52[3][3],T32[3],T42[3],T52[5];
PQP_REAL R62[3][3],R72[3][3],T62[3],T72[3],T2_t2[3];
PQP_REAL R82[3][3],T82[3][3],RInt[3][3],MInt[3][3],Mrod[3][3],Rrod[3][3], Rrod2[3][3];
PQP_REAL Rclp[3][3], Rclp2[3][3], Mclp[3][3];

PQP_REAL M02[3][3],M12[3][3],M22[3][3],M32[3][3],M42[3][3];
PQP_REAL M52[3][3],M62[3][3],M72[3][3],M82[3][3];

typedef std::vector<std::vector<std::vector< double > > > Matrix_rod;
typedef std::vector<std::vector< double  > > Matrix_robo;
Matrix_rod RodStates;
Matrix_robo RoboStates;

int mode;
double beginx, beginy;
double dis = 3000.0, azim = -30.0, elev = 38.0, swi = -120;//-115;
double ddis = 700.0, dazim = 0.0, delev = 0.0;
double rot1, rot2, rot3, rot4, rot5, rot6, rot12, rot22, rot23, rot32, rot42, rot52, rot62, rot7, rot72;
int visualizeRobots = 1, visualize = 1;
double blackFrame[] = {50./255,50./255,50./255};
double redFrame[] = {175./255,0./255,0./255};

void execute_path(int);
void load_path();

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

void InitViewerWindow()
{
	GLfloat Ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat Diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat Specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat SpecularExp[] = { 50 };
	GLfloat Emission[] = { 0.1f, 0.1f, 0.1f, 1.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, SpecularExp);
	glMaterialfv(GL_FRONT, GL_EMISSION, Emission);

	glMaterialfv(GL_BACK, GL_AMBIENT, Ambient);
	glMaterialfv(GL_BACK, GL_DIFFUSE, Diffuse);
	glMaterialfv(GL_BACK, GL_SPECULAR, Specular);
	glMaterialfv(GL_BACK, GL_SHININESS, SpecularExp);
	glMaterialfv(GL_BACK, GL_EMISSION, Emission);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	glEnable(GL_COLOR_MATERIAL);

	GLfloat light_position[] = { 10000.0, 10000.0, 10000.0, 10000000.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glShadeModel(GL_FLAT);
	glClearColor(175./255, 238./255, 238./255, 0.0);//1.0, 1.0, 0.7, 0.0);  //background color

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_NORMALIZE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.004,0.004,-0.004,0.004,.01,1000000000.0);  //part view cutoff warping

	glMatrixMode(GL_MODELVIEW);
}

void KeyboardCB(unsigned char key, int x, int y) 
{
	switch(key)
	{
	case 'q':
		delete base_to_draw;
		delete link1_to_draw;
		delete link2_to_draw;
		delete link3_to_draw;
		delete link4_to_draw;
		delete link5_to_draw;
		delete link6_to_draw;
		delete link7_to_draw;
		exit(0);
	case '1': rot1 += .1; break;
	case '2': rot2 += .1; break;
	case '3': rot3 += .1; break;
	case '4': rot4 += .1; break;
	case '5': rot5 += .1; break;
	case '6': rot6 += .1; break;
	case '7': rot7 += .1; break;
	case '8': rot12 += .1; break;
	case '9': rot22 += .1; break;
	case '0': rot32 += .1; break;
	case '-': rot42 += .1; break;
	case '=': rot52 += .1; break;
	case 'o': rot62 += .1; break;
	case 'p': rot72 += .1; break;
	case 'd': Yc += 1; break;
	case 'a': Yc -= 1; break;
	case 'w': Xc -= 1; break;
	case 's': Xc += 1; break;
	case 'r':
		std::cout << "Updating path...\n";
		load_path();
		for (int i = 0; i < RoboStates.size(); i++) {
			for (int j = 0; j < RoboStates[i].size(); j++)
				std::cout << RoboStates[i][j] << " ";
			std::cout << std::endl;
		}
		int k = 0;
		rot1 = RoboStates[k][0];
		rot2 = RoboStates[k][1];
		rot3 = RoboStates[k][2];
		rot4 = RoboStates[k][3];
		rot5 = RoboStates[k][4];
		rot6 = RoboStates[k][5];
		rot7 = RoboStates[k][6];
		rot12 = RoboStates[k][7];
		rot22 = RoboStates[k][8];
		rot32 = RoboStates[k][9];
		rot42 = RoboStates[k][10];
		rot52 = RoboStates[k][11];
		rot62 = RoboStates[k][12];
		rot72 = RoboStates[k][13];
		break;
	}
	glutPostRedisplay();
}

void MouseCB(int _b, int _s, int _x, int _y)
{
	if (_s == GLUT_UP)
	{
		dis += ddis;
		azim += dazim;
		elev += delev;
		ddis = 0.0;
		dazim = 0.0;
		delev = 0.0;
		return;
	}

	if (_b == GLUT_RIGHT_BUTTON)
	{
		mode = 0;
		beginy = _y;
		return;
	}
	else
	{
		mode = 1;
		beginx = _x;
		beginy = _y;
	}
}

void MotionCB(int _x, int _y)
{
	if (mode == 0)
	{
		ddis = dis * (_y - beginy)/200.0;
	}
	else
	{
		dazim = (_x - beginx)/5.0;
		delev = (_y - beginy)/5.0;
	}

	glutPostRedisplay();
}

inline void glVertex3v(float V[3]) { glVertex3fv(V); }
inline void glVertex3v(double V[3]) { glVertex3dv(V); }

void BeginDraw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	glTranslatef(0.0, 0.0, -(dis+ddis));
	glRotated(elev+delev, 1.0, 0.0, 0.0);
	glRotated(azim+dazim, 0.0, 1.0, 0.0);
	glRotated(swi, 1.0, 1.0, 1.0);
}

void EndDraw()
{
	//glFlush();
	glutSwapBuffers();
}

void IdleCB() 
{  
	glutPostRedisplay();
}

void DisplayCB()
{
	BeginDraw();

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

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R0,T0);
		glPushMatrix();
		glMultMatrixd(oglm);
		base_to_draw->Draw();
		glPopMatrix();
	}

	TP[0] = 0;
	TP[1] = 0;
	TP[2] = -850;

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R0,TP);
		glPushMatrix();
		glMultMatrixd(oglm);
		ped_to_draw->Draw();
		glPopMatrix();
	}

	T1[0] =  (0.024645+0.055695 - 0.02)*1000.;
	T1[1] =  ( -.25)*1000.;
	T1[2] =  (.118588+0.011038)*1000.;

	MxV(T1,R0,T1);
	VpV(T1,T0,T1);

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R1,T1);
		glPushMatrix();
		glMultMatrixd(oglm);
		link1_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  0.073*1000.;
	T2[1] =  0.;
	T2[2] =  0.245*1000.;

	//MxV(T2,R1,T2);
	T2_t[0] = R1[0][0]*T2[0]+R1[0][1]*T2[1]+R1[0][2]*T2[2];
	T2_t[1] = R1[1][0]*T2[0]+R1[1][1]*T2[1]+R1[1][2]*T2[2];
	T2_t[2] = R1[2][0]*T2[0]+R1[2][1]*T2[1]+R1[2][2]*T2[2];

	VpV(T2_t,T1,T2_t);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R2,T2_t);
		glPushMatrix();
		glMultMatrixd(oglm);
		link2_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  0.102*1000.;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T3,R2,T2);
	VpV(T3,T2_t,T3);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R3,T3);
		glPushMatrix();
		glMultMatrixd(oglm);
		link3_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  (0.069)*1000.;
	T2[1] =  0;
	T2[2] =  (.26242-.015)*1000.;

	MxV(T4,R3,T2);
	VpV(T4,T4,T3);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R4,T4);
		glPushMatrix();
		glMultMatrixd(oglm);
		link4_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  (0.10)*1000.;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T5,R4,T2);
	VpV(T5,T4,T5);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R5,T5);
		glPushMatrix();
		glMultMatrixd(oglm);
		link5_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  (0.01)*1000.;
	T2[1] =  0;
	T2[2] =  (.2707)*1000.;

	MxV(T6,R5,T2);
	VpV(T6,T5,T6);

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R6,T6);
		glPushMatrix();
		glMultMatrixd(oglm);
		link6_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  (.16)*1000.;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T7,R6,T2);
	VpV(T7,T6,T7);

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R7,T7);
		glPushMatrix();
		glMultMatrixd(oglm);
		link7_to_draw->Draw();
		glPopMatrix();
	}

	T2[0] =  0;
	T2[1] =  0;
	T2[2] =  0.05*1000.;

	MxV(TEE,R7,T2);
	VpV(TEE,T7,TEE);

	if(visualizeRobots == 1){
		glColor3d(1.0,1.0,1.0);
		MVtoOGL(oglm,REE,TEE);
		glPushMatrix();
		glMultMatrixd(oglm);
		EE_to_draw->Draw();
		glPopMatrix();
	}

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

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R12,T12);
		glPushMatrix();
		glMultMatrixd(oglm);
		link1_to_draw2->Draw();
		glPopMatrix();
	}

	T22[0] =  0.073*1000;
	T22[1] =  0;
	T22[2] =  0.245*1000;

	//MxV(T22,R12,T22);
	T2_t2[0] = R12[0][0]*T22[0]+R12[0][1]*T22[1]+R12[0][2]*T22[2];
	T2_t2[1] = R12[1][0]*T22[0]+R12[1][1]*T22[1]+R12[1][2]*T22[2];
	T2_t2[2] = R12[2][0]*T22[0]+R12[2][1]*T22[1]+R12[2][2]*T22[2];
	VpV(T2_t2,T12,T2_t2);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R22,T2_t2);
		glPushMatrix();
		glMultMatrixd(oglm);
		link2_to_draw2->Draw();
		glPopMatrix();
	}

	T22[0] =  0.102*1000;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T32,R22,T22);
	VpV(T32,T2_t2,T32);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R32,T32);
		glPushMatrix();
		glMultMatrixd(oglm);
		link3_to_draw2->Draw();
		glPopMatrix();
	}

	T22[0] =  (0.069)*1000;
	T22[1] =  0;
	T22[2] =  (.26242-.015)*1000;

	MxV(T42,R32,T22);
	VpV(T42,T42,T32);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R42,T42);
		glPushMatrix();
		glMultMatrixd(oglm);
		link4_to_draw2->Draw();
		glPopMatrix();
	}

	T22[0] =  (0.10)*1000;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T52,R42,T22);
	VpV(T52,T42,T52);

	if(visualizeRobots == 1){
		glColor3d(redFrame[0],redFrame[1],redFrame[2]);
		MVtoOGL(oglm,R52,T52);
		glPushMatrix();
		glMultMatrixd(oglm);
		link5_to_draw2->Draw();
		glPopMatrix();
	}

	T22[0] =  (0.01)*1000;
	T22[1] =  0;
	T22[2] =  (.2707)*1000;

	MxV(T62,R52,T22);
	VpV(T62,T52,T62);

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R62,T62);
		glPushMatrix();
		glMultMatrixd(oglm);
		link6_to_draw2->Draw();
		glPopMatrix();
	}

	Ti[0]=0;Ti[1]=0;Ti[2]=0;


	T22[0] =  (.16)*1000;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T72,R62,T22);
	VpV(T72,T62,T72);

	if(visualizeRobots == 1){
		glColor3d(blackFrame[0],blackFrame[1],blackFrame[2]);
		MVtoOGL(oglm,R72,T72);
		glPushMatrix();
		glMultMatrixd(oglm);
		link7_to_draw2->Draw();
		glPopMatrix();
	}

	T22[0] =  0;
	T22[1] =  0;
	T22[2] =  0.05*1000.;

	MxV(TEE2,R72,T22);
	VpV(TEE2,T72,TEE2);

	if(visualizeRobots == 1){
		glColor3d(1.0,1.0,1.0);
		MVtoOGL(oglm,REE2,TEE2);
		glPushMatrix();
		glMultMatrixd(oglm);
		EE_to_draw2->Draw();
		glPopMatrix();
	}

	double width = 2000.0f;
	glBegin(GL_QUADS);
	glColor3f(1.5*117.0/255, 1.5*67.0/255, 1.5*54.0/255);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(width, -3*width, -850.f);
	glVertex3f(width, 3*width, -850.f);
	glVertex3f(-width, 3*width, -850.f);
	glVertex3f(-width, -3*width, -850.0f);
	glEnd();

	// ----- Reposition rod at center of EE
	T2[0] =  0;
	T2[1] =  0;//$.1*1000;
	T2[2] =  0.1*1000;
	MxV(Trod2,REE2,T2);
	VpV(Trod2,Trod2,TEE2);
	//pm(REE2, "Ree2");
	//pv(Trod, "Tee2");

	T2[0] =  0;
	T2[1] =  0;//$.1*1000;
	T2[2] =  0.1*1000;

	MxV(Trod,REE,T2);
	VpV(Trod,Trod,TEE);

	double pose_angle;
	if (!grasp_pose)
		pose_angle = -3.1415926/2; // Continuous to the arm
	else
		pose_angle = 3.1415926; // Perpendicular to the gripper plane

	MRotY(Mrod,pose_angle);
	MxM(Rrod, REE, Mrod);

	MRotX(Mrod,-3.1415926);
	MxM(Rrod_v, Rrod, Mrod);

	//pm(Rrod_v, "Rrod_v");
	//pv(Trod, "Trod");

	//pm(REE, "Ree");
	//pm(Rrod, "Rrod");
	//pm(Mrod, "Mrod");
	//pv(Trod, "Trod");

	int step1 = step-1;
	for(int i=0;i<RodStates[step1].size();i++){
		PQP_REAL P[3],V[3]; //,D1[3],D2[3],P1[3],P2[3];
		P[0]=RodStates[step1][i][0];
		P[1]=RodStates[step1][i][1];
		P[2]=RodStates[step1][i][2];

		//MxVpV(V,Rrod,P,TEE);
		MxVpV(V,Rrod_v,P,Trod);
		//glColor3d(.93, .69, .13);//.93, .69, .13);//
		glColor3d(1.0, 1.0, 1.0);//.93, .69, .13);//
		glPushMatrix();
		glTranslated(V[0],V[1],V[2]);
		glutSolidSphere(14,15,15);
		glPopMatrix();
	}

	// Draw clamps
	MRotY(Mclp,-3.1415926/2);
	MxM(Rclp, Rrod, Mclp);
	MRotX(Mclp,-3.1415926/2);
	MxM(Rrod, Rclp, Mclp);

	if(visualizeRobots == 1){
		glColor3d(1.0,1.0,1.0);
		MVtoOGL(oglm,Rrod,Trod);
		glPushMatrix();
		glMultMatrixd(oglm);
		clamps_to_draw->Draw();
		glPopMatrix();
	}

	MRotX(Mclp,-3.1415926/2);
	MxM(Rclp2, REE2, Mclp);

	if (grasp_pose) {
		MRotZ(Mclp,-3.1415926/2);
		MxM(MInt, Rclp2, Mclp);
		MRotX(Mclp,0);
		MxM(Rclp2, MInt, Mclp);
	}

	if(visualizeRobots == 1){
		glColor3d(1.0,1.0,1.0);
		MVtoOGL(oglm,Rclp2,Trod2);
		glPushMatrix();
		glMultMatrixd(oglm);
		clamps_to_draw2->Draw();
		glPopMatrix();
	}

	if(withObs){
		// Table
		MRotZ(R0,3.1415926/2);
		Ti[0] = 850; Ti[1] = 0; Ti[2] = 20;
		glColor3d(.93, .69, .13);//172/255.0, 102/255.0, 13/255.0);//0.0,0.0,1.0);
		MVtoOGL(oglm,R0,Ti);
		glPushMatrix();
		glMultMatrixd(oglm);
		table_to_draw->Draw();
		glPopMatrix();

		// Obs 1
		MRotZ(R0,0);
		Ti[0] = 900; Ti[1] = -500; Ti[2] = 20;
		glColor3d(1.0,1.0,1.0);
		MVtoOGL(oglm,R0,Ti);
		glPushMatrix();
		glMultMatrixd(oglm);
		obs1_to_draw->Draw();
		glPopMatrix();

		// Cone 1
		MRotZ(R0,-3.1415926/2);
		Ti[0] = Xc; Ti[1] = Yc; Ti[2] = 20;
		glColor3d(0.0,1.0,0.0);
		MVtoOGL(oglm,R0,Ti);
		glPushMatrix();
		glMultMatrixd(oglm);
		wall_to_draw->Draw();
		glPopMatrix();

		// Cone 2
		/*MRotX(R0,3.1415926);
		Ti[0] = 910; Ti[1] = 260; Ti[2] = 380*2+80;
		glColor3d(1.0,1.0,1.0);
		MVtoOGL(oglm,R0,Ti);
		glPushMatrix();
		glMultMatrixd(oglm);
		wall_to_draw->Draw();
		glPopMatrix();*/

	}

	EndDraw();
}


void load_models(){

	// initialize the base

	FILE *fp;
	int i, ntris;

	base_to_draw = new Model("base_link.tris");

	fp = fopen("base_link.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open base_link.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	base.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;

		base.AddTri(p1,p2,p3,i);
	}
	base.EndModel();
	fclose(fp);

	ped_to_draw = new Model("pedestal.tris");

	fp = fopen("pedestal.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open base_link.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	ped.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;

		ped.AddTri(p1,p2,p3,i);
	}
	ped.EndModel();
	fclose(fp);

	// initialize link 1

	link1_to_draw = new Model("S0.tris");

	fp = fopen("S0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link1_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link1.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link1.AddTri(p1,p2,p3,i);
	}
	link1.EndModel();
	fclose(fp);

	// initialize link2
	link2_to_draw = new Model("S1.tris");

	fp = fopen("S1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link2_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link2.AddTri(p1,p2,p3,i);
	}
	link2.EndModel();
	fclose(fp);

	// initialize link3
	link3_to_draw = new Model("E0.tris");

	fp = fopen("E0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link3_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link3.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link3.AddTri(p1,p2,p3,i);
	}
	link3.EndModel();
	fclose(fp);

	// initialize link4
	link4_to_draw = new Model("E1.tris");

	fp = fopen("E1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link4_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link4.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link4.AddTri(p1,p2,p3,i);
	}
	link4.EndModel();
	fclose(fp);

	// initialize link5
	link5_to_draw = new Model("W0.tris");

	fp = fopen("W0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link5_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link5.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link5.AddTri(p1,p2,p3,i);
	}
	link5.EndModel();
	fclose(fp);

	// initialize link6
	link6_to_draw = new Model("W1.tris");

	fp = fopen("W1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link6_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link6.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link6.AddTri(p1,p2,p3,i);
	}
	link6.EndModel();
	fclose(fp);

	// initialize L7
	link7_to_draw = new Model("W2.tris");

	fp = fopen("W2.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open EE_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link7.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link7.AddTri(p1,p2,p3,i);
	}
	link7.EndModel();
	fclose(fp);

	// initialize EE
	EE_to_draw = new Model("gripper.tris");

	fp = fopen("gripper.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open EE_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	EE.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		EE.AddTri(p1,p2,p3,i);
	}
	EE.EndModel();
	fclose(fp);

	//ROBOT 2

	// initialize link 1

	link1_to_draw2 = new Model("S0.tris");

	fp = fopen("S0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link1_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link12.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link12.AddTri(p1,p2,p3,i);
	}
	link12.EndModel();
	fclose(fp);

	// initialize link2
	link2_to_draw2 = new Model("S1.tris");

	fp = fopen("S1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link2_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link22.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link22.AddTri(p1,p2,p3,i);
	}
	link22.EndModel();
	fclose(fp);

	// initialize link3
	link3_to_draw2 = new Model("E0.tris");

	fp = fopen("E0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link3_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link32.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link32.AddTri(p1,p2,p3,i);
	}
	link32.EndModel();
	fclose(fp);

	// initialize link4
	link4_to_draw2 = new Model("E1.tris");

	fp = fopen("E1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link4_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link42.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link42.AddTri(p1,p2,p3,i);
	}
	link42.EndModel();
	fclose(fp);

	// initialize link5
	link5_to_draw2 = new Model("W0.tris");

	fp = fopen("W0.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link5_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link52.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link52.AddTri(p1,p2,p3,i);
	}
	link52.EndModel();
	fclose(fp);

	// initialize link6
	link6_to_draw2 = new Model("W1.tris");

	fp = fopen("W1.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open Link6_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link62.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link62.AddTri(p1,p2,p3,i);
	}
	link62.EndModel();
	fclose(fp);

	// initialize l7
	link7_to_draw2 = new Model("W2.tris");

	fp = fopen("W2.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open EE_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	link72.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		link72.AddTri(p1,p2,p3,i);
	}
	link72.EndModel();
	fclose(fp);

	// initialize EE
	EE_to_draw2 = new Model("gripper.tris");

	fp = fopen("gripper.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open EE_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	EE2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		EE2.AddTri(p1,p2,p3,i);
	}
	EE2.EndModel();
	fclose(fp);

	// initialize clamps
	clamps_to_draw = new Model("clamps.tris");

	fp = fopen("clamps.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open clamps.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	clmp.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		clmp.AddTri(p1,p2,p3,i);
	}
	clmp.EndModel();
	fclose(fp);

	// initialize clamps
	clamps_to_draw2 = new Model("clamps.tris");

	fp = fopen("clamps.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open clamps.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	clmp2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		clmp2.AddTri(p1,p2,p3,i);
	}
	clmp2.EndModel();
	fclose(fp);

	// Obstacles

	if (withObs) {

		// initialize table
		table_to_draw = new Model("table.tris");

		fp = fopen("table.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open table.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		table.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			table.AddTri(p1,p2,p3,i);
		}
		table.EndModel();
		fclose(fp);

		// initialize obs1
		obs1_to_draw = new Model("obs.tris");

		fp = fopen("obs.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open obs.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		obs1.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			obs1.AddTri(p1,p2,p3,i);
		}
		obs1.EndModel();
		fclose(fp);

		// initialize wall
		wall_to_draw = new Model("route.tris");

		fp = fopen("route.tris","r");
		if (fp == NULL) { fprintf(stderr,"Couldn't open route.tris\n"); exit(-1); }
		fscanf(fp,"%d",&ntris);

		wall.BeginModel();
		for (i = 0; i < ntris; i++)
		{
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
			p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
			p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
			wall.AddTri(p1,p2,p3,i);
		}
		wall.EndModel();
		fclose(fp);
	}


}

void load_path() {
	const char* rod_pfile = "../path/rod_path.txt";
	const char* robot_pfile = "../path/robot_paths.txt";
	FILE *fro, *fr;
	int i, nlines;

	fr = fopen(robot_pfile,"r");
	if (fr == NULL) { fprintf(stderr,"Couldn't open robot_path.txt\n"); exit(-1); }
	fscanf(fr,"%i",&nlines);  //NOT include number in line count itself
	RoboStates.resize(nlines);

	for (i = 0; i < nlines; i++)
	{
		double rot1T,rot2T,rot3T,rot4T,rot5T,rot6T,rot7T;
		double rot52T,rot62T,rot12T,rot22T,rot32T,rot42T,rot72T;
		fscanf(fr,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&rot1T,&rot2T,&rot3T,&rot4T,&rot5T,&rot6T,&rot7T,&rot12T, \
				&rot22T,&rot32T,&rot42T,&rot52T,&rot62T,&rot72T);

		RoboStates[i].resize(14);
		RoboStates[i][0]=rot1T;RoboStates[i][1]=rot2T;RoboStates[i][2]=rot3T;
		RoboStates[i][3]=rot4T;RoboStates[i][4]=rot5T;RoboStates[i][5]=rot6T;RoboStates[i][6]=rot7T;
		RoboStates[i][7]=rot12T;RoboStates[i][8]=rot22T;RoboStates[i][9]=rot32T;
		RoboStates[i][10]=rot42T;RoboStates[i][11]=rot52T;RoboStates[i][12]=rot62T;RoboStates[i][13]=rot72T;
	}

	fclose(fr);

	fro = fopen(rod_pfile,"r");
	if (fro == NULL) { fprintf(stderr,"Couldn't open rod_path.txt\n"); exit(-1); }
	fscanf(fro,"%i",&nlines);  //DO include number in line count itself

	int config_num = 0;
	RodStates.resize(nlines/500);

	for(i=0;i<nlines/500;i++){
		RodStates[i].resize(500);
		for(int j=0;j<500;j++){
			RodStates[i][j].resize(3);
		}
	}


	for (i = 2; i <= nlines; i++)
	{
		double px,py,pz;
		if (i-501*config_num == 502){
			config_num += 1;
			continue;
		}

		fscanf(fro,"%lf %lf %lf",&px,&py,&pz);

		int index = i-501*config_num-2;

		if (config_num == 0 && i < 500){index=i-2;}

		if (index == 500){std::cout << "error in indexing" << std::endl; break;}

		RodStates[config_num][index][0]=px;
		RodStates[config_num][index][1]=py;
		RodStates[config_num][index][2]=pz;
	}

	fclose(fro);

	if(RodStates.size() != RoboStates.size()){
		std::cout << "error! Non-equal sizes" << std::endl;
	}
}


void execute_path(int k){

	if(k == 0){
		const char* rod_pfile = "../path/rod_path.txt";
		const char* robot_pfile = "../path/robot_paths.txt";
		FILE *fro, *fr;
		int i, nlines;

		fr = fopen(robot_pfile,"r");
		if (fr == NULL) { fprintf(stderr,"Couldn't open robot_path.txt\n"); exit(-1); }
		fscanf(fr,"%i",&nlines);  //NOT include number in line count itself
		RoboStates.resize(nlines);

		for (i = 0; i < nlines; i++)
		{
			double rot1T,rot2T,rot3T,rot4T,rot5T,rot6T,rot7T;
			double rot52T,rot62T,rot12T,rot22T,rot32T,rot42T,rot72T;
			fscanf(fr,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
					&rot1T,&rot2T,&rot3T,&rot4T,&rot5T,&rot6T,&rot7T,&rot12T, \
					&rot22T,&rot32T,&rot42T,&rot52T,&rot62T,&rot72T);

			RoboStates[i].resize(14);
			RoboStates[i][0]=rot1T;RoboStates[i][1]=rot2T;RoboStates[i][2]=rot3T;
			RoboStates[i][3]=rot4T;RoboStates[i][4]=rot5T;RoboStates[i][5]=rot6T;RoboStates[i][6]=rot7T;
			RoboStates[i][7]=rot12T;RoboStates[i][8]=rot22T;RoboStates[i][9]=rot32T;
			RoboStates[i][10]=rot42T;RoboStates[i][11]=rot52T;RoboStates[i][12]=rot62T;RoboStates[i][13]=rot72T;
		}

		fclose(fr);

		fro = fopen(rod_pfile,"r");
		if (fro == NULL) { fprintf(stderr,"Couldn't open rod_path.txt\n"); exit(-1); }
		fscanf(fro,"%i",&nlines);  //DO include number in line count itself

		int config_num = 0;
		RodStates.resize(nlines/500);

		for(i=0;i<nlines/500;i++){
			RodStates[i].resize(500);
			for(int j=0;j<500;j++){
				RodStates[i][j].resize(3);
			}
		}


		for (i = 2; i <= nlines; i++)
		{
			double px,py,pz;
			if (i-501*config_num == 502){
				config_num += 1;
				continue;
			}

			fscanf(fro,"%lf %lf %lf",&px,&py,&pz);

			int index = i-501*config_num-2;

			if (config_num == 0 && i < 500){index=i-2;}

			if (index == 500){std::cout << "error in indexing" << std::endl; break;}

			RodStates[config_num][index][0]=px;
			RodStates[config_num][index][1]=py;
			RodStates[config_num][index][2]=pz;
		}

		fclose(fro);

		if(RodStates.size() != RoboStates.size()){
			std::cout << "error! Non-equal sizes" << std::endl;
		}
	}

	rot1 = RoboStates[k][0];
	rot2 = RoboStates[k][1];
	rot3 = RoboStates[k][2];
	rot4 = RoboStates[k][3];
	rot5 = RoboStates[k][4];
	rot6 = RoboStates[k][5];
	rot7 = RoboStates[k][6];
	rot12 = RoboStates[k][7];
	rot22 = RoboStates[k][8];
	rot32 = RoboStates[k][9];
	rot42 = RoboStates[k][10];
	rot52 = RoboStates[k][11];
	rot62 = RoboStates[k][12];
	rot72 = RoboStates[k][13];

	//rod.BeginModel();

	for (int i=0; i<RodStates[k].size()/3; i++){
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = RodStates[k][3*i][0];
		p1[1] = RodStates[k][3*i][1];
		p1[2] = RodStates[k][3*i][2];  //P1
		p2[0] = RodStates[k][3*i+1][0];
		p2[1] = RodStates[k][3*i+1][1];
		p2[2] = RodStates[k][3*i+1][2];  //P2
		p3[0] = RodStates[k][3*i+2][0];
		p3[1] = RodStates[k][3*i+2][1];
		p3[2] = RodStates[k][3*i+2][2];  //P3
		//rod.AddTri(p1,p2,p3,i);
	}

	if(k<RoboStates.size()) {
		//if (step==1)
		//	sleep(3);

		step+=1;
		//we see that middle pose gets called, second is not
		//transition rod is the second rod

		if (k!=RoboStates.size()-1)
			glutTimerFunc(sim_velocity,execute_path,step);

		if (step_sim) {
			std::cout << step << std::endl;
			std::cout << rot1 << " " << rot2 << " " << rot3 << " " << rot4 << " " << rot5 << " " << rot6 << " " << rot7 << " " << rot12 << " " << rot22 << " " << rot32 << " " << rot42 << " " << rot52 << " " << rot62 << " " << rot72 << " " << std::endl;
			std::cin.ignore();
		}
	}

	glutPostRedisplay();

}

int main(int argc, char **argv)
{
	if (argc > 1) {
		if (atof(argv[1])==0)
			step_sim = false;
		else if (atof(argv[1])==1)
			step_sim = true;
		if (argc > 2)
			sim_velocity = atoi(argv[2]);
		else
			sim_velocity = 200;
	}
	else
		sim_velocity = 200;


	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);

	// create the window

	glutCreateWindow("Robot View");

	// load robot meshes
	load_models();

	// set OpenGL graphics state -- material props, perspective, etc.

	InitViewerWindow();

	// set the callbacks
	glutTimerFunc(0,execute_path,0);

	glutDisplayFunc(DisplayCB);
	//glutIdleFunc(IdleCB);
	glutMouseFunc(MouseCB);
	glutMotionFunc(MotionCB);
	glutKeyboardFunc(KeyboardCB);
	glutReshapeWindow(1000,1000);

	// Enter the main loop.
	glutMainLoop();
}



