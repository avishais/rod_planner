#compiler
OMPL_DIR = /usr/local
INC_CLASSES_1 = /home/avishai/Downloads/omplapp/ompl/Workspace/precomputation/Baxter/
INC_CLASSES = /home/avishai/Downloads/omplapp/ompl/Workspace/precomputation/Baxter/classes/
ROD_CLASSES = /home/avishai/Downloads/omplapp/ompl/Workspace/precomputation/roadmap/

KDL_DIR = /usr/local
EIGEN_DIR = /home/avishai/Documents/eigen

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${PQP_DIR}/include $(GL_INCPATH) -I${KDL_DIR}/include -I$(EIGEN_DIR) 
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -lPQP -lm $(GL_LIBS) -L${KDL_DIR}/lib -L/usr/lib  -lorocos-kdl

# Baxter
CPP_ADD4 = $(ROD_CLASSES)prm_gen.cpp $(ROD_CLASSES)/classes/Rod_ODE_class.cpp $(ROD_CLASSES)/classes/slice.cpp $(INC_CLASSES)robots_class.cpp $(INC_CLASSES_1)StateValidityChecker.cpp 
CPPPQP2 = ${INC_CLASSES}collisionDetection.cpp ${INC_CLASSES}model.cpp

all:
	$(CXX) main.cpp $(CPP_ADD4) $(CPPPQP2) -o gen $(CXXFLAGS) $(LDFLAGS) -std=c++11 #-Wall		





