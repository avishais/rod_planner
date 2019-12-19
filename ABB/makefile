

OMPL_DIR = /usr
WRK_SPC = /home/avishai/Documents/workspace
INC_CLASSES = ${WRK_SPC}/rod_planner/ABB/classes/
ROD_CLASSES = ${WRK_SPC}/rod_planner/roadmap/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/
EIGEN_DIR = /home/avishai/Documents/eigen

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/local/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${PQP_DIR}/include $(GL_INCPATH) -I$(EIGEN_DIR) 
LDFLAGS= -L${OMPL_DIR}/local/lib -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -llapack -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -lPQP -lm $(GL_LIBS) 

# ABB
CPP_ADD2 = $(ROD_CLASSES)prm_gen.cpp myRRTConnect.cpp $(ROD_CLASSES)/classes/Rod_ODE_class.cpp $(ROD_CLASSES)/classes/slice.cpp $(INC_CLASSES)robots_class.cpp StateValidityChecker.cpp 
CPP_ADD3 = $(ROD_CLASSES)prm_gen.cpp $(ROD_CLASSES)/classes/Rod_ODE_class.cpp $(ROD_CLASSES)/classes/slice.cpp $(INC_CLASSES)robots_class.cpp StateValidityChecker.cpp 
CPPPQP1 = ${INC_CLASSES}collisionDetection.cpp ${INC_CLASSES}model.cpp

all:
	${CXX} plan.cpp ${CPP_ADD2} ${CPPPQP1} -o plan ${CXXFLAGS} ${LDFLAGS} -std=c++11 #-Wall
	#$(CXX) smooth.cpp $(CPP_ADD3) $(CPPPQP1) -o smo $(CXXFLAGS) $(LDFLAGS) -std=c++11 #-Wall
	#$(CXX) fillpath.cpp $(CPP_ADD3) $(CPPPQP1) -o fill $(CXXFLAGS) $(LDFLAGS) -std=c++11 #-Wall

