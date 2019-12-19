INC_CLASSES = /home/avishai/Documents/workspace/rod_planner/roadmap/classes/

CXX= g++

CPP_ADD1 = gen_roadmap.cpp prm_gen.cpp $(INC_CLASSES)Rod_ODE_class.cpp $(INC_CLASSES)slice.cpp  

all:
	$(CXX) $(CPP_ADD1) -o gen -std=c++11 #-Wall
	$(CXX) seq_gen_roadmap.cpp -o seq_gen -std=c++11
