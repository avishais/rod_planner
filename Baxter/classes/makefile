CXX= g++

KDL_DIR = /usr/local
EIGEN_DIR = /home/avishai/Documents/eigen

CXXFLAGS= -I${KDL_DIR}/include  -I$(EIGEN_DIR) 
LDFLAGS=  -L${KDL_DIR}/lib -L/usr/lib  -lorocos-kdl

all:
	$(CXX) robots_class.cpp -o kd $(LDFLAGS) $(CXXFLAGS) -std=c++11
