#compiler
CXX= g++

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/
LDFLAGS= -L${PQP_DIR}/lib -lPQP -lm $(GL_LIBS) 
CXXFLAGS= -I${PQP_DIR}/include $(GL_INCPATH)

all:
	$(CXX) main.cpp model.cpp -o viz $(CXXFLAGS) $(LDFLAGS)

