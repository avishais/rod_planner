#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "./classes/slice.h"
#include "./classes/Rod_ODE_class.h"
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <algorithm>
#include <iterator>
#include <time.h>

using namespace std;

typedef KDTreeVectorOfVectorsAdaptor< Matrix, double >  my_kd_tree_t;
typedef vector<vector< double > > Matrix;
typedef vector<int > VectorInt;
typedef vector<vector< int > > MatrixInt;


struct rodSoln {
	Matrix a;
	vector<Matrix> T_end;
	vector<Matrix> P;
	VectorInt s;
	VectorInt parents;
};

struct kNeighborSoln {
	VectorInt neighbors;
	State dist;
};

struct MS {
	Matrix milestones;
	MatrixInt Knearest;
	Matrix Kdistances; // <- not stored in the files
	MatrixInt first_sub_neighbor;
	MatrixInt first_sub_indicator; // Indicates whether the first sub neighbor is an ms or sub_ms.
	Matrix dis_graph;
	MatrixInt parent_graph;

	vector<Matrix> T_end;
	vector<Matrix> P;
	VectorInt s;
};

struct SUB_MS {
	Matrix sub_milestones;
	MatrixInt neighbors2;
	MatrixInt milestones_parents;
	MatrixInt neighbor_indicator; // Indicates whether the node on the respective side (left or right) is a milestone.

	vector<Matrix> T_end;
	vector<Matrix> P;
	VectorInt s;
};

struct NODE {
	State a;
	int index; // Index of milestone in MS or SUB_MS.
	int index_indicator; // Indicates whether the previous index is in MS (1) or SUB_MS (0)
};

class PRMGenerator: public Slice {

public:
	// Constructor
	PRMGenerator();

	void set_roadmap_parameters(int, int, bool);

	void generate_roadmap();

	bool check_1_component();

	int get_random_milestone();

	//rtn KD tree for start/end connection
	my_kd_tree_t buildKDtree(Matrix&);

	// PRM data
	MS ms;
	SUB_MS subms;

	vector <NODE> path;

	VectorInt out_shortest_path;

	void query_path();
	void load_data(string);

	State random_configuration();
	VectorInt add_start_n_goal_milestones(State, State, int);
	void add_neighbor_to_node(int);

	bool is2Dor6D = true; // Indicates whether this is a 2D problem (for debugging - false) or a 6D problem for rod planning (true)

	int get_num_tot_milestones() {
		num_tot_milestones = ms.milestones.size() + subms.sub_milestones.size();
		return num_tot_milestones;
	}

	// Performance parameters
	double query_time;
	double add_startNgoal_runtime;
	int add_startNgoal_counter;
	void reset_query_time() {
		query_time = 0;
	}
	double get_query_time() {
		return query_time;
	}
	int get_add_startNgoal_counter() {
		return add_startNgoal_counter;
	}
	double get_add_startNgoal_runtime() {
		return add_startNgoal_runtime;
	}
	void reset_add_startNgoal_measure() {
		add_startNgoal_runtime = 0;
		add_startNgoal_counter = 0;
	}


	void save_roadmap_to_file();
protected:

	bool check_validity(State&);

	void valid_milestone_gen(int&);

	void get_solution(rodSoln&, State&, State&);

	void kNeighbors(my_kd_tree_t&, State, kNeighborSoln&, size_t, bool);

	void all_paths();
	void update_all_paths(VectorInt, State);
	void update_all_paths_w_new_neighbor(int, int, double);
	bool shortest_path(int start, int goal);
	bool full_shortest_path(int start, int goal);
	bool full_shortest_path(NODE as, NODE ag); // Does NN query for start and goal, and find a path between them
	bool check_path_continuous();


	void save_data();

	int dimension;
	double num_pts; // number of required points in space
	int k; // Number of neighbors
	int num_tot_milestones;
	int saveFile;


};
