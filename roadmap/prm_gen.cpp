// Steve Macenski (c) 2017

#include "prm_gen.hpp"

using namespace nanoflann;
typedef KDTreeVectorOfVectorsAdaptor< Matrix, double >  my_kd_tree_t;

// Debugging tool
template <class T>
void o(T a) {
	cout << a << endl;
}

// Constructor - define roadmap parameters
PRMGenerator::PRMGenerator() {

	dimension = is2Dor6D ? 6 : 2;
	cout << "Problem dimension: " << dimension << endl;
	srand (2);//time(NULL));

}

void PRMGenerator::set_roadmap_parameters(int num_milestones, int num_neighbors, bool save2file) {

	num_pts = num_milestones;
	k = num_neighbors;
	saveFile = save2file;
	num_tot_milestones = 0;

	initMatrix(ms.dis_graph, num_pts, num_pts);
	initMatrix(ms.parent_graph, num_pts, num_pts);

}

bool PRMGenerator::check_1_component() {
	// Checks whether the roadmap has one component.
	// If there is a -1 in the parent_graph, this means that some nodes cannot be reached from everywhere (returns false).

	for (int i = 0; i < ms.parent_graph.size(); i++)
		for (int j = 0; j < ms.parent_graph[i].size(); j++)
			if (i != j && ms.parent_graph[i][j]==-1)
				return false;

	return true;
}

void PRMGenerator::valid_milestone_gen(int &num_subsamples){

	// make required num of subsamples in space and find solutions of rod for each
	bool valid_state;

	for (int i=0; i < num_subsamples; i++){
		valid_state = false;
		do {
			//cout << "spinning on: " << i+1  << endl;
			ms.milestones[i] = random_configuration();
			valid_state = check_validity(ms.milestones[i]);
		} while (!valid_state);

		if (!is2Dor6D) { //2D
			ms.T_end.push_back({{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}});
			ms.P.push_back({{0,0,1},{0,0,2},{0,0,3}});
			ms.s.push_back(i);
		}
		else {
			ms.T_end.push_back(getT(get_Points_on_Rod()-1));
			ms.P.push_back(getPMatrix());
			ms.s.push_back(get_Points_on_Rod()-1);
		}

		/*cout << "ms: \n"; printVector(ms.milestones[i]);
		cout << "T_end: \n"; printMatrix(ms.T_end[i]);
		cin.ignore();
		cout << "P: \n"; printMatrix(ms.P[i]);
		cout << "s: " << ms.s[i] << endl;
		cin.ignore();*/
	}
}

void PRMGenerator::generate_roadmap(){
	cout << "Starting roadmap generation..." << endl;
	// number of required points in space
	int num_subsamples = num_pts;
	initMatrix(ms.milestones, num_subsamples, dimension);

	// ========================== Generate milestones ======================================

	//generate valid random milestones
	valid_milestone_gen(num_subsamples);

	cout << "Generated " << ms.milestones.size() << " milestones out of " << num_pts << " demanded." << endl;

	// Just for debugging
	//load_data();
	//ms.Knearest.clear();
	//ms.Kdistances.clear();

	// ========================== NN on the milestones level ======================================

	my_kd_tree_t KDtree(dimension, ms.milestones, 10);
	KDtree.index->buildIndex();

	ms.Knearest.resize(num_subsamples);
	ms.Kdistances.resize(num_subsamples);
	kNeighborSoln neighborSearch;

	for(int i=0; i < ms.milestones.size(); i++){
		kNeighbors(KDtree, ms.milestones[i], neighborSearch, min(4*k, num_pts-1), true);

		// Make sure there are no duplicate edges - we do this first, and after the all_path search, we add duplicate edges (from and to, vice versa)
		for (int r = 0; r < neighborSearch.neighbors.size() && ms.Knearest[i].size() < k; r++) {
			if (neighborSearch.neighbors[r] > i) { // Stores the first time the edge is seen
				ms.Knearest[i].push_back(neighborSearch.neighbors[r]);
				ms.Kdistances[i].push_back(neighborSearch.dist[r]);
			}
			else { // Checks if the current edge was already been stored. If so, check the next nearest neighbor.
				bool exist = false;
				for (int f = 0; f < ms.Knearest[neighborSearch.neighbors[r]].size(); f++)
					if (ms.Knearest[neighborSearch.neighbors[r]][f]==i)
						exist = true;
				if (!exist) {
					ms.Knearest[i].push_back(neighborSearch.neighbors[r]);
					ms.Kdistances[i].push_back(neighborSearch.dist[r]);
				}
			}
		}
	}

	// ========================== Save All-pairs shortest path ======================================

	// Solve all-pairs shortest path

	//clock_t st = clock();
	all_paths();
	//clock_t et = clock();
	//cout << "all_path() time: " << (double)(et-st)/CLOCKS_PER_SEC << "s." << endl;

	// ========================== Complete NN table ======================================

	int num_milestones = ms.milestones.size();

	// Add neighbors to milestone i of which it is a neighbor to (add duplicate edges)
	for (int i = 0; i < num_milestones; i++) {
		for (int j = 0; j < k; j++){
			int d = 0;
			while (d < ms.Knearest[ms.Knearest[i][j]].size() && ms.Knearest[ms.Knearest[i][j]][d]!=i) d++; // Check existence first
			if (d < ms.Knearest[ms.Knearest[i][j]].size())
				continue;
			//if (ms.Knearest[i][j] > i) {
			ms.Knearest[ms.Knearest[i][j]].push_back(i);
			ms.Kdistances[ms.Knearest[i][j]].push_back(ms.Kdistances[i][j]);
		}
	}

	// ========================== Generate sub_milestones ======================================
	// find solutions for each two neighboring milestones, store sub-milestones in graph

	vector<rodSoln > solutions;

	rodSoln current_solution;

	// Allocate memory for first_sub_neighbor and first_sub_indicator
	ms.first_sub_neighbor.resize(num_milestones);
	ms.first_sub_indicator.resize(num_milestones);
	for(int i = 0; i < num_milestones; i++) {
		ms.first_sub_neighbor[i].resize(ms.Knearest[i].size());
		ms.first_sub_indicator[i].resize(ms.Knearest[i].size());
	}

	// for every two neighboring milestones
	for(int i = 0; i < num_milestones; i++) {
		for(int s = 0; s < ms.Knearest[i].size(); s++) {

			// Avoid checking the same couple twice
			if (i > ms.Knearest[i][s])
				continue;

			// for each set of milestones and neighbor, find and store soln
			get_solution(current_solution, ms.milestones[i], ms.milestones[ms.Knearest[i][s]]);
			current_solution.parents = {i, ms.Knearest[i][s]};

			if (current_solution.a.size()==0) {
				ms.first_sub_neighbor[i][s] = ms.Knearest[i][s];
				ms.first_sub_indicator[i][s] = 1;
				int j = 0;
				while (ms.Knearest[ms.Knearest[i][s]][j] != i)
					j++;
				ms.first_sub_neighbor[ms.Knearest[i][s]][j] = i;
				ms.first_sub_indicator[ms.Knearest[i][s]][j] = 1;
				continue;
			}

			ms.first_sub_neighbor[i][s] = (int)subms.sub_milestones.size();
			ms.first_sub_indicator[i][s] = 0;

			if (current_solution.a.size() == 1) { // If there is only one node
				subms.sub_milestones.push_back(current_solution.a[0]);
				subms.T_end.push_back(current_solution.T_end[0]);
				subms.P.push_back(current_solution.P[0]);
				subms.s.push_back(current_solution.s[0]);
				subms.neighbor_indicator.push_back({1, 1});
				subms.neighbors2.push_back({i, ms.Knearest[i][s]});
				subms.milestones_parents.push_back(current_solution.parents);

				//ms.milestones[i].first_sub_neighbor
			}
			else
				for(int q = 0; q < current_solution.a.size(); q++) {
					subms.sub_milestones.push_back(current_solution.a[q]);
					subms.T_end.push_back(current_solution.T_end[q]);
					subms.P.push_back(current_solution.P[q]);
					subms.s.push_back(current_solution.s[q]);
					subms.milestones_parents.push_back(current_solution.parents);
					if (q==0) {
						subms.neighbor_indicator.push_back({1, 0});
						subms.neighbors2.push_back({i, (int)subms.sub_milestones.size()});
					}
					else if (q==current_solution.a.size()-1) {
						subms.neighbor_indicator.push_back({0, 1});
						subms.neighbors2.push_back({(int)(subms.sub_milestones.size()-2), ms.Knearest[i][s]});
					}
					else {
						subms.neighbor_indicator.push_back({0, 0});
						subms.neighbors2.push_back({(int)(subms.sub_milestones.size()-2), (int)subms.sub_milestones.size()});
					}
				}

			int j = 0;
			while (ms.Knearest[ms.Knearest[i][s]][j] != i)
				j++;
			ms.first_sub_neighbor[ms.Knearest[i][s]][j] = (int)subms.sub_milestones.size()-1;
			ms.first_sub_indicator[ms.Knearest[i][s]][j] = 0;
		}
	}

	// Now we have a graph of ALL sub/milestones
	num_tot_milestones = ms.milestones.size();

	//cout << "Knearest: \n"; printMatrix(ms.Knearest);
	//cout << "first_sub_neighbor: \n"; printMatrix(ms.first_sub_neighbor);
	//cout << "first_sub_indicator: \n"; printMatrix(ms.first_sub_indicator);

	cout << "Roadmap generated." << endl;

	// ========================== Save to files ======================================
	// Save to file
	if (saveFile){
		cout << "Saving data..." << endl;
		save_roadmap_to_file();
		save_data();
		//load_data();
	}

	num_tot_milestones = ms.milestones.size() + subms.sub_milestones.size();

}

void PRMGenerator::get_solution(rodSoln &soln, State &node, State &nearest_node){
	soln.a.clear();
	soln.T_end.clear();
	soln.P.clear();
	soln.s.clear();

	if (is2Dor6D) {
		// Get solutions from planning class
		a_start_ = node;
		a_goal_  = nearest_node;
		if (0)
			FindStable_A_Path_Exact();
		else
			FindStable_A_Path_w_Avg();

		for (int i = 1; i < a_steps-1; i++) {
			soln.a.push_back(A_Path[i].a);
			soln.T_end.push_back(A_Path[i].T_end_of_rod);
			soln.P.push_back(A_Path[i].P);
			soln.s.push_back(A_Path[i].s);
		}
	}
	else {
		const double dt = 0.5;
		double d = normDistance(node, nearest_node);
		int n = ceil(d/dt);
		State inter(node.size());

		for (int i = 1; i < n-1; i++) {
			for (int j = 0; j < node.size(); j++)
				inter[j] = node[j]*(1-(double)i/(n-1))  +  nearest_node[j]*((double)i/(n-1));

			soln.a.push_back(inter);
			soln.T_end.push_back({{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}});
			soln.P.push_back({{0,0,0},{0,0,0}});
			soln.s.push_back(0);
		}
	}
}

int PRMGenerator::get_random_milestone(){
	//returns index of random milestones
	int rtn = rand() % num_tot_milestones-1;
	return rtn;
}

State PRMGenerator::random_configuration(){
	//random vector of length DOF <= +/-30
	State config(dimension);
	double f;
	double FMax, FMin;

	if (!is2Dor6D) {
		FMax = 10;
		FMin = -10;
	}
	else {
		FMax = 30;
		FMin = -30;
	}

	for (int d=0; d < dimension; d++){
		f = (double)rand() / RAND_MAX;
		config[d] = FMin + f*(FMax-FMin);
	}

	//printVector(config);

	return config;
}

bool PRMGenerator::check_validity(State &state){

	if (!is2Dor6D)
		return true; // No valid checking for the 2D problem

	return isRodFeasible(state); // <= for the rod
}

void PRMGenerator::kNeighbors(my_kd_tree_t& mat_index, State query, kNeighborSoln& soln, size_t num_results, bool remove_1st_neighbor){
	// find nearest neighbors for node i in configs, which are 6-D A's.

	// do a knn search
	if (remove_1st_neighbor)
		num_results += 1;

	vector<size_t> ret_indexes(num_results);
	State out_dists_sqr(num_results);

	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

	State query_pnt = query;
	mat_index.index->findNeighbors(resultSet, &query_pnt[0], SearchParams(10));

	VectorInt rtn_values(ret_indexes.begin(), ret_indexes.end());

	if (remove_1st_neighbor) {
		rtn_values.erase(rtn_values.begin()); // Remove first node that is itself.
		out_dists_sqr.erase(out_dists_sqr.begin()); // Remove first node that is itself.
	}

	State out_dists(out_dists_sqr.size());
	for (int i = 0; i < out_dists_sqr.size(); i++)
		out_dists[i] = sqrt(out_dists_sqr[i]);

	// If some error pops, try this
	//soln.neighbors.resize(rtn_values.size());
	//soln.dist.resize(rtn_values.size());

	soln.neighbors = rtn_values;
	soln.dist = out_dists;
}

my_kd_tree_t PRMGenerator::buildKDtree(vector<vector<double> >& configurations){
	my_kd_tree_t mat_index(6, configurations, 10); //10 is leaf size
	mat_index.index->buildIndex();
}

void PRMGenerator::all_paths() {
	// All-pairs shortest path with Floyd- Warshall - based on:
	// http://www.cs.cornell.edu/~wdtseng/icpc/notes/graph_part3.pdf

	// Initialize graph matrix to Inf
	for (int i = 0; i < num_pts; i++)
		for (int j = 0; j < num_pts; j++)
			ms.dis_graph[i][j] = INT_MAX/2;

	// Add edge lengths to graph matrix
	for (int i = 0; i < ms.Knearest.size(); i++)
		for (int j = 0; j < ms.Knearest[i].size(); j++) {
			ms.dis_graph[i][ms.Knearest[i][j]] = ms.Kdistances[i][j];
			ms.dis_graph[ms.Knearest[i][j]][i] = ms.Kdistances[i][j];
		}

	for (int i = 0; i < ms.dis_graph.size(); i++)
		ms.dis_graph[i][i] = 0;

	// Initialize parent matrix to existing edges
	for (int i = 0; i < num_pts; i++)
		for (int j = 0; j < num_pts; j++) {
			if (i==j || ms.dis_graph[i][j]==INT_MAX/2)
				ms.parent_graph[i][j] = -1;
			else
				ms.parent_graph[i][j] = i;
		}

	for( int k = 0; k < num_pts; k++ )
		for( int i = 0; i < num_pts; i++ )
			for( int j = 0; j < num_pts; j++ ) {
				double newD = ms.dis_graph[i][k] + ms.dis_graph[k][j];
				if( newD < ms.dis_graph[i][j] ) {
					ms.dis_graph[i][j] = newD;
					ms.parent_graph[i][j] = ms.parent_graph[k][j];
				}
			}
}

void PRMGenerator::update_all_paths(VectorInt neighbors, State dist) {
	// Update the all pairs tables by adding a row and a column for a new node

	int n = ms.dis_graph.size();

	// ------Faster update by only checking neighbors------

	// Add column to dis_graph and parent_graph
	for (int i = 0; i < n; i++) {
		ms.dis_graph[i].push_back(INT_MAX/2);
		ms.parent_graph[i].push_back(-1);
	}

	int j = n;

	// Add row to parent_graph and dis_graph
	ms.parent_graph.push_back(ms.parent_graph[n-1]); ms.parent_graph[j][j] = -1;
	State dg_row(n+1, INT_MAX/2);
	ms.dis_graph.push_back(dg_row); ms.dis_graph[j][j] = 0;

	// Update by only checking neighbors
	for (int i = 0; i < n; i++) {
		for (int k = 0; k < neighbors.size(); k++) {
			double newD = ms.dis_graph[i][neighbors[k]]+dist[k];
			//cout << newD << endl;
			if (newD < ms.dis_graph[i][j]) {
				ms.dis_graph[i][j] = newD;
				ms.parent_graph[i][j] = neighbors[k];
			}
		}
		//ms.dis_graph[j][i] = ms.dis_graph[i][j];
	}

	double newD;
	for (int i = 0; i < n; i++) {
		for (int k = 0; k < ms.Knearest[i].size(); k++) {
			if (ms.Knearest[i][k] == j)
				newD = ms.dis_graph[i][j];
			else
				newD = ms.dis_graph[ms.Knearest[i][k]][i] + ms.dis_graph[ms.Knearest[i][k]][j];
			if (newD < ms.dis_graph[j][i]) {
				ms.dis_graph[j][i] = newD;
				ms.parent_graph[j][i] = ms.Knearest[i][k];
			}
		}
	}

	/*
	// ----- Update with the regular algorithm -----
	// Add column to dis_graph
	for (int i = 0; i < n; i++)
		ms.dis_graph[i].push_back(INT_MAX/2);
	for (int i = 0; i < neighbors.size(); i++)
		ms.dis_graph[neighbors[i]][n] = dist[i];

	int j = ms.dis_graph[0].size()-1;

	// Add row to dis_graph
	Vector dist_row(n+1, 0);
	for (int i = 0; i < n; i++)
		dist_row[i] = ms.dis_graph[i][j];
	ms.dis_graph.push_back(dist_row);

	// Add column to parent_graph
	VectorInt pg_row(n+1, -1);
	for (int i = 0; i < n; i++) {
		ms.parent_graph[i].push_back(i);
		if (i==j || ms.dis_graph[i][j] == INT_MAX/2)
			ms.parent_graph[i][j] = -1;
		else
			pg_row[i] = j;
	}
	ms.parent_graph.push_back(pg_row);

	// Update matrices
	for (int k = 0; k < n; k++)
		for (int i = 0; i < n; i++) {
			double newD = ms.dis_graph[i][k]+ms.dis_graph[k][j];
			if (newD < ms.dis_graph[i][j]) {
				ms.dis_graph[i][j] = newD;
				ms.parent_graph[i][j] = ms.parent_graph[k][j];
			}
			newD = ms.dis_graph[j][k]+ms.dis_graph[k][i];
			if (newD < ms.dis_graph[j][i]) {
				ms.dis_graph[j][i] = newD;
				ms.parent_graph[j][i] = ms.parent_graph[k][i];
			}
		}
	 */

	/*cout << "After.\n";
	cout << "dis_graph: \n";
	printMatrix(ms.dis_graph);
	cout << "parent_graph: \n";
	printMatrix(ms.parent_graph);*/

}

void PRMGenerator::update_all_paths_w_new_neighbor(int node_index, int new_neighbor, double new_dist) {
	int j = node_index;

	// Update by only checking neighbors
	for (int i = 0; i < ms.dis_graph.size(); i++) {
		double newD = ms.dis_graph[i][new_neighbor] + new_dist;
		if (newD < ms.dis_graph[i][j]) {
			ms.dis_graph[i][j] = newD;
			ms.parent_graph[i][j] = new_neighbor;
		}
	}

	double newD;
	for (int i = 0; i < j; i++) {
		for (int k = 0; k < ms.Knearest[i].size(); k++) {
			if (ms.Knearest[i][k] == j)
				newD = ms.dis_graph[i][j];
			else
				newD = ms.dis_graph[ms.Knearest[i][k]][i] + ms.dis_graph[ms.Knearest[i][k]][j];
			if (newD < ms.dis_graph[j][i]) {
				ms.dis_graph[j][i] = newD;
				ms.parent_graph[j][i] = ms.Knearest[i][k];
			}
		}
	}
}

// Finds the shortest path including only milestones
bool PRMGenerator::shortest_path(int start, int goal) {
	out_shortest_path.clear();

	out_shortest_path.push_back(start);

	if (start == goal)
		return true;

	int cur = start;
	bool found = false;
	while (!found) {
		int next = ms.parent_graph[goal][cur];

		if (next==-1)
			return false;

		out_shortest_path.push_back(next);

		if (next==goal)
			found = true;
		else
			cur = next;
	}

	return true;
}

// Finds the shortest path between any two milestones including sub-milestones in the path
bool PRMGenerator::full_shortest_path(int start, int goal) {

	if (!shortest_path(start, goal))
		return false;

	//path.clear();
	//printVector(out_shortest_path);

	if (out_shortest_path.size() > 1)
		for (int i = 0; i < out_shortest_path.size()-1; i++) {
			int as = out_shortest_path[i];
			int ag = out_shortest_path[i+1];

			path.push_back({ms.milestones[as], as, 1});

			int j = 0;
			while (ms.Knearest[as][j] != ag) j++;
			if (ms.first_sub_indicator[as][j] != 1) { // there are sub_ms in between as<->ag
				int cur_sub_ms = ms.first_sub_neighbor[as][j];
				int dirc = subms.milestones_parents[cur_sub_ms][0] == ag ? 0 : 1;
				while (subms.neighbor_indicator[cur_sub_ms][dirc] != 1) {
					path.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});

					cur_sub_ms = subms.neighbors2[cur_sub_ms][dirc];
				}
				path.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});
			}
		}

	int ag = out_shortest_path[out_shortest_path.size()-1];
	path.push_back({ms.milestones[ag], ag, 1});

	// Save to file (for graphical validation)
	/*std::string pathSoln("./data/full_path.txt");
	std::ofstream F(pathSoln);

	for (int i = 0; i < path.size(); i++) {
		std::copy(path[i].a.begin(), path[i].a.end(), std::ostream_iterator<double>(F, " "));
		F << "\n";
	}*/

	return true;
}

// Find the full shortest path between any two milestones or sub-milestones
bool PRMGenerator::full_shortest_path(NODE as, NODE ag) {
	// 'ag' and 'as' can be ms or sub-ms indicated by the 'as/ag.indicator'

	clock_t startT = clock();

	path.clear();

	// If both sub_milestones and on the same edge
	if (!as.index_indicator && !ag.index_indicator && subms.milestones_parents[as.index][0]==subms.milestones_parents[ag.index][0] && subms.milestones_parents[as.index][1]==subms.milestones_parents[ag.index][1]) {
		int ms_left_index = subms.milestones_parents[as.index][0];
		int ms_right_index = subms.milestones_parents[as.index][1];

		int j = 0;	while (ms.Knearest[ms_left_index][j] != ms_right_index)	j++;
		int sub_inx = ms.first_sub_neighbor[ms_left_index][j];
		int dirc =  abs(as.index-sub_inx) < abs(ag.index-sub_inx) ? 1 : 0;
		int cur_sub_ms = as.index;
		while (cur_sub_ms != ag.index) {
			path.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});
			cur_sub_ms = subms.neighbors2[cur_sub_ms][dirc];
		}
		path.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});

		goto log_path;
	}
	{ // ** Choose the milestones near the query points that provide the shortest path

		int index_ms4ag, index_ms4as;
		// In case both 'as' and 'ag' are ms
		if (as.index_indicator && ag.index_indicator) {
			index_ms4as = as.index;
			index_ms4ag = ag.index;
		}

		VectorInt ms4ag(2), ms4as(2);
		// In case 'as' is ms and 'ag' is subms
		if (as.index_indicator && !ag.index_indicator) {
			ms4ag = subms.milestones_parents[ag.index];
			index_ms4as = as.index;
			State asag = {ms.dis_graph[index_ms4as][ms4ag[0]], ms.dis_graph[index_ms4as][ms4ag[1]]};
			if (asag[0] < asag[1])
				index_ms4ag = ms4ag[0];
			else
				index_ms4ag = ms4ag[1];
		}
		// In case 'as' is a subms and 'ag' is ms
		if (!as.index_indicator && ag.index_indicator) {
			ms4as = subms.milestones_parents[as.index];
			index_ms4ag = ag.index;
			State asag = {ms.dis_graph[ms4as[0]][index_ms4ag], ms.dis_graph[ms4as[1]][index_ms4ag]};
			if (asag[0] < asag[1])
				index_ms4as = ms4as[0];
			else
				index_ms4as = ms4as[1];
		}
		// In case both 'as' and 'ag' are subms
		if (!as.index_indicator && !ag.index_indicator) {
			ms4as = subms.milestones_parents[as.index];
			ms4ag = subms.milestones_parents[ag.index];
			//printVector(ms4as);
			//printVector(ms4ag);

			State asag = {ms.dis_graph[ms4as[0]][ms4ag[0]], ms.dis_graph[ms4as[0]][ms4ag[1]], ms.dis_graph[ms4as[1]][ms4ag[0]], ms.dis_graph[ms4as[1]][ms4ag[1]]};
			//printVector(asag);
			int asag_i = 0;
			for (int i = 1; i < asag.size(); i++)
				if (asag[asag_i] > asag[i])
					asag_i = i;
			switch (asag_i) {
			case 0 :
				index_ms4as = ms4as[0]; index_ms4ag = ms4ag[0];
				break;
			case 1 :
				index_ms4as = ms4as[0]; index_ms4ag = ms4ag[1];
				break;
			case 2 :
				index_ms4as = ms4as[1]; index_ms4ag = ms4ag[0];
				break;
			case 3 :
				index_ms4as = ms4as[1]; index_ms4ag = ms4ag[1];
				break;
			}
		}

		int cur_sub_ms, dirc;
		// Log the sub_milestones from 'as' to ms4as (milestone near as)
		if (!as.index_indicator) {
			cur_sub_ms = as.index;
			dirc = subms.milestones_parents[cur_sub_ms][0] == index_ms4as ? 0 : 1;
			while (subms.neighbor_indicator[cur_sub_ms][dirc] != 1) {
				path.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});
				cur_sub_ms = subms.neighbors2[cur_sub_ms][dirc];
			}
			path.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});
		}

		// Log all milestones and sub-milestones from index_ms4as to index_ms4ag
		if (!full_shortest_path(index_ms4as, index_ms4ag)) {
			query_time += double(clock() - startT) / CLOCKS_PER_SEC;
			return false;
		}

		// Log the sub_milestones from ms4ag (milestone near as) to 'ag'
		if (!ag.index_indicator) {
			vector<NODE> path_ag;
			cur_sub_ms = ag.index;
			dirc = subms.milestones_parents[cur_sub_ms][0] == index_ms4ag ? 0 : 1;
			while (subms.neighbor_indicator[cur_sub_ms][dirc] != 1) { // First log backwards
				path_ag.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});
				cur_sub_ms = subms.neighbors2[cur_sub_ms][dirc];
			}
			path_ag.push_back({subms.sub_milestones[cur_sub_ms], cur_sub_ms, 0});
			for (int i = path_ag.size()-1; i >=0; i--) // Then, log in the right order to 'path'
				path.push_back(path_ag[i]);
		}
	}

	log_path:
	bool in6D = false;
	if (!is2Dor6D || in6D) { // Save to file (for graphical validation)
		std::string pathSoln("./data/full_path.txt");
		std::ofstream F(pathSoln);

		for (int i = 0; i < path.size(); i++) {
			std::copy(path[i].a.begin(), path[i].a.end(), std::ostream_iterator<double>(F, " "));
			F << "\n";
		}
		F.close();

		if (in6D) { // Save rod path (xyz) to file (for graphical validation)
			std::string rodSoln("./data/full_path_rod.txt");
			std::ofstream R(rodSoln);

			for (int i = 0; i < path.size(); i++) {
				for (int j = 0; j < get_Points_on_Rod(); j++) {
					if (path[i].index_indicator)
						std::copy(ms.P[path[i].index][j].begin(), ms.P[path[i].index][j].end(), std::ostream_iterator<double>(R, " "));
					else
						std::copy(subms.P[path[i].index][j].begin(), subms.P[path[i].index][j].end(), std::ostream_iterator<double>(R, " "));
					R << "\n";
				}
			}
			R.close();
		}
	}

	query_time += double(clock() - startT) / CLOCKS_PER_SEC;
	return true;
}

// Used only for testing paths queries
void PRMGenerator::query_path() {
	//  Test shortest path query

	/*if (shortest_path(0, 1)) {
		cout << "Shortest path: ";
		printVector(out_shortest_path);
	}
	else
		cout << "No path!\n";*/

	int j, i;
	i = 196; j = 0;
	printVector(j ? ms.milestones[i] : subms.sub_milestones[i]);
	NODE as = {j ? ms.milestones[i] : subms.sub_milestones[i], i, j};

	i = 27; j = 0;//rand() % subms.sub_milestones.size(); //104
	cout << "Goal index: " << i << endl;
	printVector(j ? ms.milestones[i] : subms.sub_milestones[i]);
	NODE ag = {j ? ms.milestones[i] : subms.sub_milestones[i], i, j};

	if (!full_shortest_path(as, ag))
		cout << "Path NOT found." << endl;
	else
		cout << "Found solution." << endl;
}

VectorInt PRMGenerator::add_start_n_goal_milestones(State as, State ag, int num_neighbors) {

	cout << "Adding start node : "; printVector(as);
	cout << "Adding goal node  : "; printVector(ag);

	add_startNgoal_counter += 2;
	clock_t startTime = clock();

	VectorInt new_ms_indices;
	my_kd_tree_t KDtree(dimension, ms.milestones, 10);
	KDtree.index->buildIndex();

	// Find nearest neighbors
	kNeighborSoln neighborSearch;
	for (int j = 0; j < 2; j++) {
		// Add to milestones list and record indices
		if (j==0) {
			ms.milestones.push_back(as);
			new_ms_indices.push_back(ms.milestones.size()-1);
			if (!check_validity(as)) {
				cout << "Error: invalid start milestone." << endl;
				exit(1);
			}
		}
		else {
			ms.milestones.push_back(ag);
			new_ms_indices.push_back(ms.milestones.size()-1);
			if (!check_validity(ag)) {
				cout << "Error: invalid goal milestone." << endl;
				exit(1);
			}
		}

		ms.T_end.push_back(getT(get_Points_on_Rod()-1));
		ms.P.push_back(getPMatrix());
		ms.s.push_back(get_Points_on_Rod()-1);

		// If in the future we would like to add the option that ag connects to as, than initiate here the KDtree (currently not working in this form):
		//my_kd_tree_t KDtree(dimension, ms.milestones, 10);
		//KDtree.index->buildIndex();

		int i = new_ms_indices[j];
		kNeighbors(KDtree, ms.milestones[i], neighborSearch, num_neighbors, false); // Might consider increasing k

		//printVector(neighborSearch.neighbors);
		//printVector(neighborSearch.dist);
		ms.Knearest.push_back(neighborSearch.neighbors);
		ms.Kdistances.push_back(neighborSearch.dist);
		for (int q = 0; q < neighborSearch.neighbors.size(); q++) {// Update neighbors of the new node
			ms.Knearest[neighborSearch.neighbors[q]].push_back(i);
			ms.Kdistances[neighborSearch.neighbors[q]].push_back(neighborSearch.dist[q]);
		}

		update_all_paths(neighborSearch.neighbors, neighborSearch.dist);

		rodSoln current_solution;

		for(int s = 0; s < ms.Knearest[i].size(); s++) {

			// for each set of milestones and neighbor, find and store soln
			get_solution(current_solution, ms.milestones[i], ms.milestones[ms.Knearest[i][s]]);
			current_solution.parents = {i, ms.Knearest[i][s]};

			ms.first_sub_neighbor.push_back({-1});
			ms.first_sub_indicator.push_back({-1});

			if (current_solution.a.size()==0) {
				ms.first_sub_neighbor[i][s] = ms.Knearest[i][s];
				ms.first_sub_indicator[i][s] = 1;
				ms.first_sub_neighbor[ms.Knearest[i][s]].push_back(i);
				ms.first_sub_indicator[ms.Knearest[i][s]].push_back(1);
				continue;
			}

			ms.first_sub_neighbor[i][s] = (int)subms.sub_milestones.size();
			ms.first_sub_indicator[i][s] = 0;

			if (current_solution.a.size() == 1) { // If there is only one node
				subms.sub_milestones.push_back(current_solution.a[0]);
				subms.T_end.push_back(current_solution.T_end[0]);
				subms.P.push_back(current_solution.P[0]);
				subms.s.push_back(current_solution.s[0]);
				subms.neighbor_indicator.push_back({1, 1});
				subms.neighbors2.push_back({i, ms.Knearest[i][s]});
				subms.milestones_parents.push_back(current_solution.parents);

				//ms.milestones[i].first_sub_neighbor
			}
			else
				for(int q = 0; q < current_solution.a.size(); q++) {

					subms.sub_milestones.push_back(current_solution.a[q]);
					subms.T_end.push_back(current_solution.T_end[q]);
					subms.P.push_back(current_solution.P[q]);
					subms.s.push_back(current_solution.s[q]);
					subms.milestones_parents.push_back(current_solution.parents);
					if (q==0) {
						subms.neighbor_indicator.push_back({1, 0});
						subms.neighbors2.push_back({i, (int)subms.sub_milestones.size()});
					}
					else if (q==current_solution.a.size()-1) {
						subms.neighbor_indicator.push_back({0, 1});
						subms.neighbors2.push_back({(int)(subms.sub_milestones.size()-2), ms.Knearest[i][s]});
					}
					else {
						subms.neighbor_indicator.push_back({0, 0});
						subms.neighbors2.push_back({(int)(subms.sub_milestones.size()-2), (int)subms.sub_milestones.size()});
					}
				}

			ms.first_sub_neighbor[ms.Knearest[i][s]].push_back((int)subms.sub_milestones.size()-1);
			ms.first_sub_indicator[ms.Knearest[i][s]].push_back(0);
		}

	}

	//cout << "Start and goal nodes added to Roadmap." << endl;
	//save_roadmap_to_file();

	add_startNgoal_runtime += double(clock() - startTime) / CLOCKS_PER_SEC;

	return new_ms_indices;
}

void PRMGenerator::add_neighbor_to_node(int node_index) {

	add_startNgoal_counter++;
	clock_t startTime = clock();

	my_kd_tree_t KDtree(dimension, ms.milestones, 10);
	KDtree.index->buildIndex();

	// Find nearest neighbors
	kNeighborSoln neighborSearch;
	int num_neighbors = ms.Knearest[node_index].size() + 1;

	kNeighbors(KDtree, ms.milestones[node_index], neighborSearch, num_neighbors, true); // Might consider increasing k

	int new_neighbor_index = neighborSearch.neighbors[neighborSearch.neighbors.size()-1];
	double new_neighbor_dist = neighborSearch.dist[neighborSearch.neighbors.size()-1];

	ms.Knearest[node_index].push_back(new_neighbor_index);
	ms.Kdistances[node_index].push_back(new_neighbor_dist);
	ms.Knearest[new_neighbor_index].push_back(node_index);
	ms.Kdistances[new_neighbor_index].push_back(new_neighbor_dist);

	update_all_paths_w_new_neighbor(node_index, new_neighbor_index, new_neighbor_dist);

	// ----- Compute sub-ms -----
	rodSoln current_solution;

	// for each set of milestones and neighbor, find and store soln
	get_solution(current_solution, ms.milestones[node_index], ms.milestones[new_neighbor_index]);
	current_solution.parents = {node_index, new_neighbor_index};

	if (current_solution.a.size()==0) {
		ms.first_sub_neighbor[node_index].push_back(new_neighbor_index);
		ms.first_sub_indicator[node_index].push_back(1);
		ms.first_sub_neighbor[new_neighbor_index].push_back(node_index);
		ms.first_sub_indicator[new_neighbor_index].push_back(1);
		return;
	}

	ms.first_sub_neighbor[node_index].push_back((int)subms.sub_milestones.size());
	ms.first_sub_indicator[node_index].push_back(0);

	if (current_solution.a.size() == 1) { // If there is only one node
		subms.sub_milestones.push_back(current_solution.a[0]);
		subms.T_end.push_back(current_solution.T_end[0]);
		subms.P.push_back(current_solution.P[0]);
		subms.s.push_back(current_solution.s[0]);
		subms.neighbor_indicator.push_back({1, 1});
		subms.neighbors2.push_back({node_index, new_neighbor_index});
		subms.milestones_parents.push_back(current_solution.parents);
	}
	else
		for(int q = 0; q < current_solution.a.size(); q++) {

			subms.sub_milestones.push_back(current_solution.a[q]);
			subms.T_end.push_back(current_solution.T_end[q]);
			subms.P.push_back(current_solution.P[q]);
			subms.s.push_back(current_solution.s[q]);
			subms.milestones_parents.push_back(current_solution.parents);
			if (q==0) {
				subms.neighbor_indicator.push_back({1, 0});
				subms.neighbors2.push_back({node_index, (int)subms.sub_milestones.size()});
			}
			else if (q==current_solution.a.size()-1) {
				subms.neighbor_indicator.push_back({0, 1});
				subms.neighbors2.push_back({(int)(subms.sub_milestones.size()-2), new_neighbor_index});
			}
			else {
				subms.neighbor_indicator.push_back({0, 0});
				subms.neighbors2.push_back({(int)(subms.sub_milestones.size()-2), (int)subms.sub_milestones.size()});
			}
		}

	ms.first_sub_neighbor[new_neighbor_index].push_back((int)subms.sub_milestones.size()-1);
	ms.first_sub_indicator[new_neighbor_index].push_back(0);

	add_startNgoal_runtime += double(clock() - startTime) / CLOCKS_PER_SEC;

}

bool PRMGenerator::check_path_continuous() {
	// Returns true if the outputted a-path is continuous - for debugging
	Matrix A;
	for (int i; i < path.size(); i++)
		A.push_back(path[i].a);

	bool flag = true;

	for (int i = 1; i < A.size(); i++) {
		for (int j = 0; j < A[0].size(); j++) {
			if (fabs(A[i][j]-A[i-1][j]) > 2) {
				flag = false;
				break;
			}
		}
	}

	if (!flag)
		printMatrix(A);

	return flag;
}

// ------------------------------ STORE DATA --------------------------

void PRMGenerator::save_roadmap_to_file(){

	cout << "Saving roadmap to txt files..." << endl;

	std::string pathSoln("./data/soln.txt");
	std::string pathStones("./data/milestones.txt");
	std::string pathDist("./data/distances.txt");
	std::string pathNear("./data/nearest.txt");
	std::string pathDistGraph("./data/dis_graph.txt");
	std::string pathParentGraph("./data/parent_graph.txt");
	std::string pathSubStones("./data/sub_milestones.txt");

	std::ofstream FILE(pathStones);//, std::ofstream::app);
	for(int i=0;i!=ms.milestones.size();i++){
		std::copy(ms.milestones[i].begin(), ms.milestones[i].end(), std::ostream_iterator<double>(FILE, " "));
		FILE << "\n";
	}
	FILE.close();

	std::ofstream FILE3(pathDist);//, std::ofstream::app);
	for(int i=0;i!=ms.Kdistances.size();i++){
		std::copy(ms.Kdistances[i].begin(), ms.Kdistances[i].end(),	std::ostream_iterator<double>(FILE3, " "));
		FILE3 << "\n";
	}
	FILE3.close();

	std::ofstream FILE4(pathNear);//, std::ofstream::app);
	for(int i=0;i!=ms.Knearest.size();i++){
		std::copy(ms.Knearest[i].begin(), ms.Knearest[i].end(),	std::ostream_iterator<double>(FILE4, " "));
		FILE4 << "\n";
	}
	FILE4.close();

	std::ofstream FILE5(pathDistGraph);//, std::ofstream::app);
	for (int i=0;i!=ms.dis_graph.size();i++){
		std::copy(ms.dis_graph[i].begin(), ms.dis_graph[i].end(), std::ostream_iterator<double>(FILE5, " "));
		FILE5 << "\n";
	}
	FILE5.close();

	std::ofstream FILE6(pathParentGraph);//, std::ofstream::app);
	for (int i=0;i!=ms.parent_graph.size();i++){
		std::copy(ms.parent_graph[i].begin(), ms.parent_graph[i].end(), std::ostream_iterator<int>(FILE6, " "));
		FILE6 << "\n";
	}
	FILE6.close();

	std::ofstream FILE7(pathSubStones);//, std::ofstream::app);
	for (int i=0;i!=subms.sub_milestones.size();i++){
		std::copy(subms.sub_milestones[i].begin(), subms.sub_milestones[i].end(), std::ostream_iterator<double>(FILE7, " "));
		FILE7 << "\n";
	}
	FILE7.close();

	cout << "Roadmap saved to text files." << endl;
}

void PRMGenerator::save_data() {
	ofstream ofs;
	if (!is2Dor6D)
		ofs.open("./data/ms2D.prm", ios::binary);
	else {
		string file = "./data/ms6D_" + to_string((int)num_pts) + "_" + to_string(k) + ".prm";
		const char *y = file.c_str();
		ofs.open(y, ios::binary);
	}
	size_t sz, lsize;

	// ------------------- ms struct -------------------------
	// Save milestones
	sz = ms.milestones.size();
	ofs.write((char*)&sz, sizeof(sz));
	lsize = ms.milestones[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&ms.milestones[i][0], lsize * sizeof(double) );
	}
	// Save Knearest
	for (int i = 0; i < sz; i++) {
		lsize = ms.Knearest[i].size();
		ofs.write((char*)&lsize, sizeof(lsize));
		ofs.write((char*)&ms.Knearest[i][0], lsize * sizeof(int) );
	}
	// Save Kdistances
	for (int i = 0; i < sz; i++) {
		lsize = ms.Kdistances[i].size();
		ofs.write((char*)&lsize, sizeof(lsize));
		ofs.write((char*)&ms.Kdistances[i][0], lsize * sizeof(double) );
	}
	// Save first_sub_neighbor
	for (int i = 0; i < sz; i++) {
		lsize = ms.first_sub_neighbor[i].size();
		ofs.write((char*)&lsize, sizeof(lsize));
		ofs.write((char*)&ms.first_sub_neighbor[i][0], lsize * sizeof(int) );
	}
	// Save first_sub_indicator
	for (int i = 0; i < sz; i++) {
		lsize = ms.first_sub_indicator[i].size();
		ofs.write((char*)&lsize, sizeof(lsize));
		ofs.write((char*)&ms.first_sub_indicator[i][0], lsize * sizeof(int) );
	}
	// Save dis_graph
	sz = ms.dis_graph.size();
	ofs.write((char*)&sz, sizeof(sz));
	lsize = ms.dis_graph[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&ms.dis_graph[i][0], lsize * sizeof(double) );
	}
	// Save parent_graph
	sz = ms.parent_graph.size();
	ofs.write((char*)&sz, sizeof(sz));
	lsize = ms.parent_graph[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&ms.parent_graph[i][0], lsize * sizeof(int) );
	}
	// Save T_end
	sz = ms.T_end.size();
	ofs.write((char*)&sz, sizeof(sz));
	lsize = 4;
	for (int i = 0; i < sz; i++) {
		for (int j = 0; j < lsize; j++)
			ofs.write((char*)&ms.T_end[i][j][0], lsize * sizeof(double) );
	}
	// Save P
	sz = ms.P.size();
	ofs.write((char*)&sz, sizeof(sz));
	for (int i = 0; i < sz; i++) {
		lsize = ms.P[i].size();
		ofs.write((char*)&lsize, sizeof(lsize));
		for (int j = 0; j < lsize; j++)
			ofs.write((char*)&ms.P[i][j][0], 3 * sizeof(double) );
	}
	// Save s
	sz = ms.s.size();
	ofs.write((char*)&sz, sizeof(sz));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&ms.s[i], sizeof(int) );
	}

	// ------------------- subms struct -------------------------
	// Save milestones
	sz = subms.sub_milestones.size();
	ofs.write((char*)&sz, sizeof(sz));
	lsize = subms.sub_milestones[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&subms.sub_milestones[i][0], lsize * sizeof(double) );
	}
	// Save neighbors2
	lsize = subms.neighbors2[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&subms.neighbors2[i][0], lsize * sizeof(int) );
	}
	// Save milestones_parents
	lsize = subms.milestones_parents[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&subms.milestones_parents[i][0], lsize * sizeof(int) );
	}
	// Save neighbor_indicator
	lsize = subms.neighbor_indicator[0].size();
	ofs.write((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&subms.neighbor_indicator[i][0], lsize * sizeof(int) );
	}
	// Save T_end
	lsize = 4;
	for (int i = 0; i < sz; i++) {
		for (int j = 0; j < lsize; j++)
			ofs.write((char*)&subms.T_end[i][j][0], lsize * sizeof(double) );
	}
	// Save P
	for (int i = 0; i < sz; i++) {
		lsize = subms.P[i].size();
		ofs.write((char*)&lsize, sizeof(lsize));
		for (int j = 0; j < lsize; j++)
			ofs.write((char*)&subms.P[i][j][0], 3 * sizeof(double) );
	}
	// Save s
	for (int i = 0; i < sz; i++) {
		ofs.write((char*)&subms.s[i], sizeof(int) );
	}

	ofs.close();

	cout << "Roadmap serialized." << endl;
}

void PRMGenerator::load_data(string filename) {

	cout << "Loading roadmap from file " << filename << " ..." << endl;

	ifstream ifs;
	if (!is2Dor6D)
		ifs.open("./data/ms2D.prm", ios::binary);
	else
		ifs.open("../roadmap/data/" + filename, ios::binary);
	if (!ifs.is_open()) {
		cout << "Error opening file name: " << filename << endl;
		exit(1);
	}

	size_t sz, lsize;

	// ------------------- ms struct -------------------------

	ms.milestones.clear();
	ms.Knearest.clear();
	ms.first_sub_neighbor.clear();
	ms.first_sub_indicator.clear();
	ms.dis_graph.clear();
	ms.parent_graph.clear();
	ms.T_end.clear();
	ms.P.clear();
	ms.s.clear();

	// Load milestones
	ifs.read((char*)&sz, sizeof(sz));
	ms.milestones.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ms.milestones[i].resize(lsize);
		ifs.read((char*)&ms.milestones[i][0], lsize * sizeof(double));
	}
	// Load Knearest
	ms.Knearest.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&lsize, sizeof(lsize));
		ms.Knearest[i].resize(lsize);
		ifs.read((char*)&ms.Knearest[i][0], lsize * sizeof(int));
	}
	// Load Kdistances
	ms.Kdistances.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&lsize, sizeof(lsize));
		ms.Kdistances[i].resize(lsize);
		ifs.read((char*)&ms.Kdistances[i][0], lsize * sizeof(double));
	}
	// Load first_sub_neighbor
	ms.first_sub_neighbor.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&lsize, sizeof(lsize));
		ms.first_sub_neighbor[i].resize(lsize);
		ifs.read((char*)&ms.first_sub_neighbor[i][0], lsize * sizeof(int));
	}
	// Load first_sub_indicator
	ms.first_sub_indicator.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&lsize, sizeof(lsize));
		ms.first_sub_indicator[i].resize(lsize);
		ifs.read((char*)&ms.first_sub_indicator[i][0], lsize * sizeof(int));
	}
	// Load dis_graph
	ifs.read((char*)&sz, sizeof(sz));
	ms.dis_graph.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ms.dis_graph[i].resize(lsize);
		ifs.read((char*)&ms.dis_graph[i][0], lsize * sizeof(double));
	}
	// Load parent_graph
	ifs.read((char*)&sz, sizeof(sz));
	ms.parent_graph.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		ms.parent_graph[i].resize(lsize);
		ifs.read((char*)&ms.parent_graph[i][0], lsize * sizeof(int));
	}
	// Load T_end
	ifs.read((char*)&sz, sizeof(sz));
	ms.T_end.resize( sz );
	lsize = 4;
	for (int i = 0; i < sz; i++) {
		ms.T_end[i].resize(lsize);
		for (int j = 0; j < lsize; j++) {
			ms.T_end[i][j].resize(lsize);
			ifs.read((char*)&ms.T_end[i][j][0], lsize * sizeof(double) );
		}
	}
	// Load P
	ifs.read((char*)&sz, sizeof(sz));
	ms.P.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&lsize, sizeof(lsize));
		ms.P[i].resize(lsize);
		for (int j = 0; j < lsize; j++) {
			ms.P[i][j].resize(3);
			ifs.read((char*)&ms.P[i][j][0], 3 * sizeof(double) );
		}
	}
	// Load s
	ifs.read((char*)&sz, sizeof(sz));
	ms.s.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&ms.s[i], sizeof(int));
	}

	// ------------------- subms struct -------------------------
	subms.sub_milestones.clear();
	subms.neighbors2.clear();
	subms.milestones_parents.clear();
	subms.neighbor_indicator.clear();
	subms.T_end.clear();
	subms.P.clear();
	subms.s.clear();

	// Load milestones
	ifs.read((char*)&sz, sizeof(sz));
	subms.sub_milestones.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		subms.sub_milestones[i].resize(lsize);
		ifs.read((char*)&subms.sub_milestones[i][0], lsize * sizeof(double));
	}
	// Load neighbors2
	subms.neighbors2.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		subms.neighbors2[i].resize(lsize);
		ifs.read((char*)&subms.neighbors2[i][0], lsize * sizeof(int));
	}
	// Load milestones_parents
	subms.milestones_parents.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		subms.milestones_parents[i].resize(lsize);
		ifs.read((char*)&subms.milestones_parents[i][0], lsize * sizeof(int));
	}
	// Load neighbor_indicator
	subms.neighbor_indicator.resize( sz );
	ifs.read((char*)&lsize, sizeof(lsize));
	for (int i = 0; i < sz; i++) {
		subms.neighbor_indicator[i].resize(lsize);
		ifs.read((char*)&subms.neighbor_indicator[i][0], lsize * sizeof(int));
	}
	// Load T_end
	subms.T_end.resize( sz );
	lsize = 4;
	for (int i = 0; i < sz; i++) {
		subms.T_end[i].resize(lsize);
		for (int j = 0; j < lsize; j++) {
			subms.T_end[i][j].resize(lsize);
			ifs.read((char*)&subms.T_end[i][j][0], lsize * sizeof(double) );
		}
	}
	// Load P
	subms.P.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&lsize, sizeof(lsize));
		subms.P[i].resize(lsize);
		for (int j = 0; j < lsize; j++) {
			subms.P[i][j].resize(3);
			ifs.read((char*)&subms.P[i][j][0], 3 * sizeof(double) );
		}
	}
	// Load s
	subms.s.resize( sz );
	for (int i = 0; i < sz; i++) {
		ifs.read((char*)&subms.s[i], sizeof(int));
	}

	ifs.close();

	cout << "Roadmap loaded." << endl;
}

// ------------------------------ STORE DATA --------------------------


