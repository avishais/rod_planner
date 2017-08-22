// Uses PRMGenerator class to generate and save roadmap

// Run with ./<exefile> <number-of-milestones> <k-nearest-neighbors> <1/0-save-to-file>

#include "prm_gen.hpp"

int main(int argc, char** argv){
	int num_pts, k;
	bool saveFile;

	if (argc >= 4){
		num_pts = atof(argv[1]);
		k = atoi(argv[2]);
		saveFile = atoi(argv[3]);
	}
	else {
		cout << "Defining default inputs." << endl;
		num_pts = 10;
		k = 2;
		saveFile = 1;
	}

	srand (time(NULL));

	// generating a Prob. Roadmap
	PRMGenerator PRMGraph;

	PRMGraph.set_roadmap_parameters(num_pts, k, saveFile);

	// -------------------------
	clock_t st = clock();

	PRMGraph.generate_roadmap();
	//PRMGraph.load_data("ms6D_2000_2.prm");
	//PRMGraph.load_data("ms2D.prm");

	clock_t et = clock();
	cout << "Generated roadmap with total of " <<  PRMGraph.get_num_tot_milestones() << " nodes in " << (double)(et-st)/CLOCKS_PER_SEC << " seconds." << endl;
	// -------------------------


	if (PRMGraph.check_1_component())
		cout << "There is one component." << endl;
	else
		cout << "There is more than one component." << endl;
/*

	//clock_t st = clock();
	//PRMGraph.add_start_n_goal_milestones(PRMGraph.random_configuration(), PRMGraph.random_configuration(), 1);
	PRMGraph.add_start_n_goal_milestones({-0.4,-2.507}, {-5.105,8.519}, 1);
	//clock_t et = clock();
	//cout << "KDtree time: " << (double)(et-st)/CLOCKS_PER_SEC << "s." << endl;

	PRMGraph.add_neighbor_to_node(10);
	cout << "Added new neighbor\n";

	PRMGraph.query_path();

	PRMGraph.save_roadmap_to_file();*/

	return 0;
}
