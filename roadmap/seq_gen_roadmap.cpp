// Compile with:
// g++ seq_gen_roadmap.cpp -o sgen -std=c++11


// Standard libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <stdio.h>

//typedef vector<int> Vector;

int main() {
	
	int N[] = {20};
	int K[] = {4};

	for (int i = 0; i < 1; i++) {
		std::cout << "Computing roadmap with " << N[i] << " milestones and " << K[i] << " neighbors." << std::endl;
		// run trial
		std::string com = "./gen " + std::to_string(N[i]) + " " + std::to_string(K[i]) + " 1";
		const char *y = com.c_str();
		system(y);
	}
	
	return 0;
}
