#include <stdlib.h>
#include <iostream>
#include <vector>
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

using namespace std;
using namespace nanoflann;
typedef vector<vector < double > > Matrix;

vector<int> kNeighbors(int& i, vector<vector<double> >& configs){
  // find nearest neighbors for node i in configs, which are 6-D As.
    //FOR TEST WE DO 1D FOR EASE 

        // REMOVE CREATION ELSEWHERE, DONT WANT TO MAKE NEXT EVERY TIME

        //build tree
        typedef KDTreeVectorOfVectorsAdaptor<Matrix, double >  my_kd_tree_t;
        my_kd_tree_t   mat_index(1, configs, 10 ); //10 is leaf size
        mat_index.index->buildIndex();


        // do a knn search
        const size_t num_results = 3; //see if can do int SET TO k=2;
        vector<size_t>   ret_indexes(num_results); //see if can do int
        vector<double> out_dists_sqr(num_results);

        nanoflann::KNNResultSet<double> resultSet(num_results);

        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        vector<double> query_pnt = configs[i];
        mat_index.index->findNeighbors(resultSet, &query_pnt[0],SearchParams(10));
  
  //print squared distances
  cout << out_dists_sqr[0] << endl;
  cout << out_dists_sqr[1] << endl;
  cout << out_dists_sqr[2] << endl;
  
  vector<int> rtn_values(ret_indexes.begin(), ret_indexes.end());
  rtn_values.erase(rtn_values.begin());
  return rtn_values;
}


int main(){

vector<vector< double > > configs;
vector<int> neighbors;
//populate configs
vector<double> node0 = {0};
vector<double> node1 = {5};
vector<double> node2 = {7};
vector<double> node3 = {8};
vector<double> node4 = {15};

configs.push_back(node0);
configs.push_back(node1);
configs.push_back(node2);
configs.push_back(node3);
configs.push_back(node4);

for (int i=0;i!=configs.size();i++){

  neighbors = kNeighbors(i, configs);
  cout << "node"<<i <<" neighors are: " << neighbors[0] << " and " << neighbors[1] << endl;
  //expected results: 0:1,2  1:2,3  2:3,1  3:2,1  4:3,2
  //   distances^2: 0,25,49  0,4,9  0,1,4  0,1,9  0,49,64
  
}

}
