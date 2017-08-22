#include <stdlib.h>
#include <iostream>
#include <vector>

using namespace std;

vector<double> random_configuration(int& DOF){
  //random vector of length DOF <= +/-30
  vector<double> config(DOF);

  for (int d=0; d<DOF; d++){
    config[d] = (rand() % 60) - 30;
  }
  return config;
}


int main(){
  cout << "No output is expected result" << endl;
  int DOF = 6;
  vector<double> r;
  for(int i=0;i<1000;i++){
    r = random_configuration(DOF);
    for(int j=0;j!=6;j++){
      if(r[j] > 30 || r[j] < -30){
      cout << r[j] << endl;
      }
    }
  }
}
