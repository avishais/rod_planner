#include <stdlib.h>
#include <iostream>

using namespace std;

int get_random_milestone(){
  //returns index of random milestone 
  int rtn = rand() % 800-1;
  return rtn;
}


int main(){
  cout << "expected output is nothing" << endl;
  int r;
  
  for(int i=0;i!=1000;i++){
    r = get_random_milestone();
    if(r > 1000 || r < 0){
      cout << r << endl;
    }
  
  }
}
