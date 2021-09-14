#include <iostream>
#include "DT5742.h"

int main(int argc, char **argv) {
  int nevts = 25000;//atoi(argv[1]);
  //int nevts = 10000;//atoi(argv[1]);
  std::cout << "***********************************************" << std::endl;
  std::cout << "***********************************************" << std::endl;
  std::cout << "***********************************************" << std::endl;
  std::cout << " START ACQUISITION: AIM " << nevts << " EVENTS " << std::endl;
  std::cout << "***********************************************" << std::endl;
  std::cout << "***********************************************" << std::endl;
  DT5742 myDRS;
  myDRS.Init();
  myDRS.Read(nevts);
  myDRS.Close();  
  return 0; 
}
