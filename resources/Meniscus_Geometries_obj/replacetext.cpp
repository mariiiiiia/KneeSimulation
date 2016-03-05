#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(){
  filebuf inf, outf;
  outf.open("Femur_Med_vertices.txt", std::ios::out);
  inf.open("Femur_Med_vert.txt", std::ios::in);
  ostream outFile(&outf);
  istream readFile(&inf);
  char readout;
  char search = 'v';
  int replace = 14475;
  printf("hello world!\n");
  while(readFile.get(readout)){
    // printf("i just got character: %c", readout);
    if(readout == search){
      printf("i found searched character %c and i replace it with %c\n", readout, replace);
      outFile << replace;
      replace++;
    }
    else {
      outFile << readout;
    }
  }
}
