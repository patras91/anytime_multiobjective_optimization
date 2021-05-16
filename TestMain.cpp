#include "Global.hpp"
#include "CostVec.hpp"
#include "Grid.hpp"
#include "GridSearch.hpp"

#include <unistd.h>
#include <string.h>
#include <signal.h>


int main(int argc, char *argv[])
{
  if(argc < 7)
  {
    cerr << "Usage : " << argv[0] 
         << " xDimMax yDimMax edgeCost stepSize choice(R/W) filename" << endl;
    exit(0);
  }

  size_t xMax(0);
  size_t yMax(0);
  size_t stepSize(DEFAULT_L1_INC);
  double maxEC(EDGE_COST_MAX);
  int    choice;

  xMax = (size_t) atoi(argv[1]);
  yMax = (size_t) atoi(argv[2]);

  if(argc >= 4)
  {
    maxEC = atof(argv[3]);
  }
  
  if(argc >= 5)
  {
    stepSize = atoi(argv[4]);
  }

  if(argc >= 6)
  {
    choice = argv[5][0];
  }

  /*
  grid testGW(xMax, yMax, maxEC);

  testGW.ConstructGrid();
  testGW.Print();

  ofstream fout;
  string fName = argv[7];
  fout.open(fName.c_str());
  testGW.WriteToFile(fout);
  fout.close();

  grid testGR;
  ifstream fin;
  fin.open(fName.c_str());
  testGR.ReadFromFile(fin);
  fin.close();
  testGR.Print();
  */

  if(('W' == choice) || ('w' == choice))
  {
    gridSearch myGridSearchObjW(xMax, yMax, maxEC);

    myGridSearchObjW.ConstructGrid();
    //myGridSearchObjW.PrintProblemInstance();

    myGridSearchObjW.MOASearchL1Ordered();

    ofstream fout;
    fout.open(argv[6]);
    myGridSearchObjW.WriteToFile(fout);
    fout.close();
  }
  else if(('R' == choice) || ('r' == choice))
  {
    struct stat fileStat;
    int retVal = stat(argv[6], &fileStat);
    if(retVal == -1)
    {
      cerr << "Error in accessing file : " << strerror(errno) << endl;
      exit(EXIT_FAILURE);
    }
    
    gridSearch myGridSearchObjR;
    ifstream fin;
    fin.open(argv[6]);
    myGridSearchObjR.ReadFromFile(fin);
    fin.close();

    myGridSearchObjR.ComputeOptSFHV();
    myGridSearchObjR.SearchContract(stepSize);
  }
  else
  {
    cerr << "Invalid choice" << endl;
  }
  
  return 0;
}
