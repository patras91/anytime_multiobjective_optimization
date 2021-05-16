#include "Global.hpp"
#include "CostVec.hpp"
#include "Grid.hpp"
#include "GridSearch.hpp"

#include <unistd.h>
#include <string.h>
#include <signal.h>

#define  MAX_ARG_LEN        9
#define  MAX_FNAME_LEN      255
#define  TIME_OUT           10 
#define  MAX_DOUBLE_VAL     0.99
#define  MIN_DOUBLE_VAL     0.09 
#define  DEFAULT_KAPPA1_VAL 0.5

// Making this object global to access from the signal handler    
gridSearch myGridSearchObjR;

void sigIntHandler(int val)
{
  exit(0);
}


void sigIntHandlerParent(int val)
{
  myGridSearchObjR.PrintExecOut();
  exit(0);
}


int main(int argc, char *argv[])
{
  //~ cout << "IN MAIN\n";
  size_t    numDim(0);
  size_t    dimMax(0);
  
  double    maxEC(EDGE_COST_MAX);
  double    step(DEFAULT_L1_INC);

  int       choice(-1);
  string    fName;

  unsigned  timeOutParam(TIME_OUT);

  if ( argc < 15 )
  {
    cerr << "Usage: "<< argv[0] 
         << " -numDim val"
         << " -dimMax val"
         << " -edgeCostMax val"
         << " -timeOut [val in second]"
         << " -mode [R(Read)/W(Write)]" 
         << " -file fileName"
         << " -step stepSize"
         << endl;

    return 1;
  }

  
  for (int i = 1; i < argc; )
	{
		if (strcmp(argv[i],"-numDim") == 0)
    {
			if (strlen (argv[i+1]) > (MAX_ARG_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      numDim = (size_t)atoi(argv[i+1]);
			i += 2;
		}
    else if (strcmp(argv[i],"-dimMax") == 0)
    {
			if (strlen (argv[i+1]) > (MAX_ARG_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      dimMax = (size_t)atoi(argv[i+1]);
			i += 2;
		}
    else if (strcmp(argv[i],"-edgeCostMax") == 0) 
    {
			if (strlen (argv[i+1]) > (MAX_ARG_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      maxEC = atof(argv[i+1]);
			i+=2;
		}
    else if (strcmp(argv[i],"-step") == 0) 
    {
      // The number of disks for hannoi
			if (strlen (argv[i+1]) > (MAX_ARG_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      step = atof(argv[i+1]);
			i+=2;
		}
		else if (strcmp(argv[i],"-mode") == 0) 
    {
			if (strlen (argv[i+1]) > (MAX_ARG_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      choice =  argv[i+1][0];
      if(!(('W' == choice) || ('w' == choice) ||
           ('R' == choice) || ('r' == choice)))
      {
        cerr << "Invalid mode" << endl;
				return 1;
      }

			i += 2;
		}
		else if (strcmp(argv[i],"-file") == 0) 
    {
			if (strlen (argv[i+1]) > (MAX_FNAME_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      fName =  argv[i+1];
			i += 2;
		}
    else if (strcmp(argv[i],"-timeOut") == 0) 
    {
      // Degree of the non terminal node
			if (strlen (argv[i+1]) > (MAX_ARG_LEN - 1)) 
      {
				cerr << "Too long argument" << endl;
				return 1;
			}

      unsigned tmp = atoi(argv[i+1]);
      if(tmp > TIME_OUT)
      {
        timeOutParam = tmp;
      }

			i+=2;
		}
		else 
    {
      cerr << "Unknown parameter " << argv[i] << " passed" << endl; 
			i++;
		}
	}

  // For execution with a time limit
  volatile int pid = fork();
  
  if (pid == -1) 
  {
    cerr << "fork failed" << endl;
    exit(1);
  }
  else if (pid == 0)
  { 
    // Child process
    signal(SIGINT, sigIntHandler); 
    sleep(timeOutParam);

    // Timeout - informing the parent.    
    pid_t parentPID = getppid();

    kill(parentPID, SIGINT);
    return 0;  
  }
  else
  {
    //Parent process
    signal(SIGINT, sigIntHandlerParent);

    if(('W' == choice) || ('w' == choice))
    {
      // Using the microseconds to initialize the seed
      struct timeval tValSeed;
      gettimeofday(&tValSeed, NULL);
      
      //cout << "Initializing seed using : " << tValSeed.tv_usec << endl;

      srand48(tValSeed.tv_usec);
      
      gridSearch myGridSearchObjW(numDim, dimMax, maxEC);
   
      myGridSearchObjW.ConstructGrid();
      //myGridSearchObjW.PrintProblemInstance();
      //~ cout << " Constructed Grid \n";
      myGridSearchObjW.MOASearchL1Ordered();
      //~ cout << "Multi objective A star run \n";
      ofstream fout;
      fout.open(fName.c_str());
      myGridSearchObjW.WriteToFile(fout);
      fout.close();
      //~ cout << "Successfully written to file \n";
    }
    else if(('R' == choice) || ('r' == choice))
    {
      struct stat fileStat;
      //~ cout<<fName.c_str();
      int retVal = stat(fName.c_str(), &fileStat);
      if(retVal == -1)
      {
        cerr << "Error in accessing file : " << strerror(errno) << endl;
        exit(EXIT_FAILURE);
      }
      
      ifstream fin;
      fin.open(fName.c_str());
      myGridSearchObjR.ReadFromFile(fin);
      fin.close();
   
      myGridSearchObjR.ComputeOptSFHV();
      //~ cout<<"mine";
      myGridSearchObjR.SearchContract(step);
      myGridSearchObjR.PrintExecOut();
    }
    
    // pid holds id of child     
    // The execution of is finished. Informing the child.
    kill(pid, SIGINT);
    return 0;
  }
  
  return 0;
}
