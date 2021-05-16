#include "Global.hpp"

#include <unistd.h>
#include <string.h>
#include <signal.h>

const size_t  MAX_FNAME_LEN      = 255;
const size_t  MAX_TC_COUNT       = 1;
const size_t  MAX_INTERVAL_CNT   = 110;
const size_t  MAX_TIME_CNT   = 1201;

struct perfPair
{
  double optimality;
  double reqTime;

  perfPair();
  perfPair(double op, double t);
  perfPair(const perfPair &p);

  perfPair& operator=(const perfPair& p);
  void Print();
};


perfPair::perfPair()
  :
    optimality(0),
    reqTime(0)
{}


perfPair::perfPair(double op, double t)
  :
    optimality(op),
    reqTime(t)
{}


perfPair::perfPair(const perfPair &p)
  :
    optimality(p.optimality),
    reqTime(p.reqTime)
{}


perfPair& perfPair::operator=(const perfPair &p)
{
  optimality = p.optimality;
  reqTime = p.reqTime;

  return (*this);
}


void perfPair::Print()
{
  cout << "<";
  cout << setiosflags(ios::fixed) << setiosflags(ios::right)
       << setw(DBL_PRINT_WIDTH) << setprecision(DBL_PRINT_PRECISION)
       << optimality;
  cout << ",";
  cout << setiosflags(ios::fixed) << setiosflags(ios::right)
       << setw(DBL_PRINT_WIDTH) << setprecision(DBL_PRINT_PRECISION)
       << reqTime;
  cout << ">";
}


int main(int argc, char *argv[])
{
  string    fName;
  if ( argc < 3 )
  {
    cerr << "Usage: "<< argv[0] 
         << " -file fileName"
         << endl;

    return 1;
  }
  int num = 3;
  //~ int maxDim = 12;
  for (int i = 1; i < argc; )
  {
	num++;
    if (strcmp(argv[i],"-file") == 0) 
    {
	  if (strlen (argv[i+1]) > (MAX_FNAME_LEN - 1)) 
      {
	    cerr << "Too long argument" << endl;
		return 1;
	  }
      fName =  argv[i+1];
	  i += 2;
	}
	else 
    {
      cerr << "Unknown parameter " << argv[i] << " passed" << endl; 
	  i++;
	}
  
 //~ cout<<"\n\n\n"<<fName.c_str()<<endl;
  struct stat fileStat;
  int retVal = stat(fName.c_str(), &fileStat);
  if(retVal == -1)
  {
    cerr << "From : " << argv[0] 
         << " Error in accessing file : " 
         << strerror(errno) << endl;
    exit(EXIT_FAILURE);
  }
  
  vector<double> time1(MAX_INTERVAL_CNT, 0.0);     //to measure the avg time required to obtain a particular optimality
  vector<size_t> tupleCountTime1(MAX_INTERVAL_CNT, 0);  //number of time intervals producing that particular optimality
  
  vector<double> optimality(MAX_TIME_CNT, 0.0);  //to measure the avg %tage within a particular time
  ifstream fin;
  fin.open(fName.c_str());

  ofstream foutTime, foutOpt;
  stringstream ss;
  ss << num ; 
  string str = ss.str();
  string foutNameTime("./plots/BMSDataForPlotAccTime"+str+".out");
  string foutNameOpt("./plots/BMSDataForPlotAccOptimality"+str+".out");
  foutTime.open(foutNameTime.c_str());
  foutOpt.open(foutNameOpt.c_str());
  
  double tpOld(0.0);
  int timeIndex(0);	  	  
  perfPair tp;
  fin >> tp.optimality;
  fin >> tp.reqTime;
  
  if(fin.is_open())
  {
	tpOld = 0;
	
	while(fin.good()) 
	{
		tpOld = 0;
		timeIndex = 0;
		cout<<endl<<endl;
		while(tp.optimality>=tpOld && fin.good())
		{
			if(tp.reqTime > 0) 
			{			
			  tupleCountTime1[tp.optimality]++;
              time1[tp.optimality] += tp.reqTime;
			
			  if((int)tp.reqTime != timeIndex) 
		      {
			    for(int i = timeIndex; i<(int)tp.reqTime; i++) 
			    {
				  optimality[i] += tpOld;
				  //~ if(i%10==0)
				  //cout<<"\t"<<i<<"\t"<<tpOld<<endl;
			    }
			    timeIndex = (int)tp.reqTime;
		      }		    		    
		      tpOld = tp.optimality; 
          
		    }
		    //~ cout<<"here"<<endl;
		    fin >> tp.optimality;
            fin >> tp.reqTime;  
		}
		//~ cout<<tp.optimality<<"\t"<<tpOld<<endl;
		for(int i = timeIndex; i<MAX_TIME_CNT; i++) 
		{
			optimality[i] += tpOld;
			//~ cout<<"\t"<<i<<"\t"<<tpOld<<endl;
		} 
	}
    fin.close();
  }
  else
  {
    exit(EXIT_FAILURE);
  }
  
  cout<<"\\begin{table}[!ht]\n\\centering\n\\label{table:grid-bms-"<<num<<"}\n\\caption{\\small Avg Time Required to achieve the corresponding quality of solution for multi-objective grid of size $"<<num<<"\\times "<<num<<"$ using Beam Stack}\n\\begin{tabular}{|l|l|l|l|l|l|l|l|}\n\\hline\n\\multicolumn{8}{|c|}{Solution Quality (in \\%) Vs Avg Time Required (in sec)}\\\\\n"<<endl;
  int count(0);
  	double timeD;
	int opt;
  for(int j = 0; j < MAX_INTERVAL_CNT; j++)
  { 

    if(tupleCountTime1[j] > 0)
    {
      opt = j;
      timeD = time1[j]/tupleCountTime1[j];

      cout << setiosflags(ios::fixed) << setiosflags(ios::right)
           << setw(7) << opt;
      cout << "\\%";
      cout << "   & ";
      cout<<"\t";
      cout << setiosflags(ios::fixed) << setiosflags(ios::right)
           << setprecision(3) << setw(7) << timeD;

      foutTime << setiosflags(ios::fixed) << setiosflags(ios::right)
           << setw(7) << opt;
      foutTime<<"\t";
      foutTime << setiosflags(ios::fixed) << setiosflags(ios::right)
           << setprecision(3) << setw(7) << timeD<<endl;
      count++;
      if(count < 4)
      {     
        cout << "   & ";
	  }
	  else 
	  {
	    cout<<"  \\\\\n";
	    count = 0;
	  }
    }
  }
  if(count<4 && count !=0) {
  count = 2*count;
  while(count<7) 
  {
	cout<<" "<<"  &  ";
    count+=1;
  }
  cout<<" \\\\\n";
  }
  foutTime<<opt<<"\t 1200"<<endl;
    cout<<"\\hline\n\\end{tabular}\n\\end{table}\n"<<endl; 
  for(int i = 0; i<MAX_TIME_CNT; i++) 
  {
	double opt = optimality[i]/MAX_TC_COUNT;
    foutOpt << setiosflags(ios::fixed) << setiosflags(ios::right)
           << setprecision(3) << setw(7) << opt<<"\t"<<i<<endl;
           
           if(opt>100) {
		   cout<<"error "<<num<<" opt = "<<opt<<endl;
		   }
    //~ cout << setiosflags(ios::fixed) << setiosflags(ios::right)
           //~ << setprecision(3) << setw(7) << opt<<"\t"<<i<<endl;
  }
  foutTime.close();
  foutOpt.close();

  
}
  return 0;
}
