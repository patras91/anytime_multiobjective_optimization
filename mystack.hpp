#ifndef __MYSTACK__
#define __MYSTACK__

#include "CostVec.hpp"
#include "Grid.hpp"

struct openEntry
{
  size_t   nodeIndex;
  size_t   pathIndex;
  costVec  gValue;
  costVec  fValue;

  openEntry();
  openEntry(size_t nIdx, size_t pIdx) { nodeIndex = nIdx; pathIndex = pIdx; } 
  openEntry(const openEntry &t);
  openEntry(size_t nIdx, size_t pIdx, costVec gVal, costVec fVal);
  bool operator==(const openEntry &oe) const;
  bool operator<(const openEntry &oe) const;
  void Print();
  
};

class gridSearch;

class mystackComp {
  public:
   int level;
   int d;
   
   virtual void add(openEntry open, int level) { cout << "Virtual add"; }
   virtual void add(openEntry open) { cout << "Virtual add"<<endl; }
   virtual void print() {
     cout << "Virtual print"<<endl;
   }
   virtual void InitializeContract(int contract) { cout << "Virtual initialize contract"<<endl; }
   virtual void AdjustContract(int contract) {cout << "Virtual adjust contract"<<endl;}
   virtual bool IsEmpty(int sIdx) { cout << "Virtual IsEmpty(sIdx) "<<endl;return true; }
   virtual bool IsEmpty() { cout << "Virtual IsEmpty()"<<endl;return true; }
   virtual void AnytimeContractSearchStack(gridSearch &GS, size_t sIdx) {cout << "Virtual ACTRSS(sIdx)"<<endl;}
   virtual size_t AnytimeContractSearchStack(gridSearch &GS) { cout << "Virtual ACTRSS()"<<endl;return 0; }
   virtual openEntry GetFront() { openEntry open; cout << "Virtual GetFront()"<<endl;return open; }
   virtual list<openEntry> GetList() { list<openEntry> op; cout << "Virtual GetList()"<<endl;return op; }
   virtual list<openEntry>::iterator GetIterator() { list<openEntry>::iterator it; cout << "Virtual GetIterator()"<<endl;return it;}
   virtual list<openEntry>::iterator GetEnd() { list<openEntry>::iterator it; cout << "Virtual GetEnd()"<<endl;return it;}
   virtual void remove(openEntry open) { cout << "Virtual remove()"<<endl; }
   virtual void Prune(gridSearch& GS, openEntry curSol, size_t sIdxCur) {cout << "Virtual Prune()"<<endl;}
   virtual void PruneRec(gridSearch& GS, openEntry curSol) {cout << "Virtual PruneRec()"<<endl;}
   virtual list<openEntry>::iterator Erase(list<openEntry>::iterator it1) { list<openEntry>::iterator it;cout << "Virtual Erase()"<<endl; return it;}
   virtual void Normalize(vector<double> &r) {cout << "Virtual Normalize()"<<endl;}
};

class mylist: public mystackComp {
  public:
  list<openEntry> baseList;

  mylist(int n) {
    level = n;
  }
  
  void print() {
    //~ list<openEntry>::iterator it = baseList.begin();
    //~ while(it!=baseList.end()) {
      //~ cout << it->n << "\n";
      //~ it++;
    //~ }
  }
  
  void add(openEntry open) {
	 //~ cout << "pushing"<<endl;
    baseList.push_back(open);
    //~ cout << "size = "<<baseList.size()<<endl;
  }
  
  bool IsEmpty() {
    if (baseList.size() == 0) {
      //~ cout << "in IsEmpty() "<<endl;
      return true;
    } else {
      return false;
    }
  }
  
  openEntry GetFront() {
    return baseList.front();
  }
  
  list<openEntry> GetList() {
    return baseList;
  }
  
  list<openEntry>::iterator GetIterator() {
    return baseList.begin();
  }
  
  list<openEntry>::iterator GetEnd() {
    return baseList.end();
  }
  
  void remove(openEntry open) {
    baseList.remove(open);
  }
  
  list<openEntry>::iterator Erase(list<openEntry>::iterator it) {
    return (baseList.erase(it));
  }
};


class mystack: public mystackComp{
  public:
  vector<mystackComp*> 		sArr;
  vector<double> 			    rArr;
  vector<double> 			    expCount;
  vector<double> 			    lim;
  vector<bool> 				    suspend;
  size_t				 	        numRange;
  size_t						      depthBound;
  int 						        step;
  
  //~ size_t                  totalExpCount;
  
  mystack(int l, int s, grid& probInstance, int dB);
  
  int CalculateRange(openEntry open);
  
  void add(openEntry entry, int l);
  
  void InitializeContract(int contract);
  
  void AdjustContract(int contract);
  
  openEntry SelectBestFromAllLevels(bool &exist, size_t &level);
  
  void AnytimeContractSearchStack(gridSearch &GS, size_t sIdx);
  size_t AnytimeContractSearchStack(gridSearch &GS);
  void Prune(gridSearch& GS, openEntry curSol, size_t sIdxCur);
  void PruneRec(gridSearch& GS, openEntry curSol);
  bool IsEmpty(int sIdx);
  bool IsEmpty(); 
  void print();
  void Normalize(vector<double> &r); 
};

#endif // __MYSTACK__
