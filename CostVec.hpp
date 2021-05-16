#ifndef __COSTVEC__
#define __COSTVEC__

#include "Global.hpp"


struct costVec
{
  costVec();
  costVec(const costVec &cv);

  costVec&       operator=(const costVec &cv);
  costVec&       operator+=(const costVec &cv);
  const costVec  operator+(const costVec &cv);

  // Denotes the comparison in L1 order 
  bool           IsLessL1(const costVec &cv) const;
  
  // Denotes the comparison in L2 order 
  bool           IsLessL2(const costVec &cv) const;
  
  bool           operator==(const costVec &cv) const;

  bool           operator<(const costVec &cv) const;
  bool           operator<=(const costVec &cv) const;
  double         GetCostComponent(size_t cid) const;
  double         GetCostComponentInverse(size_t cid) const;
  void           SetCostComponent(size_t cid, double val);
  void           Print();
  void           WriteToFile(ofstream &fout);
  void           ReadFromFile(ifstream &fin);
  bool   		 Dominates(costVec cv, size_t k);


  private:
    double      c[VEC_DIMENSION];
};


#endif //__COSTVEC__
