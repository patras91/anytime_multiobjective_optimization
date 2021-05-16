#ifndef __GRID__
#define __GRID__

#include "Global.hpp"
#include "CostVec.hpp"


struct vertex
{
  size_t        idx;  // id of the vertex
  //~ size_t        xCd;  // Cd stands for coordinate
  //~ size_t        yCd;  // Cd stands for coordinate
  vector<size_t> Cds;
                
  bool          parents[MAX_DIM];  //whether parent exists in dimension i for i = 1 to MAX_DIM
  bool          children[MAX_DIM]; //whether children exists in dimension i for i = 1 to MAX_DIM
                
  costVec       edgeCostIn[MAX_DIM];  //incoming edge cost from ith dimension's parent for i = 1 to MAX_DIM
  costVec       edgeCostOut[MAX_DIM]; //outgoing edge cost to the child along ith dimension for i = 1 to MAX_DIM
                
  costVec       hVal;          //heuristic value of the vertex

  vertex();
  //~ vertex(size_t index, size_t x, size_t y);
  vertex(const vertex &v);
  vertex(size_t index, vector<size_t> &cds) ;


  vertex&        operator=(const vertex &v);

  double         GetHVal(size_t cid)
    { assert(cid < VEC_DIMENSION); return hVal.GetCostComponent(cid); }

  void           SetHVal(size_t cid, double val) 
    { assert(cid < VEC_DIMENSION); hVal.SetCostComponent(cid, val); }

  void           Print();
  void           WriteToFile(ofstream &fout);
  void           ReadFromFile(ifstream &fin);
};


struct grid
{
  size_t         numDim;
  size_t         dimMax;
  //~ size_t         xDimMax;
  //~ size_t         yDimMax;
  size_t         startCd[MAX_DIM];  // Cd stands for coordinate
  size_t         endCd[MAX_DIM];    // Cd stands for coordinate
               
  double         maxEdgeCost;
  double         minEdgeCost;

  double         pathLength;
  double         pathCostMin;
  double         pathCostMax;
               
  size_t         lastVertexIndex;
  vector<vertex> vertexArr;

  grid();
  grid(size_t xMax, size_t yMax, double maxEC = EDGE_COST_MAX);
  
  void           ComputeRanges();
  costVec        GetVertexHVal(size_t xCd, size_t yCd);
  costVec        GetOutEdgeCost(size_t xCd, size_t yCd, size_t dim);

  void           ConstructGrid();
  void           Print();
  void           WriteToFile(ofstream &fout);
  void           ReadFromFile(ifstream &fin);
  void           GetSuccessor(vector<size_t> &current) ;
  size_t         GetParentIndex(vector<size_t> &Cds, size_t dim) ;
  size_t 		 CalculateIndex(vector<size_t> &Cds) ;
  costVec 		 GetOutEdgeCost(vector<size_t> &Cds, size_t dim);
  costVec 		 GetVertexHVal(vector<size_t> &Cds);
};

#endif  //__GRID__
