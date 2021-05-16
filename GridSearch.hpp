#ifndef __GRIDSEARCH__
#define __GRIDSEARCH__

#include "CostVec.hpp"
#include "Grid.hpp"
#include "mystack.hpp"


struct pathEntry
{
  size_t   predIndex;
  size_t   pathIndex;
  costVec  pathCost;
  bool     isActive;


  pathEntry();
  pathEntry(const pathEntry &);
  pathEntry(size_t pIdx, size_t pathIdx, costVec pCost);
  void Print();
};


struct node
{
  node();
  //~ node(size_t xVal, size_t yVal);
  node(vector<size_t> &Cds);
  node(const node &n);
  
  size_t  GetIndex() { return index; }
  void    SetIndex(size_t idx) { index = idx; }

  costVec GetHVal() { return hEst; }
  void    SetHVal(costVec hV) { hEst = hV; }

  node& operator=(const node &n);
  bool operator==(const node &n) const;
  
  void Print();

  public:
    size_t                          index;
    //~ size_t                          xCd;
    //~ size_t                          yCd;
    vector<size_t>				          Cds;
    
    
    costVec                         hEst; 

    size_t                          lastPathIndex;
    vector<pathEntry>               pathArr;
};


bool CompCostVecL1(costVec v1, costVec v2);


class gridSearch
{
  public:
    gridSearch();
    gridSearch(size_t numDim, size_t dimMax, double maxEC = EDGE_COST_MAX, double k1 = 0.0);

    void                        ConstructGrid();
    void                        PrintProblemInstance();

    void                        WriteToFile(ofstream &fout);
    void                        ReadFromFile(ifstream &fin);
    
    openEntry                   ExtractMinL1();
    openEntry                   ExtractMinL2();
    openEntry                   ExtractMinND();
                           
    void                        InitSearch();
    bool                        IsGoalNode(node &curNode);
    size_t                      SearchNodeIndex(node &n);
    bool                        IsDominatedBySolPath(costVec fV);
    bool                        IsFiltered(costVec fV);
    bool                        IsDominatedByNodePath(size_t nIdx, costVec gV);
    void                        PruneOpen();
    void                        PruneOpenC1Threshold();
    void                        PruneNodePaths(size_t nIdx, costVec gV);
    void                        SortSolutionPaths();
                           
    //~ void                        CheckModifyNextEGNode(openEntry qCur, 
                                                           //~ size_t nextNodeXCd, size_t nextNodeYCd,
                                                           //~ costVec edgeCost);
    //~ openEntry                   GetNextEGNodeL1Ordered(openEntry qCur, 
                                                           //~ size_t nextNodeXCd, size_t nextNodeYCd,
                                                           //~ costVec edgeCost, bool &addSucc);
    
    // NAMOA Search with L1 ordering
    void                        AddToOpenL1Ordered(openEntry &qSucc);
    void                        ExpandNextNodeL1Ordered();
    void                        MOASearchL1Ordered();

    bool                        IsDominatedBySolPathQArr(costVec fVal);
    bool                        UpdateSolutionFrontier(size_t sIdx);

    
    //ACTR Search
    size_t                      ComputeStackIndex(costVec fVal);
    void                        AnytimeContractSearchStack(size_t sIdx);
    //~ openEntry                   GetNextEGNodeACTR(openEntry qCur, 
                                                           //~ size_t nextNodeXCd, size_t nextNodeYCd,
                                                           //~ costVec edgeCost, bool &addSucc);
    openEntry                   GetNextEGNodeACTR(openEntry qCur, 
                                    vector<size_t> &Cds,
                                    costVec edgeCost, bool &addSucc);
    void                        AdjustContract(int contract);
    void                        InitializeContract();
    void                        SearchContract(double L1Inc = DEFAULT_L1_INC);
    //~ openEntry                   SelectBestFromAllLevels(size_t sIdx, bool &exist, size_t &level);
    void                        AddToOpenArr(openEntry &qSucc, size_t level);
    void                        PruneOpenArr(size_t sIdxCur) ;
    void                        Normalize(vector<double> &r);

    double                      ComputeOptSFHV();
    double                      ComputeCurSFHV();
    double                      ComputeABSQuality();
                           
    void                        PrintOpen();
    void                        PrintExplicitGraph();
    void                        ReportGoal(openEntry &tmpGoalEntry);
    void                        PrintSolutionPathsMOA();
    void                        PrintSolutionPathsBB();
    void                        PrintSolutionPathsBB(size_t sIdx);
    void                        PrintBBStack(size_t sIdx);
    void                        PrintExecOut();
    void                        UpdateSFQualityOutput();
    
    openEntry                   GetNextEGNodeL1Ordered(openEntry qCur, vector<size_t> &Cds, costVec edgeCost, bool &addSucc);
    void                        CheckModifyNextEGNode(openEntry qCur, vector<size_t> &Coordinates, costVec edgeCost);
    double 						          hso(list<costVec> &curSolCostQ); 
    double 						          CalculateMinContract(double step);   
    bool              CallBack(openEntry qCur, size_t lvl);  
    bool              SolnPathEmpty(size_t sIdx);
    void              ErasePath(openEntry qTmp);
  private:                 
    size_t                      numDim;
    size_t                      dimMax;

    size_t                      c1ExtractCnt;
    size_t                      c2ExtractCnt;
    size_t                      openInsertCnt1;
    size_t                      openInsertCnt2;

    grid                        probInstance;
                           
    vector<node>                nodeArr;
                           
    list<openEntry>             openQ;
    list<openEntry>             closedQ;
    list<openEntry>             solPathQ;
    list<costVec>               optSolCostQ;
    list<costVec>               curSolCostQ;
    
    double                      optSFHV; // Solution Frontier Hyper Volume
    double                      curSFHV; // Solution Frontier Hyper Volume 
    double                      targetSFQuality;
    bool                        reportSFQuality;
    ostringstream               execOut;

    // ABS specific data
    size_t                      numRange;
    vector< list<openEntry> >   solPathQArr;
    vector<double>              rangeArr;
    vector<list<openEntry> >             stackArr;
    //~ mystackComp            	     *stackArr;
    // ABS-ACTR specific

    mystackComp    					*openArr;
    //~ vector<vector<list<openEntry> > >    openArr;
    struct timeval                       tvStartBB;
    //~ vector<vector<size_t> >              expCount;
    //~ vector<size_t>                       expCountRange;
    //~ vector<vector<bool> >                suspendFlag;
    //~ vector<int>                       d;
    //~ vector<vector<size_t> >              lim;
    size_t                               depthBound;
    size_t                               MINC;
    size_t                               MAXC;
    int                                  m;
    int                                  alpha;
    int                                  beta;
                           
    size_t                      lastNodeIndex;
    double                      c1Threshold;
    double                      kappa1;
                           
    bool                        goalReached;
    bool                        c1MinReached;
    bool                        desiredGoalReached;
                           
    openEntry                   curGoalEntry;
    openEntry                   desiredGoalEntry;
    openEntry                   c1MinEntry;
    openEntry                   curC1MinEntry;
    openEntry                   curC2MinEntry;

    list<openEntry>::iterator   minC1EntryIt;
};

#endif //__GRIDSEARCH__
