#include "GridSearch.hpp"


pathEntry::pathEntry()
{
  predIndex = ULONG_MAX;
  pathIndex = ULONG_MAX;
  isActive = false;
}


pathEntry::pathEntry(const pathEntry &t)
{
  predIndex = t.predIndex;
  pathIndex = t.pathIndex;
  pathCost  = t.pathCost;
  isActive  = t.isActive;
}


pathEntry::pathEntry(size_t pIdx, size_t pathIdx, costVec pCost)
{
  predIndex = pIdx;
  pathIndex = pathIdx;
  pathCost  = pCost;
  isActive = true;
}


void pathEntry::Print()
{
  cout << "[ parent : " << setw(4) << predIndex 
       << ", path : " << setw(4) << pathIndex 
       << ", isActive : " << isActive 
       << ", cost :";
  pathCost.Print();
  cout << "]";
}


openEntry::openEntry()
{
  nodeIndex = ULONG_MAX;
  pathIndex = ULONG_MAX;

  for(size_t i = 0; i < VEC_DIMENSION; i++) 
  {
	gValue.SetCostComponent(i, DBL_MAX);
    fValue.SetCostComponent(i, DBL_MAX);
  }
} 

openEntry::openEntry(const openEntry &t)
{
  nodeIndex = t.nodeIndex;
  pathIndex = t.pathIndex;
  gValue    = t.gValue;
  fValue    = t.fValue;
}


openEntry::openEntry(size_t nIdx, size_t pIdx, costVec gVal, costVec fVal)
{
  nodeIndex = nIdx;
  pathIndex = pIdx;
  gValue    = gVal;
  fValue    = fVal;
}


bool openEntry::operator==(const openEntry &oe) const
{
  return ((nodeIndex == oe.nodeIndex) && (pathIndex == oe.pathIndex));
}


bool openEntry::operator<(const openEntry &oe) const
{
  return fValue.IsLessL1(oe.fValue);
}


void openEntry::Print()
{
  cout << "[ index : " << setw(4) << nodeIndex
       << ", path : " << setw(4) << pathIndex 
       << ", g : ";
  gValue.Print();
  cout << ", f :";
  fValue.Print();
  cout << "]";
}


node::node()
{
  index = ULONG_MAX;
  //~ xCd = ULONG_MAX;
  //~ yCd = ULONG_MAX;
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    Cds.push_back(ULONG_MAX);

  }
      cout << "size = "<<Cds.size()<<"\n";
  lastPathIndex = ULONG_MAX;
}

/*
node::node(size_t xVal, size_t yVal)
{
  index = ULONG_MAX;
  xCd = xVal;
  yCd = yVal;
  lastPathIndex = ULONG_MAX;
}
*/
node::node(vector<size_t> &Coordinates)
{
  index = ULONG_MAX;
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    Cds.push_back(Coordinates[i]);
  }
  lastPathIndex = ULONG_MAX;
}

node::node(const node &n)
{
  index = n.index;
  //~ xCd = n.xCd;
  //~ yCd = n.yCd;
  
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    Cds.push_back(n.Cds[i]);
  }
  hEst = n.hEst;
  
  lastPathIndex = n.lastPathIndex;
  for(size_t i = 0; i < n.pathArr.size(); i++)
  {
    pathArr.push_back(n.pathArr[i]);
  }
}


bool node::operator==(const node &n) const
{
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    if(Cds[i] != n.Cds[i]) 
    {
      return false;
    }
  }
  //~ return ((xCd == n.xCd) && (yCd == n.yCd));
  return true;
}


node& node::operator=(const node &n)
{
  index = n.index;
  //~ xCd = n.xCd;
  //~ yCd = n.yCd;
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    Cds[i] = n.Cds[i];
  }
  hEst = n.hEst;
  
  lastPathIndex = n.lastPathIndex;
  for(size_t i = 0; i < n.pathArr.size(); i++)
  {
    pathArr.push_back(n.pathArr[i]);
  }

  return *this;
}


void node::Print()
{
  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
  cout << "ExpGraph Node Details : " << endl;
  cout << " [Index : "<< index <<  "]";
  //~ cout << " [xCd : "<< xCd << "]";
  //~ cout << " [yCd : "<< yCd << "]";
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    cout << " [Cds " << i << " : " << Cds[i] << "]";
  }
  cout << " [Heuristic Estimate : ";
  hEst.Print();
  cout <<  "]" << endl; 

  if(pathArr.size() > 0)
    cout << "Path Vector :" << endl;
  for(size_t i = 0; i < pathArr.size(); i++)
  {
    pathArr[i].Print();
    cout << "\t";
  }
  cout << endl;
  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
}


bool CompCostVecL1(costVec cV1, costVec cV2)
{
  return cV1.IsLessL1(cV2);
}


gridSearch::gridSearch()
  :
    //~ xDimMax(DEFAULT_X_DIM_MAX),
    //~ yDimMax(DEFAULT_Y_DIM_MAX),
    numDim(DEFAULT_NUM_DIM),
    dimMax(DEFAULT_DIM_MAX),
    c1ExtractCnt(0),
    c2ExtractCnt(0),
    openInsertCnt1(0),
    openInsertCnt2(0),
    probInstance(numDim, dimMax),
    optSFHV(0),
    curSFHV(0),
    targetSFQuality(INITIAL_SF_QUALITY),
    reportSFQuality(true),
    numRange(DEFAULT_NUM_RANGE),
    depthBound(DEFAULT_DEPTH_BOUND),
    lastNodeIndex(ULONG_MAX),
    c1Threshold(0.0),
    kappa1(0.0),
    goalReached(false),
    c1MinReached(false),
    desiredGoalReached(false)
{
}


gridSearch::gridSearch(size_t numberOfDimensions, size_t maximumAlongDimension, double maxEC, double k1)
  :
    numDim(numberOfDimensions),
    dimMax(maximumAlongDimension),
    c1ExtractCnt(0),
    c2ExtractCnt(0),
    openInsertCnt1(0),
    openInsertCnt2(0),
    probInstance(numberOfDimensions, maximumAlongDimension, maxEC),
    optSFHV(0),
    curSFHV(0),
    targetSFQuality(INITIAL_SF_QUALITY),
    reportSFQuality(true),
    numRange(DEFAULT_NUM_RANGE),
    depthBound(DEFAULT_DEPTH_BOUND),
    lastNodeIndex(ULONG_MAX),
    c1Threshold(0.0),
    kappa1(k1),
    goalReached(false),
    c1MinReached(false),
    desiredGoalReached(false) 
{
}


void gridSearch::ConstructGrid()
{ 
   probInstance.ConstructGrid();
}


void gridSearch::PrintProblemInstance()
{
  probInstance.Print();
}

    
void gridSearch::WriteToFile(ofstream &fout)
{
  assert(fout.is_open() && fout.good());

  //~ fout << ' ' << xDimMax;
  //~ fout << ' ' << yDimMax;
  fout << ' ' << numDim;
  fout << ' ' << dimMax;
  
  probInstance.WriteToFile(fout);
  
  fout << ' ' <<  solPathQ.size();
  for(list<openEntry>::iterator it = solPathQ.begin();
      it != solPathQ.end(); it++)
  {
    (*it).fValue.WriteToFile(fout);
  }  
}


void gridSearch::ReadFromFile(ifstream &fin)
{
  assert(fin.is_open() && fin.good());

  //~ fin >> xDimMax;
  //~ fin >> yDimMax;
  fin >> numDim;
  fin >> dimMax;

  probInstance.ReadFromFile(fin);

  size_t solPathCnt(0);
  fin >> solPathCnt;
  for(size_t i = 0; i < solPathCnt; i++)
  {
    costVec cV;
    cV.ReadFromFile(fin);
    optSolCostQ.push_back(cV);
  }

  /*
  for(list<costVec>::iterator scIt = optSolCostQ.begin();
      scIt != optSolCostQ.end(); scIt++)
    scIt->Print();
    */
  optSolCostQ.sort(CompCostVecL1);  
}


openEntry gridSearch::ExtractMinL1()
{
  openEntry qCur = openQ.front();
  list<openEntry>::iterator it, minIt;

  for(it = openQ.begin(), minIt = openQ.begin(); it != openQ.end(); it++)
  {
    if(it->fValue.IsLessL1(qCur.fValue))
    {
      qCur = *it;
      minIt = it;
    }
  }

  // Removing the min entry
  openQ.erase(minIt);

  /*
  cout << endl;
  cout << "Extracted from Open L1" << endl;
  qCur.Print();
  cout << endl;
  */

  return qCur;
}


openEntry gridSearch::ExtractMinL2()
{
  openEntry qCur = openQ.front();
  list<openEntry>::iterator it, minIt;

  for(it = openQ.begin(), minIt = openQ.begin(); it != openQ.end(); it++)
  {
    if(it->fValue.IsLessL2(qCur.fValue))
    {
      qCur = *it;
      minIt = it;
    }
  }

  // Removing the min entry
  openQ.erase(minIt);

  /*
  cout << endl;
  cout << "Extracted from Open L2" << endl;
  qCur.Print();
  cout << endl;
  */

  return qCur;
}


openEntry gridSearch::ExtractMinND()
{
  openEntry qCur;

  for(list<openEntry>::iterator itCur = openQ.begin();
      itCur != openQ.end(); itCur++)
  {
    qCur = *itCur;
    bool isCurEntryDominated(false);

    for(list<openEntry>::iterator it = openQ.begin(); it != openQ.end(); it++)
    {
      if(it->fValue < qCur.fValue)
      {
        isCurEntryDominated = true;
      }
    }

    if(!isCurEntryDominated)
    {
      // Removing the current entry entry
      openQ.erase(itCur);
      return qCur;
    }
  }
  
  cerr << "Error : No non dominated entry found" << endl;
  PrintOpen();
  PRINT_ERROR_LOC;

  return qCur;
}


void gridSearch::InitSearch()
{
  //PrintProblemInstance();

  // costVec is initialized at (0,0)
  costVec gVal;
  
  vector<size_t> init;
  //~ cout <<"size = "<<init.size();
  for(size_t i = 0; i < numDim; i++) 
  {
	//~ cout <<"pushing "<<i<<"\n";
    init.push_back(0);
    //~ cout << "current size = "<<init.size()<<"\n";
  }
  //~ cout << "Calculating hval\n";
  costVec hVal = probInstance.GetVertexHVal(init);
  costVec fVal;

  //~ cout<<"Construct the root node of the explicit graph\n";
  lastNodeIndex = 0;
  node n(init);
  n.SetIndex(lastNodeIndex);
  n.SetHVal(hVal);
  n.lastPathIndex = 0;

  // Push the node in nodeArr
  nodeArr.push_back(n);
  
  fVal = gVal + hVal;

  // Push the corresponding entry to OPEN
  openEntry q(lastNodeIndex, 0, gVal, fVal);  
  openQ.push_back(q);
  
  c1Threshold = fVal.GetCostComponent(0) * (1 + kappa1);
  //cout << "c1Threshold : " << c1Threshold << endl;
  curC1MinEntry = q;
  curC2MinEntry = q;

  minC1EntryIt = openQ.begin();
}


bool gridSearch::IsGoalNode(node &curNode)
{
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    if(curNode.Cds[i] != dimMax - 1) 
    {
      return false;
    }
  }
  return true;
  //~ return (curNode.xCd == (xDimMax - 1) && curNode.yCd == (yDimMax - 1));
}


size_t gridSearch::SearchNodeIndex(node &n)
{
  for(size_t i = 0; i < nodeArr.size(); i++)
  {
    if(n == nodeArr[i])
      return i;
  }

  return ULONG_MAX;
}


bool gridSearch::IsDominatedBySolPath(costVec fV)
{
  bool ignoreNewNextNode(false);

  // Checking whether the fValue is already dominated by any goal
  list<openEntry>::iterator it = solPathQ.begin();
  for(; it != solPathQ.end(); it++)
  {
    // Check for dominance
    if(it->fValue <= fV)
    {
      ignoreNewNextNode = true;
      break;
    }
  }

  return ignoreNewNextNode;
}


bool gridSearch::IsDominatedByNodePath(size_t nIdx, costVec gV)
{
  bool skipFlag(false);
  
  // Check whether this path is non-dominated
  for(vector<pathEntry>::iterator itPath = nodeArr[nIdx].pathArr.begin(); 
      itPath != nodeArr[nIdx].pathArr.end(); itPath++)
  {
    if(itPath->pathCost <= gV)
    {
      skipFlag = true;
      break;
    }
  }

  return skipFlag;
}


void gridSearch::PruneOpen()
{
  list<openEntry>::iterator it = openQ.begin();
  for(; it != openQ.end();)
  {
    // Prune Open
    if(curGoalEntry.fValue <= it->fValue)
    {
      openEntry qTmp = *it;
      it = openQ.erase(it);
      nodeArr[qTmp.nodeIndex].pathArr[qTmp.pathIndex].isActive = false;
    }
    else
    {
      it++;
    }
  }
}


void gridSearch::PruneNodePaths(size_t nIdx, costVec gV)
{
  vector<pathEntry>::iterator itPath;
  size_t pathIdx;

  // Invalidate the costs which are dominated by gV
  for(pathIdx = 0, itPath = nodeArr[nIdx].pathArr.begin(); 
      itPath !=  nodeArr[nIdx].pathArr.end(); itPath++, pathIdx++)
  {
    if((itPath->isActive) && (gV <= itPath->pathCost))
    {
      itPath->isActive = false;
    }
  }

  for(list<openEntry>::iterator it = openQ.begin(); it != openQ.end();)
  {
    // Prune the corresponding entry from Open
    if((nIdx == (it->nodeIndex)) && (gV <= it->gValue))
    {
      it = openQ.erase(it);
    }
    else
    {
      it++;
    }
  }
}


void gridSearch::SortSolutionPaths()
{
  if(solPathQ.size() > 1)
  {
    // Sorting the list for debug purpose
    list<openEntry>::iterator it = solPathQ.begin();  
    it++;
    while(it != solPathQ.end())
    {
      --it;
      list<openEntry>::iterator itLoop = it;
      ++it;
     
      while(itLoop != solPathQ.begin() && (*it).fValue.IsLessL1(itLoop->fValue))
      {
        itLoop--;
      }

      if(itLoop != solPathQ.begin())
      {
        itLoop++;
        if(itLoop != it)
        {
          solPathQ.insert(itLoop, (*it));
          it = solPathQ.erase(it);
        }
        else
        {
          it++;
        }
      }
      else
      {
        if(!(*it).fValue.IsLessL1(itLoop->fValue))
        {
          itLoop++;
        }
        solPathQ.insert(itLoop, (*it));
        it = solPathQ.erase(it);
      }
    }
  }
}


void gridSearch::AddToOpenL1Ordered(openEntry &qSucc) 
{
  list<openEntry>::iterator oeIt = openQ.begin();
  openEntry oeC1Min = qSucc;

  while((oeIt != openQ.end()) && (oeIt->fValue.IsLessL1(qSucc.fValue)))
  {
    if(oeIt->fValue.IsLessL1(oeC1Min.fValue))
    {
      oeC1Min = *oeIt;
      minC1EntryIt = oeIt;
    }
    oeIt++;
  } 
  openQ.insert(oeIt, qSucc);

  oeIt--;  
  while(oeIt != openQ.end())
  {
    if(oeIt->fValue.IsLessL1(oeC1Min.fValue))
    {
      oeC1Min = *oeIt;
      minC1EntryIt = oeIt;
    }
    oeIt++;
  }
      
  curC1MinEntry = oeC1Min;

/*
  cout << "curC1MinEntry : ";
  curC1MinEntry.Print();
  cout << endl;
*/
}
 
/*
// Checks and modifies the next explicit graph node
// Takes care of the dominated paths and
// inserts the corresponding entry to Open
openEntry gridSearch::GetNextEGNodeL1Ordered(openEntry qCur, 
                                    size_t nextNodeXCd, size_t nextNodeYCd,
                                    costVec edgeCost, bool &addSucc)
{
  assert(addSucc);
  size_t nIdx = qCur.nodeIndex;

  node nextNode(nextNodeXCd, nextNodeYCd);
  size_t newNodeIdx(ULONG_MAX);
  
  // Check whether this node is already created 
  if((newNodeIdx = SearchNodeIndex(nextNode)) == ULONG_MAX)
  {
    // The node is not present in the explicit graph
    // Compute the g and f        
    costVec hV = probInstance.GetVertexHVal(nextNodeXCd, nextNodeYCd);
    costVec gV = qCur.gValue + edgeCost;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPath(fV))
    {
      // Setting the heuristic value of the newly created node
      nextNode.hEst = hV;

      // Add the path in the path array
      nextNode.lastPathIndex = 0;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      nextNode.pathArr.push_back(pCur);

      // Create a new node in the explicit graph
      lastNodeIndex++;
      nextNode.SetIndex(lastNodeIndex);
      nodeArr.push_back(nextNode);
      
      // Push the corresponding entry in OPEN
      openEntry qNew(nextNode.index, nextNode.lastPathIndex, gV, fV);
      return qNew;

    }
    else
    {
      addSucc = false;
      //cout << "current successor ignored" << endl;
    }
  }
  else
  {
    // This node is already present in nodeArr; existing Next Node
    node &exNextNode = nodeArr[newNodeIdx];
    
    costVec gV = qCur.gValue + edgeCost;
    costVec hV = exNextNode.hEst;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPath(fV) && !IsDominatedByNodePath(newNodeIdx, gV))
    {
      // Remove the costs which are dominated by gV
      PruneNodePaths(newNodeIdx, gV);

      // Add the path in the path array
      exNextNode.lastPathIndex++;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      exNextNode.pathArr.push_back(pCur);

      // Push the corresponding entry in OPEN
      openEntry qNew(exNextNode.index, exNextNode.lastPathIndex, gV, fV);
      return qNew;
      
    }
    else
    {
      addSucc = false;
      //cout << "current successor skipped" << endl;
    }
  }

  openEntry qTemp;
  return qTemp;
}*/
 
openEntry gridSearch::GetNextEGNodeL1Ordered(openEntry qCur, 
                                    vector<size_t> &Cds,
                                    costVec edgeCost, bool &addSucc)
{
  assert(addSucc);
  size_t nIdx = qCur.nodeIndex;

  node nextNode(Cds);
  size_t newNodeIdx(ULONG_MAX);
  
  // Check whether this node is already created 
  if((newNodeIdx = SearchNodeIndex(nextNode)) == ULONG_MAX)
  {
    // The node is not present in the explicit graph
    // Compute the g and f        
    costVec hV = probInstance.GetVertexHVal(Cds);
    costVec gV = qCur.gValue + edgeCost;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPath(fV))
    {
      // Setting the heuristic value of the newly created node
      nextNode.hEst = hV;

      // Add the path in the path array
      nextNode.lastPathIndex = 0;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      nextNode.pathArr.push_back(pCur);

      // Create a new node in the explicit graph
      lastNodeIndex++;
      nextNode.SetIndex(lastNodeIndex);
      nodeArr.push_back(nextNode);
      
      // Push the corresponding entry in OPEN
      openEntry qNew(nextNode.index, nextNode.lastPathIndex, gV, fV);
      return qNew;

      /*
      cout << "Added to Open" << endl;
      qNew.Print();
      cout << endl;

      cout << "New Child Node" << endl;
      nextNode.Print();
      */
    }
    else
    {
      addSucc = false;
      //cout << "current successor ignored" << endl;
    }
  }
  else
  {
    // This node is already present in nodeArr; existing Next Node
    node &exNextNode = nodeArr[newNodeIdx];
    
    costVec gV = qCur.gValue + edgeCost;
    costVec hV = exNextNode.hEst;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPath(fV) && !IsDominatedByNodePath(newNodeIdx, gV))
    {
      // Remove the costs which are dominated by gV
      PruneNodePaths(newNodeIdx, gV);

      // Add the path in the path array
      exNextNode.lastPathIndex++;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      exNextNode.pathArr.push_back(pCur);

      // Push the corresponding entry in OPEN
      openEntry qNew(exNextNode.index, exNextNode.lastPathIndex, gV, fV);
      return qNew;
      
      /*
      cout << "Added to Open" << endl;
      qNew.Print();
      cout << endl;
      
      cout << "Updated Child Node" << endl;
      exNextNode.Print();
      */
    }
    else
    {
      addSucc = false;
      //cout << "current successor skipped" << endl;
    }
  }

  openEntry qTemp;
  return qTemp;
}
 
// Checks and modifies the next explicit graph node
// Takes care of the dominated paths and
// inserts the corresponding entry to Open
void gridSearch::CheckModifyNextEGNode(openEntry qCur, 
                                    vector<size_t> &Coordinates,
                                    costVec edgeCost)
{
  size_t nIdx = qCur.nodeIndex;

  node nextNode(Coordinates);
  size_t newNodeIdx(ULONG_MAX);
  
  // Check whether this node is already created 
  if((newNodeIdx = SearchNodeIndex(nextNode)) == ULONG_MAX)
  {
    // The node is not present in the explicit graph
    // Compute the g and f        
    costVec hV = probInstance.GetVertexHVal(Coordinates);
    costVec gV = qCur.gValue + edgeCost;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPath(fV))
    {
      // Setting the heuristic value of the newly created node
      nextNode.hEst = hV;

      // Add the path in the path array
      nextNode.lastPathIndex = 0;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      nextNode.pathArr.push_back(pCur);

      // Create a new node in the explicit graph
      lastNodeIndex++;
      nextNode.SetIndex(lastNodeIndex);
      nodeArr.push_back(nextNode);
      
      // Push the corresponding entry in OPEN
      openEntry qNew(nextNode.index, nextNode.lastPathIndex, gV, fV);
      
      list<openEntry>::iterator opIt = openQ.begin();
      while((opIt != openQ.end()) && (opIt->fValue.IsLessL1(qNew.fValue)))
        opIt++;
      openQ.insert(opIt, qNew);

      /*
      cout << "Added to Open" << endl;
      qNew.Print();
      cout << endl;

      cout << "New Child Node" << endl;
      nextNode.Print();
      */
    }
    else
    {
      //cout << "current successor ignored" << endl;
    }
  }
  else
  {
    // This node is already present in nodeArr; existing Next Node
    node &exNextNode = nodeArr[newNodeIdx];
    
    costVec gV = qCur.gValue + edgeCost;
    costVec hV = exNextNode.hEst;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPath(fV) && !IsDominatedByNodePath(newNodeIdx, gV))
    {
      // Remove the costs which are dominated by gV
      PruneNodePaths(newNodeIdx, gV);

      // Add the path in the path array
      exNextNode.lastPathIndex++;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      exNextNode.pathArr.push_back(pCur);

      // Push the corresponding entry in OPEN
      openEntry qNew(exNextNode.index, exNextNode.lastPathIndex, gV, fV);
      
      list<openEntry>::iterator opIt = openQ.begin();
      while((opIt != openQ.end()) && (opIt->fValue.IsLessL1(qNew.fValue)))
        opIt++;
      openQ.insert(opIt, qNew);
      
      /*
      cout << "Added to Open" << endl;
      qNew.Print();
      cout << endl;
      
      cout << "Updated Child Node" << endl;
      exNextNode.Print();
      */
    }
    else
    {
      //cout << "current successor skipped" << endl;
    }
  }
}


void gridSearch::ExpandNextNodeL1Ordered()
{
  //PrintOpen();
  openEntry qCur = openQ.front();
  openQ.pop_front();

  closedQ.push_back(qCur);
  c1ExtractCnt++;

  assert(qCur.nodeIndex < nodeArr.size());
  size_t nIdx = qCur.nodeIndex;

  if(IsGoalNode(nodeArr[nIdx]))
  {
    // If it is a goal node there is nothing to do.
    // hEst contains the exact cost remaining.
    // The fValue entry of qCur is the cost of this goal.
    goalReached = true;
    
    if(!c1MinReached)
    {
      c1MinReached = true;
      c1MinEntry = qCur;
      c1Threshold = c1MinEntry.fValue.GetCostComponent(0) * (1 + kappa1);
    }

    curGoalEntry = qCur;
    solPathQ.push_back(qCur);
    PruneOpen();
    return;
  }

  /*
  cout << "Current Node" << endl;
  nodeArr[nIdx].Print();
  */
  
  vector<size_t> coordinates;
  for(size_t i = 0; i < numDim; i++) 
  {
    coordinates.push_back(nodeArr[nIdx].Cds[i]);
  }
  for(size_t i = 0; i < numDim; i++) 
  {
    bool addiDimSucc(true);
    openEntry iDimSucc;
    if(nodeArr[nIdx].Cds[i] < dimMax - 1) 
    {
      coordinates[i]++;
      costVec edgeCost = probInstance.GetOutEdgeCost(nodeArr[nIdx].Cds, i);
      iDimSucc = GetNextEGNodeL1Ordered(qCur, coordinates, edgeCost, addiDimSucc);
    }
    else 
    {
      addiDimSucc = false;
    }
    
    if(addiDimSucc) 
    {
      AddToOpenL1Ordered(iDimSucc);
    }
  }
  /*
  bool addXDimSucc(true), addYDimSucc(true);
  openEntry xDimSucc, yDimSucc;
  // Explore the successors of this node
  // x-dimension
  if(nodeArr[nIdx].xCd < (xDimMax - 1))
  {
    size_t nextNodeXCd = nodeArr[nIdx].xCd + 1;
    size_t nextNodeYCd = nodeArr[nIdx].yCd;
    costVec edgeCost = probInstance.GetOutEdgeCost(nodeArr[nIdx].xCd, 
                                                   nodeArr[nIdx].yCd, XDIM);

    xDimSucc = GetNextEGNodeL1Ordered(qCur, nextNodeXCd, nextNodeYCd, edgeCost, addXDimSucc);
  }
  else
  {
    addXDimSucc = false;
  }
  
  if(addXDimSucc)
  {
    AddToOpenL1Ordered(xDimSucc);
  }
  
  // y-dimension
  if(nodeArr[nIdx].yCd < (yDimMax - 1))
  {
    size_t nextNodeXCd = nodeArr[nIdx].xCd;
    size_t nextNodeYCd = nodeArr[nIdx].yCd + 1;
    costVec edgeCost = probInstance.GetOutEdgeCost(nodeArr[nIdx].xCd, 
                                                   nodeArr[nIdx].yCd, YDIM);

    yDimSucc = GetNextEGNodeL1Ordered(qCur, nextNodeXCd, nextNodeYCd, edgeCost, addYDimSucc);
  }
  else
  {
    addYDimSucc = false;
  }
  
  if(addYDimSucc)
  {
    AddToOpenL1Ordered(yDimSucc);
  }*/
}
  

void gridSearch::MOASearchL1Ordered()
{
  // Check the contents of OPEN and
  // repeatedly expand the next node

  struct timeval tvStart, tvFinish;

  gettimeofday(&tvStart, NULL);

  InitSearch();
  
  //PrintOpen();

  while(!openQ.empty())
  {
	//~ cout << "\n Expanding Node L1 ordered \n";
    ExpandNextNodeL1Ordered();
  }

  gettimeofday(&tvFinish, NULL);

  unsigned diff = (tvFinish.tv_sec * 1000000 + tvFinish.tv_usec) -
                  (tvStart.tv_sec * 1000000 + tvStart.tv_usec);

  double timeD = ((double) diff) / 1000000.0;


  //cout << "Total time taken : ";
  cout << "   & ";
  cout << setw(7) << solPathQ.size(); 
  cout << "   & ";
  cout << setiosflags(ios::fixed) << setiosflags(ios::right)
       << setprecision(3) << setw(7) << timeD;
  cout << endl;
        
  PrintSolutionPathsMOA();
} 


double gridSearch::ComputeOptSFHV()
{
  if(optSolCostQ.size() == 0)
  {
    optSFHV = 0;
    return optSFHV;
  }

  optSFHV = hso(optSolCostQ);
  //~ cout << "opt SFHV = "<<optSFHV<<"\n"; 
  return optSFHV;
  //cout << "Refpoint = " << probInstance.pathCostMax << endl;
/*
  list<costVec>::iterator scItNext = optSolCostQ.begin();
  list<costVec>::iterator scIt = optSolCostQ.begin();
  double height(0);
  double width(0);
  double vol(0);

  scItNext++;
  for(; scItNext != optSolCostQ.end(); scIt++, scItNext++)
  {
    height = probInstance.pathCostMax - (*scIt).GetCostComponent(YDIM);
    width = (*scItNext).GetCostComponent(XDIM) - (*scIt).GetCostComponent(XDIM);
    vol = height * width;

    //scIt->Print();
    //cout << endl;
    //cout << "height = " << height 
    //     << " width = " << width 
    //     << " vol = " << vol << endl;

    optSFHV += vol;
  }
  
  height = probInstance.pathCostMax - (*scIt).GetCostComponent(YDIM);
  width = probInstance.pathCostMax - (*scIt).GetCostComponent(XDIM);
  vol = height * width;

  //scIt->Print();
  //cout << endl;
  //cout << "height = " << height 
  //     << " width = " << width 
  //     << " vol = " << vol << endl;

  optSFHV += vol;

  return optSFHV;*/
}

class hpvNode 
{
	public:
	size_t index;
    list<costVec> solnCost;
	
	hpvNode(size_t idx, list<costVec> solnC) 
	{
		index = idx;
		for(list<costVec>::iterator li = solnC.begin(); li != solnC.end(); li++) 
		{
			solnCost.push_back(*li);
		}
	}
	static list<hpvNode> slice(list<costVec> p1, size_t k, size_t pathcostmax) 
	{
		costVec p = p1.front();
		p1.pop_front();
		
		list<costVec> q1;
		list<hpvNode> s;
		
		while(!p1.empty())
		{
			q1 = insert(p, k+1, q1);
			costVec p_dash = p1.front();
			hpvNode x(abs(p.GetCostComponent(k) - p_dash.GetCostComponent(k)), q1);
			s.push_back(x);
			p = p_dash;
			p1.pop_front();
		}
		q1 = insert(p, k+1, q1);
		hpvNode y(abs(p.GetCostComponent(k) - pathcostmax), q1);
		s.push_back(y);
		return s;
	}
	
	static list<costVec> insert(costVec p, size_t k, list<costVec> p1) 
	{
		list<costVec> q1;
		while(!p1.empty() && p1.front().GetCostComponent(k) < p.GetCostComponent(k))
		{
			q1.push_back(p1.front());
			p1.pop_front();
		}
		q1.push_back(p);
		while(!p1.empty()) 
		{
			if(!p.Dominates(p1.front(), k)) 
			{
				q1.push_back(p1.front());
			}
			p1.pop_front();
		}
		return q1;
	}

	
	static double Modulus(costVec cv, size_t pathcostmax, size_t numDim) 
	{
		//~ cout << "pathcostmax = "<<pathcostmax<<endl;
		double y = abs(cv.GetCostComponent(numDim - 1) - pathcostmax);
		//~ cout<<" returning "<<y<<endl;
		return y;
    
	}

};

double gridSearch::hso(list<costVec> &curSolCostQ) 
{
  //~ cout << "Computing hyper volume \n";
  hpvNode snode(1, curSolCostQ);
  list<hpvNode> s;
  s.push_back(snode);
  
  for(size_t k = 0; k < numDim - 1; k++) 
  {
	  list<hpvNode> s1;
	  for(list<hpvNode>::iterator it = s.begin(); it != s.end(); it++) 
      {
		  list<hpvNode> xq1 = hpvNode::slice(it->solnCost, k, probInstance.pathCostMax);
		  for(list<hpvNode>::iterator it2 = xq1.begin(); it2 != xq1.end(); it2++) 
		  {
			    hpvNode xx1q11((it2->index) * (it->index), it2->solnCost); 
				s1.push_back(xx1q11); 
		  }
	  }
	  s = s1;
  }
  double vol(0);
  
  for(list<hpvNode>::iterator it = s.begin(); it != s.end(); it++) 
  {
	 vol += (it->index)*(hpvNode::Modulus((it->solnCost).front(), probInstance.pathCostMax, numDim));
	 //~ cout << " vol = "<<vol<<endl;
  }
  return vol;
}

double gridSearch::ComputeCurSFHV()
{
  curSFHV = 0;
  if(curSolCostQ.size() == 0)
  {
    curSFHV = 0;
    return curSFHV;
  }
  curSFHV = hso(curSolCostQ);
  //~ cout << "current Cur SFHV = "<<curSFHV<<"\n"; 
  return curSFHV;
  //cout << "Refpoint = " << probInstance.pathCostMax << endl;
/*
  list<costVec>::iterator scItNext = curSolCostQ.begin();
  list<costVec>::iterator scIt = curSolCostQ.begin();
  double height(0);
  double width(0);
  double vol(0);

  scItNext++;
  for(; scItNext != curSolCostQ.end(); scIt++, scItNext++)
  {
    height = probInstance.pathCostMax - (*scIt).GetCostComponent(YDIM);
    width = (*scItNext).GetCostComponent(XDIM) - (*scIt).GetCostComponent(XDIM);
    vol = height * width;

    //scIt->Print();
    //cout << endl;
    //cout << "height = " << height 
    //     << " width = " << width 
    //     << " vol = " << vol << endl;

    curSFHV += vol;
  }
  
  height = probInstance.pathCostMax - (*scIt).GetCostComponent(YDIM);
  width = probInstance.pathCostMax - (*scIt).GetCostComponent(XDIM);
  vol = height * width;

  //scIt->Print();
  //cout << endl;
  //cout << "height = " << height 
  //     << " width = " << width 
  //     << " vol = " << vol << endl;

  curSFHV += vol;

  return curSFHV;*/
}


double gridSearch::ComputeABSQuality()
{
  double fracOpt = curSFHV / optSFHV;
  fracOpt *= 100;

  return fracOpt;
}


void gridSearch::PrintOpen()
{
  cout << setfill('#') << setw(80) << "#" << setfill(' ') << endl;

  cout << "Contents of Open:" << endl;

  list<openEntry>::iterator it;
  for(it = openQ.begin(); it != openQ.end(); it++)
  {
    it->Print();
    cout << endl;
  }

  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
}


void gridSearch::PrintExplicitGraph()
{
  cout << setfill('*') << setw(80) << "*" << setfill(' ') << endl;

  cout << "Explicit Graph Details" << endl;
  for(size_t i = 0; i < nodeArr.size(); i++)
  {
    nodeArr[i].Print();
  }

  cout << setfill('=') << setw(80) << "=" << setfill(' ') << endl;
}


void gridSearch::ReportGoal(openEntry &tmpGoalEntry)
{
  size_t nIdx = tmpGoalEntry.nodeIndex;
  size_t pIdx = tmpGoalEntry.pathIndex;
  
  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;

  cout << "Goal Entry : ";
  tmpGoalEntry.Print();
  cout << endl;

  cout << "Tracing the path from the Goal node to Root node" << endl;
  while(nIdx != 0)
  {
    pathEntry tmp = nodeArr[nIdx].pathArr[pIdx];
    cout << "Node : " << nIdx << endl;
    cout << "Path : ";
    tmp.Print();
    cout << endl;
    cout << "Cost : ";
    tmp.pathCost.Print();
    cout << endl;
    pIdx = tmp.pathIndex;
    nIdx = tmp.predIndex;
  }

  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
}


void gridSearch::PrintSolutionPathsMOA()
{  
  cout << "Printing the soltuion paths" << endl;
  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;

  size_t solPathCnt = 0;
  list<openEntry>::iterator it = solPathQ.begin();  
  for(; it != solPathQ.end(); solPathCnt++, it++)
  {
    //~ openEntry tmpGoalEntry = *it;
    cout << "Solution Number " << setw(4) << solPathCnt << " : ";
    it->Print();
    cout << endl;
    //ReportGoal(tmpGoalEntry);
  }
  
  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
}


void gridSearch::PrintExecOut()
{
  ComputeCurSFHV();
  double SFQuality = ComputeABSQuality();
  //~ cout<<"here";
  if(curSFHV == 0)
  {
    double negTime(-2);
    SFQuality = DEFAULT_SF_QUALITY;
    
    execOut << ' ';
    execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
            << setprecision(3) << setw(7) 
            << SFQuality;

    execOut << ' ';
    execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
            << setprecision(3) << setw(7) 
            << negTime;

    execOut << endl;
  }
  else
  {
    while(SFQuality <= (INITIAL_SF_QUALITY - DEFAULT_SF_Q_INC))
    {
      SFQuality += DEFAULT_SF_Q_INC;
    }
  }
  
  while(SFQuality < FINAL_SF_QUALITY)
  {
    double negTime(-2);
    execOut << ' ';
    execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
            << setprecision(3) << setw(7) 
            << targetSFQuality;
    
    execOut << ' ';
    execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
            << setprecision(3) << setw(7) 
            << negTime;

    execOut << endl;

    targetSFQuality += DEFAULT_SF_Q_INC;
    SFQuality += DEFAULT_SF_Q_INC;
  }

  cout << execOut.str();
  cout.flush();
}
