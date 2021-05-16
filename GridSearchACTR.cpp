#include "GridSearch.hpp"


size_t gridSearch::ComputeStackIndex(costVec fVal)
{
  double c1Val = fVal.GetCostComponent(0);
  assert(c1Val < rangeArr[numRange]);
  size_t sIdx(1);
  for(; (sIdx < numRange) && (c1Val >= rangeArr[sIdx]) ; sIdx++)
  {}
  sIdx--;

  //~ cout <<"cost = "<<c1Val<<" sidx = "<<sIdx<<endl;
  return sIdx;
}

void gridSearch::AddToOpenArr(openEntry &qSucc, size_t level)
{/*
  size_t sIdx = ComputeStackIndex(qSucc.fValue);
  //cout << "Inserting into stack " << sIdx << endl;
  openArr[sIdx][level].push_back(qSucc);
*/
}


bool gridSearch::IsDominatedBySolPathQArr(costVec fVal)
{
  bool ignoreNewNextNode(false);

  // Checking whether the fValue is already dominated by any goal
  size_t sIdxCur = ComputeStackIndex(fVal);
  for(size_t sIdx = 0; sIdx <= sIdxCur; sIdx++)
  {
    if(false == solPathQArr[sIdx].empty())
    {
      list<openEntry>::iterator it = solPathQArr[sIdx].begin();
      for(; it != solPathQArr[sIdx].end(); it++)
      {
        // Check for dominance
        if(it->fValue <= fVal)
        {
          ignoreNewNextNode = true;
          break;
        }
      }
    }
  }

  return ignoreNewNextNode;
}


bool gridSearch::UpdateSolutionFrontier(size_t sIdxCur)
{
  //cout << "Before updating the solPathQArr[" << sIdxCur << "]" << endl;
  //PrintSolutionPathsBB(sIdxCur);

  bool solFrontierChanged(true);

  for(size_t sIdx = 0; sIdx <= sIdxCur; sIdx++)
  {
    if(false == solPathQArr[sIdx].empty())
    {  
      list<openEntry>::iterator it = solPathQArr[sIdx].begin();
      for(; it != solPathQArr[sIdx].end(); it++)
      {
        //Check whether the newly found solution is non-dominated or not
        if(it->fValue < curGoalEntry.fValue)
        {
          // Ignore this soltuion and return
          solFrontierChanged = false;
          //cout << "After updating the solPathQArr[" << sIdx << "]" << endl;
          //PrintSolutionPathsBB(sIdx);
          return solFrontierChanged;
        }
        else if(it->fValue == curGoalEntry.fValue)       
        {
          // Push this solution but dont trigger prunning of the stackArr[sIdx]
          solPathQArr[sIdx].push_back(curGoalEntry);
          solFrontierChanged = false;
          //cout << "After updating the solPathQArr[" << sIdx << "]" << endl;
          //PrintSolutionPathsBB(sIdx);
          return solFrontierChanged;
        }
      }
    }
  }
  
  for(size_t sIdx = sIdxCur; sIdx < numRange; sIdx++)
  {
    if(false == solPathQArr[sIdx].empty())
    {  
      list<openEntry>::iterator it = solPathQArr[sIdx].begin();
      for(; it != solPathQArr[sIdx].end();)
      {
        //Prune the existing solutions that are dominated by newly found solution
        if(curGoalEntry.fValue < it->fValue)
        {
          openEntry qTmp = *it;
          it = solPathQArr[sIdx].erase(it);
          nodeArr[qTmp.nodeIndex].pathArr[qTmp.pathIndex].isActive = false;
        }
        else
        {
          it++;
        }
      }
    }
  }

  solPathQArr[sIdxCur].push_back(curGoalEntry);

  curSolCostQ.clear();
  
  for(size_t sIdx = 0; sIdx < numRange; sIdx++)
  {
    if(false == solPathQArr[sIdx].empty())
    {  
      for(list<openEntry>::iterator it = solPathQArr[sIdx].begin();
          it != solPathQArr[sIdx].end(); it++)
      {
        curSolCostQ.push_back((*it).fValue);
      }
    }
  }
  curSolCostQ.sort(CompCostVecL1);

  //for(list<costVec>::iterator scIt = curSolCostQ.begin();
  //    scIt != curSolCostQ.end(); scIt++)
  //scIt->Print();
  //cout << endl;
  
  //cout << "After updating the solPathQArr[" << sIdxCur << "]" << endl;
  //PrintSolutionPathsBB(sIdxCur);
  
  return solFrontierChanged;
}


void gridSearch::ErasePath(openEntry qTmp) {
  nodeArr[qTmp.nodeIndex].pathArr[qTmp.pathIndex].isActive = false;
}


void gridSearch::PruneOpenArr(size_t sIdxCur) 
{/*
  for(size_t sIdx = sIdxCur; sIdx < numRange; sIdx++)
  {
    if(false == solPathQArr[sIdx].empty())
    {  
      for(size_t j = 0; j<depthBound; j++) 
      {
        list<openEntry>::iterator it = openArr[sIdx][j].begin();
        for(; it != openArr[sIdx][j].end();)
        {
          // Prune Open
          if(curGoalEntry.fValue < it->fValue)
          {
            openEntry qTmp = *it;
            it = openArr[sIdx][j].erase(it);
            nodeArr[qTmp.nodeIndex].pathArr[qTmp.pathIndex].isActive = false;
          }
          else
          {
            it++;
          }
        }
      }
    }
  }*/
}
// Checks and modifies the next explicit graph node
// Takes care of the dominated paths and
// inserts the corresponding entry to Open
openEntry gridSearch::GetNextEGNodeACTR(openEntry qCur, 
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
    
    if(!IsDominatedBySolPathQArr(fV))
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

/*      
      cout << "Added to Open" << endl;
      qNew.Print();
      cout << endl;

      cout << "New ExpGraph Node" << endl;
      nextNode.Print();      
*/      
      return qNew;
    }
    else
    {
      addSucc = false;
      //cout << "current successor (new node) ignored" << endl;
    }
  }
  else
  {
    // This node is already present in nodeArr; existing Next Node
    node &exNextNode = nodeArr[newNodeIdx];
    
    costVec gV = qCur.gValue + edgeCost;
    costVec hV = exNextNode.hEst;
    costVec fV = gV + hV;
    
    if(!IsDominatedBySolPathQArr(fV) && !IsDominatedByNodePath(newNodeIdx, gV))
    {
      // Remove the costs which are dominated by gV
      PruneNodePaths(newNodeIdx, gV);

      // Add the path in the path array
      exNextNode.lastPathIndex++;
      pathEntry pCur(nodeArr[nIdx].index, qCur.pathIndex, gV);
      exNextNode.pathArr.push_back(pCur);

      // Push the corresponding entry in OPEN
      openEntry qNew(exNextNode.index, exNextNode.lastPathIndex, gV, fV);
      
/*      
      cout << "Added to Open" << endl;
      qNew.Print();
      cout << endl;
      
      cout << "Updated ExpGraph Node" << endl;
      exNextNode.Print();
*/      
      
      return qNew;      
    }
    else
    {
      addSucc = false;
      //cout << "current successor (node exists) skipped" << endl;
      //cout << "Existing ExpGraph Node" << endl;
      //exNextNode.Print();      
    }
  }

  openEntry qTemp;
  return qTemp;
}

/*
openEntry gridSearch::SelectBestFromAllLevels(size_t sIdx, bool &exist, size_t &level) 
{
  size_t i = 0;
  openEntry best;
  while((openArr[sIdx][i].empty() == true || suspendFlag[sIdx][i] == true) && i < numRange) 
  {
    i++;
    //~ cout<<i<<endl;
  }
  if(i >= numRange) 
  {
    exist = false;
      //~ cout<<"returning false best"<<endl;
    return best;
  }
  
  //~ cout<<"stack index = "<<sIdx<<" i = "<<i<<endl;
  best = openArr[sIdx][i].front();
  list<openEntry>::iterator bestIt = openArr[sIdx][i].begin();
  level = i;
  //~ cout<<"after i"<<endl;
  for(; i< depthBound; i++) 
  {
	if(openArr[sIdx][i].empty() == false) 
	{
      list<openEntry> open1 = openArr[sIdx][i];
      list<openEntry>::iterator it;
      //~ cout<<"i="<<i<<endl;
      for(it = open1.begin(); it != open1.end(); it++) 
      {
        if( it->fValue < best.fValue) 
        {
			//~ cout<<"found best"<<endl;
          best = *it;
          bestIt = it;
          level = i;
        }
      }  
    }
  }
    //~ cout<<"returning best before"<<endl;
  openArr[sIdx][level].remove(best);
  exist = true;
  //~ cout<<"returning best"<<endl;
  return best;
}*/

bool gridSearch::SolnPathEmpty(size_t sIdx) {
  return (false == solPathQArr[sIdx].empty());
}

bool gridSearch::CallBack(openEntry qCur, size_t lvl) {
  size_t nIdx = qCur.nodeIndex;
  assert(qCur.nodeIndex < nodeArr.size());
  if(IsGoalNode(nodeArr[nIdx]))
    {
      // If it is a goal node then update the solution frontier accordingly.
      // hEst contains the exact cost remaining.
      // The fValue entry of qCur is the cost of this goal.
      goalReached = true;
      curGoalEntry = qCur;  

      bool solFrontierChanged(false);
      size_t sIdx = ComputeStackIndex(qCur.fValue);
      solFrontierChanged = UpdateSolutionFrontier(sIdx);
      if(true == solFrontierChanged)
      {
        openArr->Prune(*this, qCur, sIdx);
        UpdateSFQualityOutput();
        //~ return true;
      }
      //~ return false;
      return false;
    }
 
    //cout << "Expanding ExpGraph Node" << endl;
    //nodeArr[nIdx].Print();
    vector<size_t> coordinates;
    bool added(false);
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
        iDimSucc = GetNextEGNodeACTR(qCur, coordinates, edgeCost, addiDimSucc);
      }
      else 
      {
        addiDimSucc = false;
      }
    
      if(addiDimSucc) 
      {
        //~ AddToOpenArr(iDimSucc, level + 1);
        openArr->add(iDimSucc, lvl + 1);
        added = true;
      }
    }
    
    return added;
    //~ return false;
    /*if(added) 
    {
      expCountRange[sIdx]++;
      expCount[sIdx][level]++;
      if(expCount[sIdx][level] >= lim[sIdx][level]) 
      {
        suspendFlag[sIdx][level] = true;
      }
    }*/ 
}

void gridSearch::AnytimeContractSearchStack(size_t sIdx)
{
	//~ cout<<"in stack "<<sIdx<<endl;
  /*do 
  {
    bool exist(false);
    size_t level(0);
    openEntry qCur = SelectBestFromAllLevels(sIdx, exist, level);
        //~ cout<<"exist = "<<exist<<endl;
    if(exist == false) 
    {
      return;
    }

    assert(qCur.nodeIndex < nodeArr.size());
    size_t nIdx = qCur.nodeIndex;
    if(IsGoalNode(nodeArr[nIdx]))
    {
      // If it is a goal node then update the solution frontier accordingly.
      // hEst contains the exact cost remaining.
      // The fValue entry of qCur is the cost of this goal.
      goalReached = true;
      curGoalEntry = qCur;  

      bool solFrontierChanged(false);
      solFrontierChanged = UpdateSolutionFrontier(sIdx);
      if(true == solFrontierChanged)
      {
        PruneOpenArr(sIdx);
        UpdateSFQualityOutput();
      }
      return;
    }
 
    //cout << "Expanding ExpGraph Node" << endl;
    //nodeArr[nIdx].Print();
    vector<size_t> coordinates;
    bool added(false);
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
        iDimSucc = GetNextEGNodeACTR(qCur, coordinates, edgeCost, addiDimSucc);
      }
      else 
      {
        addiDimSucc = false;
      }
    
      if(addiDimSucc) 
      {
        AddToOpenArr(iDimSucc, level + 1);
        added = true;
      }
    }
    if(added) 
    {
      expCountRange[sIdx]++;
      expCount[sIdx][level]++;
      if(expCount[sIdx][level] >= lim[sIdx][level]) 
      {
        suspendFlag[sIdx][level] = true;
      }
    } 
  }while(true);
  cout<<"returning"<<endl;*/
}

void gridSearch::Normalize(vector<double> &r) 
{
  double sum(0.0);
  for(size_t i = 0; i<r.size(); i++) 
  {
    sum += r[i];
  }
  for(size_t i = 0; i<r.size(); i++) 
  {
    if(sum > 0 ) 
    {
      r[i] = r[i]/sum;
    }
    else 
    {
      r[i] = 1/r.size();
    }
  }
  
}
void gridSearch::AdjustContract(int newContract) {
  //~ cout<<"in adjust contract"<<endl;
 /* vector<double> expR;
  vector<vector<double> > WRR;
  for(size_t i = 0; i<numRange; i++) 
  {
    expR.push_back(d[i]+expCountRange[i]);
    vector <double> WRRi;
    for(size_t j = 0; j<depthBound; j++) 
    {
      WRRi.push_back(lim[i][j]+expCount[i][j]);
    }
    Normalize(WRRi);
    WRR.push_back(WRRi);
  }
  Normalize(expR);
  for(size_t i = 0; i < numRange ; i++) 
  {
    d[i] = expR[i]*newContract;
        //~ cout << "d["<<i<<"]="<<d[i]<<endl;
    //~ cout<<"Range  "<<i<<" d = "<<d[i]<<endl;
    for(size_t j = 0; j<depthBound; j++) 
    {
      lim[i][j] = WRR[i][j]*d[i];
      //~ cout<<"\t WRR = "<<WRR[i][j]<<" lim "<<j<<" = "<<lim[i][j]<<" "<<suspendFlag[i][j]<<endl;
      suspendFlag[i][j] = false;
      //~ cout << "lim["<<i<<"]["<<j<<"]="<<lim[i][j]<<endl;
      
    }
  }*/
}

double gridSearch::CalculateMinContract(double step) {
	
	numRange = ceil(probInstance.dimMax * (probInstance.maxEdgeCost - probInstance.minEdgeCost) / step);
	
	MINC = depthBound;
	
	for(size_t i = 1; i <= numDim; i++) {
		MINC = numRange*(2*MINC + m*(numRange - 1))/2;
	}
	return MINC;
}



void gridSearch::InitializeContract() {
  /*m = 1;
  alpha = 1;
  beta = 1;
  MINC = (numRange)*depthBound + (numRange)*(numRange - 1)*m/2;
  MAXC = 10000*MINC;
  for(size_t i = 0; i<numRange; i++) {
    d[i] = MINC/(numRange) + m*(numRange - 1 - 2*i)/2; 
    //~ cout << "d["<<i<<"]="<<d[i];
    for(size_t j = 0; j<depthBound; j++) {
      lim[i][j] = d[i]/depthBound;
    }
  }*/
}

void gridSearch::SearchContract(double L1Inc) {
  assert(L1Inc > 0);
  numRange = ceill((probInstance.pathCostMax - probInstance.pathCostMin) / L1Inc);
  rangeArr.push_back(probInstance.pathCostMin);
      //~ numRange = ceil(probInstance.dimMax * (probInstance.maxEdgeCost - probInstance.minEdgeCost) / L1Inc);

  cout << " num range = "<<numRange<<endl;

  //~ depthBound = 2 * xDimMax * yDimMax; // can it be L1Inc * yDimMax=?
    //~ cout<<" Num range = "<<numRange<<" depth bound = "<<depthBound<<endl;
  //~ depthBound = numDim  * pow(dimMax, numDim) + 1; //latest
  cout <<" depthBound = "<<depthBound<<endl;
  	depthBound = numDim * pow(dimMax, numDim) + 1;
  	//~ cout << "depth bound = "<<depthBound<<endl;
  openArr = new mystack(numDim, L1Inc, probInstance, depthBound);
  for(size_t i = 0; i < numRange; i++)
  {
    double rEntry = rangeArr[i] + L1Inc;
    rangeArr.push_back(rEntry);
    //~ cout <<"rentry = "<<rEntry<<endl;
    //~ vector<list<openEntry> > openstackTmp;
    //~ vector<size_t> tmpExpCnt, tmpLim;
    //~ vector<bool> tmpSus;
     
    //~ for(size_t j = 0; j< depthBound; j++) 
    //~ {
      //~ list<openEntry> tmpOpen;
      //~ openstackTmp.push_back(tmpOpen);  
      //~ tmpExpCnt.push_back(0);
      //~ tmpLim.push_back(0);
      //~ tmpSus.push_back(false);
    //~ }
    //~ expCount.push_back(tmpExpCnt);
    //~ expCountRange.push_back(0);
    //~ suspendFlag.push_back(tmpSus);
    //~ d.push_back(0);
    //~ lim.push_back(tmpLim);
    //~ openArr.push_back(openstackTmp);
    
    list <openEntry> stackTmp;
    solPathQArr.push_back(stackTmp);
  }
  //~ rangeArr[numRange] = probInstance.pathCostMax;

  gettimeofday(&tvStartBB, NULL);

  // costVec is initialized at (0,0)
  costVec gVal;
  vector<size_t> coordinates;
  for(size_t i = 0; i < numDim; i++) 
  {
    coordinates.push_back(0);
  }
  costVec hVal = probInstance.GetVertexHVal(coordinates);
  costVec fVal;

  // Construct the root node of the explicit graph
  lastNodeIndex = 0;
  node n(coordinates);
  n.SetIndex(lastNodeIndex);
  n.SetHVal(hVal);
  n.lastPathIndex = 0;

  // Push the node in nodeArr
  nodeArr.push_back(n);
  
  fVal = gVal + hVal;

  // Push the corresponding entry to the first stack
  openEntry q(lastNodeIndex, 0, gVal, fVal);  
  //~ openArr[0][0].push_back(q);
  openArr->add(q, 0);
  double contract = CalculateMinContract(L1Inc);
  openArr->InitializeContract(contract);
  //Search each of the stack in a round-robin fashion
  bool continueFlag(false);
  //~ size_t roundIndex(1);
  //~ int contract(MINC);
  do {
    //Reset before every round-robin search
    continueFlag = false;
/*
    for(size_t i = 0; i < numRange; i++)
    {
      for(size_t j = 0; j < depthBound; j++) 
      {
        if(false == openArr[i][j].empty())
        {
          //~ cout << "Anytime Contract "<<i<<endl;
          AnytimeContractSearchStack(i);
          break;
        }
      }
    }*/
    
  for(size_t i = 0; i < numRange; i++) {
	  //~ cout << "\n i = "<<i<<endl<<endl;
	  bool empty = openArr->IsEmpty(i);
	  if(!empty) {
	  //~ cout << " stack "<<i << " is not empty"<<endl;
      openArr->AnytimeContractSearchStack(*this, i);
	  } else {
	    //~ cout << "stack "<<i<<" is empty"<<endl;
	  }
	  empty = openArr->IsEmpty(i);
	  
	  if(!empty) {
	    continueFlag = true;
	  }
	}
	
    contract += MINC;
    openArr->AdjustContract(contract);
    
    /*for(size_t i = 0; i < numRange; i++)
    {
      for(size_t j = 0; j<depthBound;j++) 
      {
        if(false == openArr[i][j].empty())
        {
          continueFlag = true;
          roundIndex++;
          break;
        }
      }
      if(continueFlag == true) 
      {
        break;
      }
    }*/
  //~ cout<<"here"<<endl;
  }while(true == continueFlag);
  
  for(size_t i = 0; i < numRange; i++)
  {
    if(false == solPathQArr[i].empty())
    {
      solPathQArr[i].sort();
    }
  }
  //~ cout<<"here2"<<endl;
}
/*
void gridSearch::SearchContract(double L1Inc) {
  assert(L1Inc > 0);
  numRange = ceill((probInstance.pathCostMax - probInstance.pathCostMin) / L1Inc);
  rangeArr.push_back(probInstance.pathCostMin);


  //~ depthBound = 2 * xDimMax * yDimMax; // can it be L1Inc * yDimMax=?
    //~ cout<<" Num range = "<<numRange<<" depth bound = "<<depthBound<<endl;
  depthBound = numDim  * pow(dimMax, numDim) + 1;
  //~ cout <<" depthBound = "<<depthBound<<endl;
  for(size_t i = 0; i < numRange; i++)
  {
    double rEntry = rangeArr[i] + L1Inc;
    rangeArr.push_back(rEntry);
    //~ cout <<"rentry = "<<rEntry<<endl;
    vector<list<openEntry> > openstackTmp;
    vector<size_t> tmpExpCnt, tmpLim;
    vector<bool> tmpSus;
    
    for(size_t j = 0; j< depthBound; j++) 
    {
      list<openEntry> tmpOpen;
      openstackTmp.push_back(tmpOpen);  
      tmpExpCnt.push_back(0);
      tmpLim.push_back(0);
      tmpSus.push_back(false);
    }
    expCount.push_back(tmpExpCnt);
    expCountRange.push_back(0);
    suspendFlag.push_back(tmpSus);
    d.push_back(0);
    lim.push_back(tmpLim);
    openArr.push_back(openstackTmp);
    
    list <openEntry> stackTmp;
    solPathQArr.push_back(stackTmp);
  }
  rangeArr[numRange] = probInstance.pathCostMax;

  gettimeofday(&tvStartBB, NULL);

  // costVec is initialized at (0,0)
  costVec gVal;
  vector<size_t> coordinates;
  for(size_t i = 0; i < numDim; i++) 
  {
    coordinates.push_back(0);
  }
  costVec hVal = probInstance.GetVertexHVal(coordinates);
  costVec fVal;

  // Construct the root node of the explicit graph
  lastNodeIndex = 0;
  node n(coordinates);
  n.SetIndex(lastNodeIndex);
  n.SetHVal(hVal);
  n.lastPathIndex = 0;

  // Push the node in nodeArr
  nodeArr.push_back(n);
  
  fVal = gVal + hVal;

  // Push the corresponding entry to the first stack
  openEntry q(lastNodeIndex, 0, gVal, fVal);  
  openArr[0][0].push_back(q);

  InitializeContract();
  //Search each of the stack in a round-robin fashion
  bool continueFlag(false);
  size_t roundIndex(1);
  int contract(MINC);
  do{
    //Reset before every round-robin search
    continueFlag = false;

    for(size_t i = 0; i < numRange; i++)
    {
      for(size_t j = 0; j< depthBound; j++) 
      {
        if(false == openArr[i][j].empty())
        {
          //~ cout << "Anytime Contract "<<i<<endl;
          AnytimeContractSearchStack(i);
          break;
        }
      }
    }
    contract += MINC;
    AdjustContract(contract);
    for(size_t i = 0; i < numRange; i++)
    {
      for(size_t j = 0; j<depthBound;j++) 
      {
        if(false == openArr[i][j].empty())
        {
          continueFlag = true;
          roundIndex++;
          break;
        }
      }
      if(continueFlag == true) 
      {
        break;
      }
    }
  //~ cout<<"here"<<endl;
  }while(true == continueFlag);
  
  for(size_t i = 0; i < numRange; i++)
  {
    if(false == solPathQArr[i].empty())
    {
      solPathQArr[i].sort();
    }
  }
  //~ cout<<"here2"<<endl;
}
*/
void gridSearch::PrintBBStack(size_t sIdx)
{
  cout << setfill('#') << setw(80) << "#" << setfill(' ') << endl;

  cout << "Contents of Stack : " << sIdx << endl;

  list<openEntry>::iterator it = stackArr[sIdx].begin();
  for(; it != stackArr[sIdx].end(); it++)
  {
    it->Print();
    cout << endl;
  }

  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
}


void gridSearch::PrintSolutionPathsBB()
{  
  cout << "Printing the soltuion paths" << endl;
  cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;

  for(size_t i = 0; i < numRange; i++)
  {
    if(false == solPathQArr[i].empty())
    {
      cout << "Printing the soltuion paths in range " << i << endl;
      cout << setfill('-') << setw(80) << "-" << setfill(' ') << endl;
      PrintSolutionPathsBB(i);
    }
  }
  cout << setfill('=') << setw(80) << "=" << setfill(' ') << endl;
}


void gridSearch::PrintSolutionPathsBB(size_t sIdx)
{  
  static size_t solPathCnt = 0;

  if(true == solPathQArr[sIdx].empty())
  {
    cout << "solPathQArr[" << sIdx << "] is empty" << endl;
    return;
  }

  list<openEntry>::iterator it = solPathQArr[sIdx].begin();  
  for(; it != solPathQArr[sIdx].end(); solPathCnt++, it++)
  {
    //~ openEntry tmpGoalEntry = *it;
    cout << "Solution Number " << setw(4) << solPathCnt << " : ";
    it->Print();
    cout << endl;
    //ReportGoal(tmpGoalEntry);
  }
}


void gridSearch::UpdateSFQualityOutput()
{
  ComputeCurSFHV();
  double SFQuality = ComputeABSQuality();
 
  if((true == reportSFQuality) || (targetSFQuality <= SFQuality))
  {
    struct timeval tvCur;

    gettimeofday(&tvCur, NULL);
    
    unsigned diff = (tvCur.tv_sec * 1000000 + tvCur.tv_usec) -
                    (tvStartBB.tv_sec * 1000000 + tvStartBB.tv_usec);
    double timeD = ((double) diff) / 1000000.0;

    //cout << "Time elapsed : ";

    //cout << " optimality attained = ";
    //cout << setiosflags(ios::fixed) << setiosflags(ios::right)
    //     << setprecision(3) << setw(7) << SFQuality << "%"
    //     << " target = " << targetSFQuality;
    //cout << endl;

    if(true == reportSFQuality)
    {
      /*
      if(targetSFQuality <= SFQuality)
      {
        // When the initial solution exceeds the targetSFQuality
        execOut << ' ';
        execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
                << setprecision(3) << setw(7) << 0;
   
        execOut << ' ';
        execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
                << setprecision(3) << setw(7) << timeD;
        
        execOut << endl;
      }
      */

      reportSFQuality = false;
      size_t qIncStep = 0;    

      while(targetSFQuality <= SFQuality)
      {
        // Accounting the intermediate skipped intervals
        qIncStep++;
        /*
        if(qIncStep > 1)
        {
          execOut << ' ';
          execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
                  << setprecision(3) << setw(7) 
                  << 0;
          
          execOut << ' ';
          execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
                  << setprecision(3) << setw(7) 
                  << timeD;

          execOut << endl;
        }
        */
        targetSFQuality += DEFAULT_SF_Q_INC;
      }
    
      execOut << ' ';
      execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
              << setprecision(3) << setw(7)
              << targetSFQuality - DEFAULT_SF_Q_INC;

      execOut << ' ';
      execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
              << setprecision(3) << setw(7) << timeD;

      execOut << endl;
    }
    else
    {
      size_t qIncStep = 0;    

      while(targetSFQuality <= SFQuality)
      {
        // Accounting the intermediate skipped intervals
        qIncStep++;
        /*
        if(qIncStep > 1)
        {
          execOut << ' ';
          execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
                  << setprecision(3) << setw(7) 
                  << targetSFQuality - DEFAULT_SF_Q_INC;
          
          execOut << ' ';
          execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
                  << setprecision(3) << setw(7) 
                  << timeD;

          execOut << endl;
        }
        */
        targetSFQuality += DEFAULT_SF_Q_INC;
      }
      
      execOut << ' ';
      execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
              << setprecision(3) << setw(7)
              << targetSFQuality - DEFAULT_SF_Q_INC;

      execOut << ' ';
      execOut << setiosflags(ios::fixed) << setiosflags(ios::right)
              << setprecision(3) << setw(7) << timeD;

      execOut << endl;
    }
  }
}
