#include "mystack.hpp"
#include "GridSearch.hpp"

//~ void mystackComp::AnytimeContractSearchStack(GridSearch &GS, size_t sIdx) {}
//~ void mystackComp::AnytimeContractSearchStack(GridSearch &GS) {}

mystack::mystack(int l, int s, grid& probInstance, int dB) {
    level = l;
    step = s;
    double range = probInstance.numDim * (probInstance.dimMax - 1) * probInstance.minEdgeCost;
    numRange = ceil(probInstance.numDim * (probInstance.dimMax - 1) * (probInstance.maxEdgeCost - probInstance.minEdgeCost) / step);
    //~ cout <<"\t\t\t\t\t level = "<<l<<" \tnumRange = "<<numRange<<endl;
    //~ depthBound = dB;
    depthBound = 1;
    
    if(level == 0) {
       for(size_t i = 0; i < depthBound; i++) {
         mylist *elem = new mylist(0);
         sArr.push_back(elem);
         expCount.push_back(0);
         lim.push_back(0);
         suspend.push_back(0);
       }
       d = 0;
       //~ totalExpCount = 0;
     } else {
	   //~ cout <<" level = "<<level<<endl;
       for(size_t i = 0; i < numRange; i++) {
         mystack *elem = new mystack(level - 1, step, probInstance, dB);
         sArr.push_back(elem);
         rArr.push_back(range);
         //~ cout << "i = "<<i<< " range = "<<range<<endl;
         range += step;
         expCount.push_back(0);
         lim.push_back(0);
         //~ d.push_back(0);
         suspend.push_back(false); 
         
       }
       d = 0;
       //~ totalExpCount = 0;
       rArr.push_back(range);
     }
     
  }
  
int mystack::CalculateRange(openEntry open) {

  double c1Val = open.fValue.GetCostComponentInverse(level - 1);
  //~ cout << "c1Val = "<<c1Val<<" numrange = "<<numRange<<" rarr size = "<<rArr.size()<<endl;
	assert(c1Val < rArr[numRange]);
	size_t sIdx(1);
	for(; (sIdx < numRange) && (c1Val >= rArr[sIdx]) ; sIdx++)
	{}
	sIdx--;

	//~ cout <<"cost = "<<c1Val<<" sidx = "<<sIdx<<endl;
	return sIdx;
  }
  
  void mystack::add(openEntry entry, int l) {
	//~ cout << " IN ADD level = "<<level<<endl;
    if(level == 0) {
		
      //~ cout << "depthBound = "<<depthBound<< "\tadding  in level 0 : "<<l<<endl;
      //~ sArr[l]->add(entry);
      sArr[0]->add(entry);
      expCount[l]++;
      if(expCount[l] >= lim[l]) {
        suspend[l] = true;
      }
      return;
    }
    int idx = CalculateRange(entry);
    //~ cout << "level = "<<level<<" adding in stack "<<idx<<endl;
    sArr[idx]->add(entry, l);
    //~ cout << "added successfully "<<endl;
  
  }
  
  void mystack::print() {
    for (int i = 0; i < 100/step; i++) {
      //~ cout << "level = " << level << "stack no = "<<i<<endl;
      sArr[i]->print();
    }
  }
  
void mystack::InitializeContract(int contract) {
	
	d = contract;
	if(level == 0) {
		int limit = d/depthBound;
		for(size_t i = 0; i < depthBound; i++) {
		  lim[i] = limit;
		}
		return;
	}
	
	double L1 = d/numRange + m*(numRange - 1)/2;
	for(size_t i = 0; i < numRange; i++) {
		lim[i] = L1 - i*m;
		sArr[i]->InitializeContract(lim[i]);
	}
}

bool mystack::IsEmpty() {
  //~ cout <<" level = "<<level<<" numRange = "<<numRange<<endl;
  size_t range = (level == 0)?depthBound:numRange;
  for(size_t i = 0; i < range; i++) {
	//~ cout << " \t\t i = "<<i<<endl;
    if(!sArr[i]->IsEmpty()) {
      return false;
    } 
  }
  return true;
}

bool mystack::IsEmpty(int sIdx) {
  if(!sArr[sIdx]->IsEmpty()) {
    return false;
  } 
  
  return true;
}


void mystack::Normalize(vector<double> &r) 
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


void mystack::AdjustContract(int contract) {

  vector<double> WRR;
  
  d = contract;
  size_t range = (level == 0)?depthBound:numRange;
  
  for(size_t i = 0; i < range; i++) {
    WRR.push_back(lim[i] + expCount[i]);
  }
  Normalize(WRR);
  for(size_t i = 0; i < range; i++) {
    lim[i] = WRR[i]*d;
    if(level > 0) { 
      sArr[i]->AdjustContract(lim[i]);
    }
    suspend[i] = false;
  }
}

openEntry mystack::SelectBestFromAllLevels(bool &exist, size_t &lvl) 
{
  //~ cout << " in select best "<<endl;
  size_t i = 0;
  openEntry best;
  while(i < depthBound && (sArr[i]->IsEmpty() == true || suspend[i] == true)) 
  {
    i++;
    //~ cout << i <<endl;
  }
  //~ cout << "i = "<<endl;
  if(i >= depthBound) 
  {
    exist = false;
    //~ cout << " not exist "<<endl;
    return best;
  }
  
  best = sArr[i]->GetFront();
  list<openEntry>::iterator bestIt = sArr[i]->GetIterator();
  lvl = i;
  //~ cout<<"after i"<<endl;
  for(; i< depthBound; i++) 
  {
    if(sArr[i]->IsEmpty() == false) 
    {
      list<openEntry>::iterator it;
      //~ cout<<"i="<<i<<endl;
      for(it = sArr[i]->GetIterator(); it != sArr[i]->GetEnd(); it++) 
      {
        if( it->fValue < best.fValue) 
        {
          //~ cout<<"found best"<<endl;
          best = *it;
          bestIt = it;
          lvl = i;
        }
      }  
    }
  }
  //~ cout<<"returning best before"<<endl;
  sArr[lvl]->remove(best);
  exist = true;
  //~ cout<<"returning best"<<endl;
  return best;
}


void mystack::PruneRec(gridSearch& GS, openEntry curSol) {
  if(level == 0) {
    for(size_t i = 0; i < depthBound; i++) {
      list<openEntry>::iterator it = sArr[i]->GetIterator();
        for(; it != sArr[i]->GetEnd();)
        {
          // Prune Open
          if(curSol.fValue < it->fValue)
          {
            openEntry qTmp = *it;
            it = sArr[i]->Erase(it); 
            GS.ErasePath(qTmp);
          }
          else
          {
            it++;
          }
        }
    }
    return;
  }
  
  for(size_t i = 0; i < numRange; i++) {
    sArr[i]->PruneRec(GS, curSol);
  }
}
void mystack::Prune(gridSearch& GS, openEntry curSol, size_t sIdxCur) {
  for(size_t sIdx = sIdxCur; sIdx < numRange; sIdx++)
  {
    if(GS.SolnPathEmpty(sIdx))
    {
      sArr[sIdx]->PruneRec(GS, curSol);
    }   
  }
}


size_t mystack::AnytimeContractSearchStack(gridSearch& GS) {
  //search in a round robin manner in this step as well
  if(level > 0) {
    size_t totalCount(0);
    for(size_t i = 0; i < numRange; i++) {
      size_t count = sArr[i]->AnytimeContractSearchStack(GS);
      expCount[i] += count;
      totalCount += count;
      //~ totalExpCount += count;
      if(expCount[i] >= lim[i]) {
        suspend[i] = true;
      }
    }
    return totalCount;
  }
  
  size_t count(0);
  do {
    bool exist(false);
    size_t lvl(0);
    openEntry qCur = SelectBestFromAllLevels(exist, lvl);
    if(!exist) 
    {
      return count;
    }
    
    bool added = GS.CallBack(qCur, lvl);
    
    if(added) { 
      //update expcount and suspend
      //~ size_t sIdx = CalculateRange(qCur);
      expCount[lvl]++;
      count++;
      //~ totalExpCount++;
      if(expCount[lvl] >= lim[lvl]) {
        suspend[lvl] = true;
      }
    }
    
    //~ if(prune) {
      //~ Prune(qCur);
    //~ }
    
  } while(true);
  return count;
}


void mystack::AnytimeContractSearchStack(gridSearch &GS, size_t sIdx) {
  
  //search in a round robin manner in this step as well
  sArr[sIdx]->AnytimeContractSearchStack(GS);
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

