#include "Grid.hpp"
#include <math.h>

//  {
//    cerr << "At File: " << __FILE__ << "::Line " << __LINE__
//         << "> Function: " << __FUNCTION__
//         << endl;
//    cerr << "Invalid File Name" << endl;
//    exit(0);
//  }


vertex::vertex()
{
  idx = 0;
  
  for(size_t i = 0; i < MAX_DIM;  i++) 
  {
	  Cds.push_back(0);
	  parents[i]  = false;
      children[i] = false;
  }
}


vertex::vertex(const vertex &v)
{
  idx = v.idx;
  
  for(size_t i = 0; i < v.Cds.size();  i++) 
  {
	  Cds.push_back(v.Cds[i]);
	  parents[i]  = v.parents[i];
      children[i] = v.children[i];
      
	  for(size_t i = 0; i < MAX_DIM; i++)
      {
        edgeCostIn[i] = v.edgeCostIn[i];
        edgeCostOut[i] = v.edgeCostOut[i];
      }
  }
  hVal = v.hVal;
}

/*
vertex::vertex(size_t index, size_t x, size_t y)
{
  idx = index;
  //~ xCd = x;
  //~ yCd = y;

  parents[XDIM] = false;
  parents[YDIM] = false;
  children[XDIM] = false;
  children[YDIM] = false;
}
*/

vertex::vertex(size_t index, vector<size_t> &cds) 
{
	idx = index;
	for(size_t i = 0; i < cds.size(); i++) 
	{
		Cds.push_back(cds[i]);
		parents[i] = false;
		children[i] = false;
	}
}


vertex& vertex::operator=(const vertex &v)
{
  idx = v.idx;
  
  for(size_t i = 0; i < v.Cds.size();  i++) 
  {
	  Cds.push_back(v.Cds[i]);
	  parents[i]  = v.parents[i];
      children[i] = v.children[i];
      
	  for(size_t i = 0; i < MAX_DIM; i++)
      {
        edgeCostIn[i] = v.edgeCostIn[i];
        edgeCostOut[i] = v.edgeCostOut[i];
      }
  }
  hVal = v.hVal;

  return (*this);
}


void vertex::Print()
{
  //~ cout << "Vertex "  << idx << " : ["
       //~ << xCd << ", " << yCd << "]" << endl;
  /*
  if(parents[XDIM] || parents[YDIM])
  {
    cout << "Incoming edges :";
    if(parents[XDIM])
    {
      //~ cout << "[From (" << xCd-1 << ", " << yCd << ") : ";
      edgeCostIn[XDIM].Print();
      //~ cout << "]";
    }
    if(parents[YDIM])
    {
      //~ cout << "[From (" << xCd << ", " << yCd-1 << ") : ";
      edgeCostIn[YDIM].Print();
      //~ cout << "]";
    }
    cout << endl;
  }

  if(children[XDIM] || children[YDIM])
  {
    cout << "Outgoing edges : ";
    if(children[XDIM])
    {
      //~ cout << "[To (" << xCd+1 << ", " << yCd << ") : ";
      edgeCostOut[XDIM].Print();
      //~ cout << "]";
    }
    if(children[YDIM])
    {
      //~ cout << "[To (" << xCd << ", " << yCd+1 << ") : ";
      edgeCostOut[YDIM].Print();
      //~ cout << "]";
    }
    cout << endl;
  }

  cout << "Heuristic Estimate : ";
  hVal.Print();
  cout << endl;*/
}


void vertex::WriteToFile(ofstream &fout)
{
  assert(fout.is_open() && fout.good());
  
  fout << ' ' << idx;
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
    fout << ' ' << Cds[i];
    fout << ' ' << parents[i];
    fout << ' ' << children[i];
    edgeCostIn[i].WriteToFile(fout);
    edgeCostOut[i].WriteToFile(fout);
  }
  hVal.WriteToFile(fout);
}


void vertex::ReadFromFile(ifstream &fin)
{
  assert(fin.is_open() && fin.good());
  
  fin >> idx;
  
  //~ cout << "Vertex index = "<<idx<<endl;
  for(size_t i = 0; i < MAX_DIM; i++) 
  {
	fin >> Cds[i];
	fin >> parents[i];
    fin >> children[i];
    //~ cout<<"Children along "<<i<<" "<<children[i]<<endl;
    edgeCostIn[i].ReadFromFile(fin);
    edgeCostOut[i].ReadFromFile(fin);
  }
  
  hVal.ReadFromFile(fin);
}
    //~ xDimMax(DEFAULT_X_DIM_MAX),
    //~ yDimMax(DEFAULT_Y_DIM_MAX),

grid::grid()
  :
    numDim(DEFAULT_NUM_DIM),
    dimMax(DEFAULT_DIM_MAX),    
    maxEdgeCost(EDGE_COST_MAX),
    minEdgeCost(EDGE_COST_MIN),
    lastVertexIndex(0)
{
	for(size_t i = 0; i < numDim; i++) 
	{
		startCd[i] = 0;
		endCd[i] = 0;
	} 
}


grid::grid(size_t numberOfDim, size_t dimensionMax, double maxEC)
{
  numDim = numberOfDim;
  dimMax = dimensionMax;

  for(size_t i = 0; i < numDim; i++) 
  {
	  startCd[i] = 0;
	  endCd[i] = dimMax - 1;
  }

  maxEdgeCost = (maxEC > EDGE_COST_MIN) ? maxEC : EDGE_COST_MAX;
  minEdgeCost = EDGE_COST_MIN;

  lastVertexIndex = 0;
}


void grid::ComputeRanges()
{
  pathLength = (dimMax - 1)*numDim;
  pathCostMin = pathLength * minEdgeCost;
  pathCostMax = pathLength * maxEdgeCost;
}

/*
costVec grid::GetVertexHVal(size_t xCd, size_t yCd)
{
  size_t vIdx = yCd * xDimMax + xCd;

  assert(vIdx < vertexArr.size());
  return vertexArr[vIdx].hVal;
}*/

size_t grid::CalculateIndex(vector<size_t> &Cds) 
{
	size_t index = Cds[0];
	size_t max = dimMax;
	for(size_t i = 1; i < numDim; i++) 
	{
		index = index*max + Cds[i];
	}
	return index; 
}

costVec grid::GetVertexHVal(vector<size_t> &Cds)
{
  size_t vIdx = CalculateIndex(Cds);

  assert(vIdx < vertexArr.size());
  return vertexArr[vIdx].hVal;
}

/*

costVec grid::GetOutEdgeCost(size_t xCd, size_t yCd, size_t dim)
{
  size_t vIdx = yCd * xDimMax + xCd;

  assert(vIdx < vertexArr.size());
  assert(dim == XDIM || dim == YDIM);

  assert(vertexArr[vIdx].children[dim] == true);

  return vertexArr[vIdx].edgeCostOut[dim];
}*/

costVec grid::GetOutEdgeCost(vector<size_t> &Cds, size_t dim)
{
  size_t vIdx = CalculateIndex(Cds);

  assert(vIdx < vertexArr.size());
  assert(dim <= numDim - 1);
  /*cout<<endl;
  
  
  for(int i = 0; i < numDim; i++)
  {
	  cout << Cds[i]<<",";
  }*/

  //~ cout << endl<<"child of "<<vIdx<<" along "<<dim<<endl;
  assert(vertexArr[vIdx].children[dim] == true);

  return vertexArr[vIdx].edgeCostOut[dim];
}


void grid::GetSuccessor(vector<size_t> &current) 
{
	/*vector<size_t> successor;
	for(size_t i = 0; i < current.size(); i++) 
	{
		successor.push_back(current[i]);
	}
	*/
		/*		cout << "\n";
			for(size_t j = 0; j < numDim; j++) 
			{
				cout << current[j] << ",";
			}
			cout << "\n";*/
	for(int i = current.size() - 1; i >= 0; i--) 
	{
		//~ cout << " i = "<<i<<endl;
		if(current[i] < dimMax - 1) 
		{
			current[i]++;
			for(size_t j = i+1; j < current.size(); j++) 
			{
				current[j] = 0;
			}
			/*cout << "\n";
			for(size_t j = 0; j < numDim; j++) 
			{
				cout << current[j] << ",";
			}
			cout << "\n";*/
			return;
		}
	}
}


size_t grid::GetParentIndex(vector<size_t> &Cds, size_t dim) 
{
	size_t index = 0;
	//~ size_t index = Cds[0];
	size_t max = dimMax;
	for(size_t i = 0; i < numDim; i++) 
	{
	    size_t j = (i==dim)?(Cds[i]-1):Cds[i];
		index = index*max + j;
	}
	return index; 
}


void grid::ConstructGrid()
{

  vector<size_t> Cds;
  for(size_t i = 0; i < numDim; i++) 
  {
	  Cds.push_back(0);
  }
  //size_t count = 1;
  //~ cout << "Cds size = "<<Cds.size()<<"\n";
  while(lastVertexIndex < pow(dimMax, numDim)) 
  {

    vertex tmpV(lastVertexIndex, Cds);  
	  
	for(size_t i = 0; i < numDim; i++) 
	{
	  //~ cout << "\n last vertex index = "<<lastVertexIndex;
	  if(Cds[i] > 0) 
	  {
	    tmpV.parents[i] = true;
		costVec c;
		  
		for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
        {
          double tu = GenNumInRange(maxEdgeCost, minEdgeCost);
          c.SetCostComponent(cid, (double)tu);
        }
         
        tmpV.edgeCostIn[i] = c;
          
        size_t pIdx = GetParentIndex(Cds, i);
        /*cout << " Parent of ";
        for(size_t m = 0; m < numDim; m++) 
        {
			cout << Cds[m]<< "," ;
		}
		cout <<" along "<<i<<" is "<< pIdx;*/
        vertexArr[pIdx].edgeCostOut[i] = c;
	 }
     if(Cds[i] < dimMax - 1) 
     {
	   tmpV.children[i] = true;
	   //~ cout << "\n possible children along "<<i;
	 }
    }	  
	 double cost = 0;
	 for(size_t j = 0; j < numDim; j++) 
	 {
	   cost += dimMax - 1 - Cds[j];
	 }
	 for(size_t j = 0; j < numDim; j++) 
	 {
	   tmpV.hVal.SetCostComponent(j, cost);
	 }
	 	 //~ cout << "here before pushing \n";
	 vertexArr.push_back(tmpV);
	 lastVertexIndex++;
	 GetSuccessor(Cds);

  }
  lastVertexIndex--;
  ComputeRanges();
  
  //Print();
}


void grid::WriteToFile(ofstream &fout)
{
  assert(fout.is_open() && fout.good());
  
  //~ fout << ' ' << xDimMax;
  //~ fout << ' ' << yDimMax;
  fout << ' ' << numDim;
  fout << ' ' << dimMax;
  fout << ' ' << maxEdgeCost;
  fout << ' ' << minEdgeCost;
  for(size_t i = 0; i < numDim; i++) 
  {
    //~ fout << "start " <<i<<" : "<< startCd[i];
    //~ fout << "end " << i <<" : "<< endCd[i];
    fout << ' ' << startCd[i];
    fout << ' ' << endCd[i];
  }
  //~ fout << ' ' << startCd[XDIM];
  //~ fout << ' ' << startCd[YDIM];
  //~ fout << ' ' << endCd[XDIM];
  //~ fout << ' ' << endCd[YDIM];
  fout << ' ' << maxEdgeCost;
  fout << ' ' << minEdgeCost;
  fout << ' ' << pathLength;
  fout << ' ' << pathCostMin;
  fout << ' ' << pathCostMax;
  fout << ' ' << lastVertexIndex;

  for(size_t i = 0; i < vertexArr.size(); i++)
  {
    vertexArr[i].WriteToFile(fout);
  }
}


void grid::ReadFromFile(ifstream &fin)
{
  assert(fin.is_open() && fin.good());
  
  //~ fin >> xDimMax;
  //~ fin >> yDimMax;
  fin >> numDim;
  fin >> dimMax;
  fin >> maxEdgeCost;
  fin >> minEdgeCost;
  
  for(size_t i = 0; i < numDim; i++) 
  {
	  fin >> startCd[i];
	  fin >> endCd[i];
  }
  //~ fin >> startCd[XDIM];
  //~ fin >> startCd[YDIM];
  //~ fin >> endCd[XDIM];
  //~ fin >> endCd[YDIM];
  fin >> maxEdgeCost;
  fin >> minEdgeCost;
  fin >> pathLength;
  fin >> pathCostMin;
  fin >> pathCostMax;
  fin >> lastVertexIndex;

  for(size_t i = 0; i <= lastVertexIndex; i++)
  {
    vertex vTmp;
    vTmp.ReadFromFile(fin);
    vertexArr.push_back(vTmp);
  }
}


void grid::Print()
{
  cout << "Displaying the constructed grid" << endl;
  cout << "NumDim, DimMax : [" << numDim << ", "
       << dimMax << "]\n" << endl;

  for(size_t i = 0; i <= lastVertexIndex; i++)
  {
    vertexArr[i].Print();
    cout << endl; 
  }
}
