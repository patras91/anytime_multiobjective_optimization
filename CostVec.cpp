#include "CostVec.hpp"


costVec::costVec()
{
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    c[cid] = 0;
  }
}
  

costVec::costVec(const costVec &cv)
{
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    c[cid] = cv.c[cid];
  }
}


costVec& costVec::operator=(const costVec &cv)
{
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    c[cid] = cv.c[cid];
  }

  return *this;
}


costVec& costVec::operator+=(const costVec &cv)
{
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    c[cid] += cv.c[cid];
  }

  return *this;
}


const costVec costVec::operator+(const costVec &cv)
{
  costVec result(*this);

  result += cv;

  return result;
}


bool costVec::IsLessL1(const costVec &cv) const
{
  // Using hack for two dimension
  /*if(c[0] < cv.GetCostComponent(0))
    return true;
  else if((c[0] == cv.GetCostComponent(0)) && (c[1] < cv.GetCostComponent(1)))
    return true;
  else
    return false;
    */
  for(size_t i = 0; i < VEC_DIMENSION; i++) 
  {
	  if(c[i] < cv.GetCostComponent(i)) {
	    return true;
	  } 
	  else if(c[i] > cv.GetCostComponent(i)) 
	  {
		return false;  
	  }
  }
  return false;
}


bool costVec::IsLessL2(const costVec &cv) const
{
  // Using hack for two dimension
  if(c[1] < cv.GetCostComponent(1))
    return true;
  else if((c[1] == cv.GetCostComponent(1)) && (c[0] < cv.GetCostComponent(0)))
    return true;
  else
    return false;
}


bool costVec::operator==(const costVec &cv) const
{
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    if(c[cid] != cv.GetCostComponent(cid)) 
      return false;
  }

  return true;
}


bool costVec::operator<(const costVec &cv) const
{
  // Using hack for two dimension
  /*if((c[0] <= cv.GetCostComponent(0)) && (c[1] <= cv.GetCostComponent(1)) && !((*this) == cv))
    return true;
  else
    return false;*/
    
  for(size_t i = 0; i < VEC_DIMENSION; i++) 
  {
    if(c[i] > cv.GetCostComponent(i)) 
    {
		return false;
	}
  }
  if(!((*this) == cv))
  {
	  return false;
  } 
  else 
  {
	  return true;
  }
}


bool costVec::operator<=(const costVec &cv) const
{
  // Using hack for two dimension
  /*if((c[0] <= cv.GetCostComponent(0)) && (c[1] <= cv.GetCostComponent(1)))
    return true;
  else
    return false;
    */
  for(size_t i = 0; i < VEC_DIMENSION; i++) 
  {
    if(c[i] > cv.GetCostComponent(i)) 
    {
		return false;
	}
  }
  return true;
}


double costVec::GetCostComponent(size_t cid) const
{
  assert(cid < VEC_DIMENSION);
  return c[cid];
}

double costVec::GetCostComponentInverse(size_t cid) const  //for mystack::CalculateRange()
{
  assert(cid < VEC_DIMENSION);
  return c[VEC_DIMENSION - 1 - cid];
}


void costVec::SetCostComponent(size_t cid, double val)
{
  assert(cid < VEC_DIMENSION);
  c[cid] = val;
}


void costVec::Print()
{
  cout << "(";
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    cout << setiosflags(ios::fixed) << setiosflags(ios::right)
         << setw(DBL_PRINT_WIDTH) << setprecision(DBL_PRINT_PRECISION)
         << c[cid];

    if(cid != VEC_DIMENSION - 1)
      cout << ",";
  }
  cout << ")";
}
  

void costVec::WriteToFile(ofstream &fout)
{
  assert(fout.is_open() && fout.good());
  
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    fout << ' ' << c[cid];
  }
}


void costVec::ReadFromFile(ifstream &fin)
{
  assert(fin.is_open() && fin.good());
  
  for(size_t cid = 0; cid < VEC_DIMENSION; cid++)
  {
    fin >> c[cid];
  }
}


bool costVec::Dominates(costVec cv, size_t k) 
{
  for(size_t i = k; i < VEC_DIMENSION; i++) 
  {
	  if(cv.GetCostComponent(i) > c[i])
	  {
		  return false;
	  }
  }
  for(size_t i = k; i < VEC_DIMENSION; i++) 
  {
	  if(cv.GetCostComponent(i) < c[i])
	  {
		  return true;
	  }
  }
  return false;
}
