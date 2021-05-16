#include "Global.hpp"


unsigned GenNumInRange(unsigned max, unsigned min)
{
  if(max <= min)
  {
    cerr << "max min argument screwed up" << endl;
    exit(0);
  }
  
  unsigned num = random() % (max - min);
  num += min;

  assert((num >= min) && (num <= max));
  return num;
}


double GenNumInRange(double max, double min)
{
  if(max <= min)
  {
      cerr << "max min argument screwed up" << endl;
      exit(0);
  }

  double w = drand48();

  while ((w < MIN_DOUBLE_VAL) || (w > MAX_DOUBLE_VAL))
  {
    w = drand48();
  }

  double num = max - min;
  num *= w;
  num += min;
  num = round(num);

  assert((num >= min) && (num <= max));
  return num;
}
