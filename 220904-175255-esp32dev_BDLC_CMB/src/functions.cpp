#include "headers.h"

float avgNoZero(float *arr, int size)
{
  float sum=0;
  int summed=0;
  for(int i=0;i<size;i++)
  {
    if(arr[i]!=0)
    {
      sum+=arr[i];
      summed++;
    }
  }
  if(summed>0)
    return sum/summed;
  else
    return 0;
}

int radiansToImpulses(float rad)
{
  return rad*ENCODER_PULSES_PER_REVOLUTION/(2*PI);
}

float impulsesToRadians(int impulses)
{
  return impulses*2*PI/ENCODER_PULSES_PER_REVOLUTION;
}
