#ifndef __EXT_LIB_C
#define __EXT_LIB_C

struct pidData
{
  int loopID; // Unique identifier to prevent multiple allocations for the same loop
  double dt;  // Sample time
  double kp, ki, kd;
  double integ;
  double prevError;
  double loLim, hiLim;
  int first;
};

#define NDATA 100
static struct pidData *pData[NDATA];  // Array of pointers to PID data sets
static int n = 0;  // Number of PIDs instantiated
static int initArray = 1; // Initialize array to NULL

int PIDCinit(int loopID, double dt, double kp, double ki, double kd, double loLim, double hiLim)
{
  if(initArray == 1)
  {
	  initArray = 0;  // Only do this once
	  for(int j = 0; j < NDATA; j++)
	  {
		  // Initialize Array to NULL
		  struct pidData *ppd = pData[j];
		  pData[j] = NULL;
	  }
  }
  
  if(loopID >= NDATA)
  {
	  printf("<PIDCinit> loopID too big, loopID = %d, NDATA = %d\n", loopID, NDATA);
	  exit(3);
  }
  
  struct pidData *pd = NULL;
  
  if(pData[loopID] != NULL)
  {
	  // This block is already allocated -- re-initialize it
	  pd = pData[loopID];
  }
  else
  {
	// Block not initialized yet, allocate memory for it
	n++; //Increment number of data sets
	//printf("i %d, n %d, NDATA %d\n", loopID, n, NDATA);
	pd = (struct pidData *)malloc(sizeof(struct pidData));
	pData[loopID] = pd;  // Store pointer to this data set
  }
  
  pd->loopID = loopID;
  pd->first = 1;  // 'true' so first execution can set previous value of error
  pd->kp = kp;
  pd->ki = ki;
  pd->kd = kd;
  pd->loLim = loLim;
  pd->hiLim = hiLim;
  pd->integ = 0.0;  // Initialize integrator
  pd->dt = dt;
  
  return loopID;
}

double PIDCstep(int i, double val, double setpoint)
{
  double mVal;  // Output (manipulated variable)
  double error = setpoint - val;
  double deriv;
	double pTerm, iTerm, dTerm;
  
  struct pidData *pd = pData[i];
  if(pd->first)
  {
    pd->first = 0;
    deriv = 0.0; // derivative is not defined yet
  }
  else
  {
    deriv = (error - pd->prevError) / pd->dt;
  }
  pd->prevError = error;
  pd->integ += error * pd->dt;
  // Apply limits keeping reset (integrator) windup in mind
  pTerm = pd->kp * error;
  iTerm = pd->ki * pd->integ;
  dTerm = pd->kd * deriv;
  mVal = pTerm + iTerm + dTerm;  // Trial value
  if(mVal > pd->hiLim)
  {
    if(pd->ki != 0.0)
    {
      iTerm = pd->hiLim - (pTerm + dTerm);
      if(iTerm < 0.0)iTerm = 0.0;
      pd->integ = iTerm / pd->ki;
    }
    mVal = pd->hiLim;    
  }
  else if(mVal < pd->loLim)
  {
    if(pd->ki != 0.0)
    {
      iTerm = (pTerm + dTerm) - pd->loLim;
      if(iTerm > 0.0)iTerm = 0.0;
      pd->integ = iTerm / pd->ki;
    }
    mVal = pd->loLim;
  }
  return mVal;
}
#endif