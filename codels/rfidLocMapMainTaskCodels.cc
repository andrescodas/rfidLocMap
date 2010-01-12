/**
 ** rfidLocMapMainTaskCodels.cc
 **
 ** Codels called by execution task rfidLocMapMainTask
 **
 ** Author: 
 ** Date: 
 **
 **/

#include <portLib.h>

#include "server/rfidLocMapHeader.h"


/*------------------------------------------------------------------------
 *
 * rfidLocMapInit  --  Initialization codel (fIDS, ...)
 *
 * Description: 
 * 
 * Returns:    OK or ERROR
 */

STATUS
rfidLocMapInit(int *report)
{
  /* ... add your code here ... */
  return OK;
}

/*------------------------------------------------------------------------
 * ActualizePositions
 *
 * Description: 
 *
 * Reports:      OK
 *              S_rfidLocMap_RFID_POSTER_NOT_FOUND
 *              S_rfidLocMap_RFLEX_POSTER_NOT_FOUND
 */

/* rfidLocMapActualizePositionsStart  -  codel START of ActualizePositions
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapActualizePositionsStart(int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/* rfidLocMapActualizePositionsInter  -  codel INTER of ActualizePositions
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapActualizePositionsInter(int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/*------------------------------------------------------------------------
 * StartRobotParticles
 *
 * Description: 
 *
 * Reports:      OK
 */

/* rfidLocMapStartRobotParticlesStart  -  codel START of StartRobotParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapStartRobotParticlesStart(POSITION *position, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/* rfidLocMapStartRobotParticlesInter  -  codel INTER of StartRobotParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapStartRobotParticlesInter(POSITION *position, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/*------------------------------------------------------------------------
 * SetNumberRobotParticles
 *
 * Description: 
 *
 * Reports:      OK
 */

/* rfidLocMapSetNumberRobotParticlesStart  -  codel START of SetNumberRobotParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetNumberRobotParticlesStart(int *numberParticles, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/* rfidLocMapSetNumberRobotParticlesInter  -  codel INTER of SetNumberRobotParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetNumberRobotParticlesInter(int *numberParticles, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/*------------------------------------------------------------------------
 * GetNumberRobotParticles
 *
 * Description: 
 *
 * Reports:      OK
 */

/* rfidLocMapGetNumberRobotParticlesStart  -  codel START of GetNumberRobotParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetNumberRobotParticlesStart(int *numberParticles, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

/* rfidLocMapGetNumberRobotParticlesInter  -  codel INTER of GetNumberRobotParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetNumberRobotParticlesInter(int *numberParticles, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}


