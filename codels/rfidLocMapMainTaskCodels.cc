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
#include "tagDetection.h"
#include "input.h"
#include "RobotParticles.h"
#include "Tag.h"
#include "RobotMonteCarlo.h"
#include "TagMonteCarlo.h"
#include "../server/rfidLocMapHeader.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

int numberParticlesRobot = 200;
int numberParticlesTag = 1000;
double inertiaTag = 0.99;
double inertiaRobot = 0.95;
int step = 0;


RobotParticles robotParticles(numberParticlesRobot);
double old_odo[3]; // x y t
TagMap tagMap;
TagParticlesMap inferringTags;

/*------------------------------------------------------------------------
 *
 * rfidLocMapInit  --  Initialization codel (fIDS, ...)
 *
 * Description:
 *
 * Returns:    OK or ERROR
 */

STATUS rfidLocMapInit(int *report) {

	tagMap.clear();
	tagMap["e0040000c1b2fd01\0"] = Point2D(-4.07, -1); //tag: 0
	tagMap["e00400007cd7fc01\0"] = Point2D(-4.07, 1.84); //tag: 1
	tagMap["e0040000079efd01\0"] = Point2D(3, 1.5); //tag: 2
	tagMap["e004000076defc01\0"] = Point2D(00.50, 3.55); //tag: 3
//	tagMap["e0040000dab1fd01\0"] = Point2D(01.50, +00.50); //tag: 4
	tagMap["e0040000fa9afd01\0"] = Point2D(+02.50, -03.00); //tag: 5
	tagMap["e004000080ddfc01\0"] = Point2D(-0.84, -03.27); //tag: 6

	RobotParticle robotParticle(0,0,0,0);
	robotParticles = RobotParticles(robotParticle,numberParticlesRobot);

	if (initInput() != 0) {
		printf("Problems in input initialization\n");
	}

	SDI_F->position.xRob = 0;
	SDI_F->position.yRob = 0;
	SDI_F->position.theta = 0;
	SDI_F->tagsPosition.nbTags = 0;

	for (int i = 0; i < 6; ++i) {
		SDI_F->estimationError.position_cov[i] = 0;
	}

	initSensorModel();
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
ACTIVITY_EVENT rfidLocMapActualizePositionsStart(int *report) {


	double odo_position[3];
	double odo_cov[3][3];

	char outputFile[64];

	int k;

	TagDetectionSet tagDetectionSet;
	TagDetectionSet fixedTagsDetectionSet;
	TagDetectionSet inferringTagsDetectionSet;


	if (readRflex(odo_position, odo_cov) != 0) {
		return ETHER;
	}
	if (readRFID(&tagDetectionSet) != 0) {
		return ETHER;
	}

	step++;

	sortDetectionsByTagMap(&fixedTagsDetectionSet,&inferringTagsDetectionSet,&tagDetectionSet,&tagMap);

	locateRobot(&fixedTagsDetectionSet,odo_position,old_odo,odo_cov,&tagMap,&robotParticles,inertiaRobot,step);

	locateTags(&inferringTags,&robotParticles,&inferringTagsDetectionSet,inertiaTag,numberParticlesTag,step);

	robotParticles.estimatePosition(&(SDI_F->position.xRob),&(SDI_F->position.yRob),&(SDI_F->position.theta),(SDI_F->estimationError.position_cov));

	k = 0;
	for(TagParticlesMap::iterator iTagIt = inferringTags.begin(); iTagIt != inferringTags.end();iTagIt++){
		printf("currentTag = %s\n",iTagIt->first.c_str());
		strcpy(SDI_F->tagsPosition.tags[k].tagId.tagId,iTagIt->first.c_str());
		(iTagIt->second)->estimatePosition(
				&(SDI_F->tagsPosition.tags[k].tag_position.x),
				&(SDI_F->tagsPosition.tags[k].tag_position.y),
				SDI_F->tagsPosition.tags[k].tag_position_error.tag_position_cov);

		printf("TagId = %s, x = %lf, y = %lf\n",SDI_F->tagsPosition.tags[k].tagId.tagId,SDI_F->tagsPosition.tags[k].tag_position.x,SDI_F->tagsPosition.tags[k].tag_position.y);
		k++;
	}
	SDI_F->tagsPosition.nbTags = k;

	old_odo[0] = odo_position[0];
	old_odo[1] = odo_position[1];
	old_odo[2] = odo_position[2];
	deleteTagDetections(&tagDetectionSet);
  /* ... add your code here ... */
  return ETHER;
}

/* rfidLocMapActualizePositionsInter  -  codel INTER of ActualizePositions
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapActualizePositionsInter(int *report) {
	printf("rfidLocMapActualizePositionsInter \n");
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
ACTIVITY_EVENT rfidLocMapStartRobotParticlesStart(POSITION *position,
		int *report) {
  /* ... add your code here ... */
	RobotParticle robotParticle(position->xRob,position->yRob,position->theta,0);

	robotParticles = RobotParticles(robotParticle,numberParticlesRobot);

	SDI_F->position.xRob = position->xRob;
	SDI_F->position.yRob = position->yRob;
	SDI_F->position.theta = position->theta;

	/* Set new variances on (x,y,theta)
	 With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */
	SDI_F->estimationError.position_cov[0] = 0;
	SDI_F->estimationError.position_cov[1] = 0;
	SDI_F->estimationError.position_cov[2] = 0;
	SDI_F->estimationError.position_cov[3] = 0;
	SDI_F->estimationError.position_cov[4] = 0;
	SDI_F->estimationError.position_cov[5] = 0;
  return ETHER;
}

/* rfidLocMapStartRobotParticlesInter  -  codel INTER of StartRobotParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapStartRobotParticlesInter(POSITION *position,
		int *report) {
  printf("rfidLocMapStartRobotParticlesInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * StartOdometry
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapStartOdometryStart  -  codel START of StartOdometry
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapStartOdometryStart(POSITION *position, int *report)
{
  /* ... add your code here ... */
	old_odo[0] = position->xRob;
	old_odo[1] = position->yRob;
	old_odo[2] = position->theta;
  return ETHER;
}

/* rfidLocMapStartOdometryInter  -  codel INTER of StartOdometry
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapStartOdometryInter(POSITION *position, int *report)
{
  /* ... add your code here ... */
	printf("rfidLocMapStartOdometryInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * StartInput
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapStartInputStart  -  codel START of StartInput
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapStartInputStart(int *report)
{
  /* ... add your code here ... */

	if (initInput() != 0) {
		printf("Problems in input initialization\n");
	}

  return ETHER;
}

/* rfidLocMapStartInputInter  -  codel INTER of StartInput
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapStartInputInter(int *report)
{
  /* ... add your code here ... */
	printf("rfidLocMapStartInputInter\n");
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
ACTIVITY_EVENT rfidLocMapSetNumberRobotParticlesStart(int *numberParticles,
		int *report) {


	numberParticlesRobot = *numberParticles;

	rfidLocMapStartRobotParticlesStart(&(SDI_F->position),report);

  /* ... add your code here ... */
  return ETHER;
}

/* rfidLocMapSetNumberRobotParticlesInter  -  codel INTER of SetNumberRobotParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapSetNumberRobotParticlesInter(int *numberParticles,
		int *report) {
	printf("rfidLocMapSetNumberRobotParticlesInter\n");
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
ACTIVITY_EVENT rfidLocMapGetNumberRobotParticlesStart(int *numberParticles,
		int *report) {
	*numberParticles = numberParticlesRobot;
  return ETHER;
}

/* rfidLocMapGetNumberRobotParticlesInter  -  codel INTER of GetNumberRobotParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapGetNumberRobotParticlesInter(int *numberParticles,
		int *report) {
  	printf("rfidLocMapGetNumberRobotParticlesInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * SetNumberTagParticles
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapSetNumberTagParticlesStart  -  codel START of SetNumberTagParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetNumberTagParticlesStart(int *numberParticles, int *report)
{
  numberParticlesTag = *numberParticles;
  return ETHER;
}

/* rfidLocMapSetNumberTagParticlesInter  -  codel INTER of SetNumberTagParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetNumberTagParticlesInter(int *numberParticles, int *report)
{
  printf("rfidLocMapSetNumberTagParticlesInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * GetNumberTagParticles
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapGetNumberTagParticlesStart  -  codel START of GetNumberTagParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetNumberTagParticlesStart(int *numberParticles, int *report)
{
	*numberParticles = numberParticlesTag;
  return ETHER;
}

/* rfidLocMapGetNumberTagParticlesInter  -  codel INTER of GetNumberTagParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetNumberTagParticlesInter(int *numberParticles, int *report)
{
  printf("rfidLocMapGetNumberTagParticlesInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * SetRobotInertia
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapSetRobotInertiaStart  -  codel START of SetRobotInertia
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetRobotInertiaStart(double *inertia, int *report)
{
  inertiaRobot = *inertia;
  return ETHER;
}

/* rfidLocMapSetRobotInertiaInter  -  codel INTER of SetRobotInertia
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetRobotInertiaInter(double *inertia, int *report)
{
  printf("rfidLocMapSetRobotInertiaInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * GetRobotInertia
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapGetRobotInertiaStart  -  codel START of GetRobotInertia
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetRobotInertiaStart(double *inertia, int *report)
{
  *inertia = inertiaRobot;
  return ETHER;
}

/* rfidLocMapGetRobotInertiaInter  -  codel INTER of GetRobotInertia
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetRobotInertiaInter(double *inertia, int *report)
{
  printf("rfidLocMapGetRobotInertiaInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * SetTagInertia
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapSetTagInertiaStart  -  codel START of SetTagInertia
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetTagInertiaStart(double *inertia, int *report)
{
  inertiaTag = *inertia;
  return ETHER;
}

/* rfidLocMapSetTagInertiaInter  -  codel INTER of SetTagInertia
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapSetTagInertiaInter(double *inertia, int *report)
{
  printf("rfidLocMapSetTagInertiaInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * GetTagInertia
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidLocMapGetTagInertiaStart  -  codel START of GetTagInertia
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetTagInertiaStart(double *inertia, int *report)
{
  *inertia = inertiaTag;
  return ETHER;
}

/* rfidLocMapGetTagInertiaInter  -  codel INTER of GetTagInertia
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidLocMapGetTagInertiaInter(double *inertia, int *report)
{
  printf("rfidLocMapGetTagInertiaStart\n");
  return ETHER;
}


