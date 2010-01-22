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


const int numberParticlesRobot = 200;
const int numberParticlesTag = 100;
const double inertiaTag = 0.98;
const double inertiaRobot = 0.95;

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
	tagMap = initTagMap();

	RobotParticle robotParticle(0,0,0,0);
	robotParticles = RobotParticles(robotParticle,numberParticlesRobot);

	if (initInput() != 0) {
		printf("Problems in input initialization\n");
	}

	SDI_F->position.xRob = 0;
	SDI_F->position.yRob = 0;
	SDI_F->position.theta = 0;
	SDI_F->tagsPosition.nbTags = 0;

	for (int i = 0; i < 5; ++i) {
		SDI_F->estimationError.position_cov[i] = 0;
	}

	initSensorModel();

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

	double quality;
	double odo_position[3];
	double odo_cov[3][3];

	TagDetectionSet tagDetectionSet;
	TagDetectionSet fixedTagsDetectionSet;
	TagDetectionSet inferringTagsDetectionSet;
	TagDetectionSet knownInferringTagsDetection;
	TagDetectionSet newInferringTagsDetection;
	TagDetectionSet tagDetectionIt;
	TagDetectionSet::iterator detectionIt;

	TagParticles *newDetectedTag;

	if (readRflex(odo_position, odo_cov) != 0) {
		return ETHER;
	}
	if (readRFID(&tagDetectionSet) != 0) {
		return ETHER;
	}

	sortDetectionsByTagMap(&fixedTagsDetectionSet,&inferringTagsDetectionSet,&tagDetectionSet,&tagMap);

	locateRobot(&fixedTagsDetectionSet,odo_position,old_odo,odo_cov,&tagMap,&robotParticles,inertiaRobot);

	sortDetectionsByTagParticlesMap(&knownInferringTagsDetection,&newInferringTagsDetection,&inferringTagsDetectionSet,&inferringTags);

	for(TagParticlesMap::iterator iTagIt = inferringTags.begin(); iTagIt != inferringTags.end();iTagIt++){

		tagDetectionIt.clear();
		sortDetectionsByTagid(&tagDetectionIt,&knownInferringTagsDetection, iTagIt->first);
		quality = weightNormalizeTags(&tagDetectionIt,(iTagIt->second),&robotParticles,iTagIt->first);

		if(quality > 1){
			printf("WeightNormalizeTags with quality > 1\n");
		}

		resampleExploreTags((iTagIt->second),&tagDetectionIt,inertiaTag+(1-inertiaTag)*quality,&robotParticles,numberParticlesTag,true);

	}

	detectionIt = newInferringTagsDetection.begin();
	while(detectionIt != newInferringTagsDetection.end()){

		tagDetectionIt.clear();
		sortDetectionsByTagid(&tagDetectionIt,&newInferringTagsDetection, (*detectionIt)->tagid);

		newDetectedTag = new TagParticles();

		resampleExploreTags(newDetectedTag,&tagDetectionIt,0,&robotParticles,numberParticlesTag,false);

		for(detectionIt = tagDetectionIt.begin();detectionIt != tagDetectionIt.end();detectionIt++){
			newInferringTagsDetection.erase((*detectionIt));
		}
		detectionIt = newInferringTagsDetection.begin();
	}


	old_odo[0] = odo_position[0];
	old_odo[1] = odo_position[1];
	old_odo[2] = odo_position[2];
	deleteTagDetections(&tagDetectionSet);
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
	return ETHER;
}

/* rfidLocMapStartRobotParticlesInter  -  codel INTER of StartRobotParticles
 Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapStartRobotParticlesInter(POSITION *position,
		int *report) {
	printf("rfidLocMapStartRobotParticlesInter");
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
	/* ... add your code here ... */
	return ETHER;
}

/* rfidLocMapSetNumberRobotParticlesInter  -  codel INTER of SetNumberRobotParticles
 Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapSetNumberRobotParticlesInter(int *numberParticles,
		int *report) {
	printf("rfidLocMapSetNumberRobotParticlesInter");
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
	/* ... add your code here ... */
	return ETHER;
}

/* rfidLocMapGetNumberRobotParticlesInter  -  codel INTER of GetNumberRobotParticles
 Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidLocMapGetNumberRobotParticlesInter(int *numberParticles,
		int *report) {
	printf("rfidLocMapGetNumberRobotParticlesInter");
	return ETHER;
}

