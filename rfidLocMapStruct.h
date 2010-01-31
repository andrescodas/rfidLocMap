#ifndef RFID_POSITIONER_STRUCT_H
#define RFID_POSITIONER_STRUCT_H

#include "rfidLocMapConst.h"


typedef struct POSITION
{
	double xRob;
	double yRob;
	double theta;
} POSITION;


/* Set new variances on (x,y,theta)
   With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */
typedef struct POSITION_ERROR
{
	double position_cov[6];
} POSITION_ERROR;

typedef struct TAG_POSITION
{
	double x;
	double y;
} TAG_POSITION;

typedef struct TAG_POSITION_ERROR
{
	double tag_position_cov[3];
} TAG_POSITION_ERROR;

typedef struct TAG_ID
{
	char tagId[16];
} TAG_ID;

typedef struct TAG_POSITION_LIST
{
	int nbTags;
	TAG_ID tagId_List[TAG_POSITION_LIST_MAX];
	TAG_POSITION tag_positions[TAG_POSITION_LIST_MAX];
	TAG_POSITION_ERROR tag_positions_error[TAG_POSITION_LIST_MAX];
} TAG_POSITION_LIST;


#endif /* RFID_POSITIONER_STRUCT_H */
