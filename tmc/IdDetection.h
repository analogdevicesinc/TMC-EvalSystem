#ifndef ID_DETECTION_H
#define ID_DETECTION_H

	#include "BoardAssignment.h"

	// id detection state definitions
	#define ID_STATE_WAIT_LOW   0  // id detection waiting for first edge (currently low)
	#define ID_STATE_WAIT_HIGH  1  // id detection waiting for second edge (currently high)
	#define ID_STATE_DONE       2  // id detection finished successfully
	#define ID_STATE_INVALID    3  // id detection failed - we got an answer, but no corresponding ID (invalid ID pulse length)
	#define ID_STATE_NO_ANSWER  4  // id detection failed - board doesn't answer
	#define ID_STATE_TIMEOUT    5  // id detection failed - board id pulse went high but not low
	#define ID_STATE_NOT_IN_FW  6  // id detection detected a valid id that is not supported in this firmware

	void IDDetection_init(void);
	void IDDetection_deInit(void);
	uint8 IDDetection_detect(IdAssignmentTypeDef *out);
	void IDDetection_initialScan(IdAssignmentTypeDef *ids);

#endif /* ID_DETECTION_H */
