#ifndef __MARKERS__
#define __MARKERS__

#include "stdbool.h"

//Predefined markers
#define MARK_NO 0
#define MARK_NO_CHAR 'N'
#define	MARK_R_PEAK		'R'
#define	MARK_WINDOW	'W'
#define	MARK_S_PEAK		'S'
#define MARK_FVT_START	'B'
#define MARK_FVT_FINISH	'O'

void AppendMarker(int* dest, int marker);
void ParseMarkers(int marker_signs, char* markers,	int size);
bool SearchFor(uint8_t marker, int marker_signs);

#endif