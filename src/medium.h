#ifndef MEDIUM_H
#define MEDIUM_H

#include "aito.h"

struct Medium;

// Implemented later
struct MediumInterface
{
	MediumInterface();
	MediumInterface(const Medium* medium);
};


#endif // MEDIUM_H
