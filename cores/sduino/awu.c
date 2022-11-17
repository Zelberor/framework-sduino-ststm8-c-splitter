#include "awu_private.h"
#include "awu.h"
#include "Arduino.h"

extern volatile unsigned long awu_irq_count;

// <--#SPLIT#--> //

#ifndef NO_AWU
volatile unsigned long awu_irq_count = 0;

void awu_irq()
{
	++awu_irq_count;
	/* Clear AWU peripheral pending bit */
	AWU_GetFlagStatus();
}

#endif

// <--#SPLIT#--> //

unsigned long awu_count()
{
	return awu_irq_count;
}
