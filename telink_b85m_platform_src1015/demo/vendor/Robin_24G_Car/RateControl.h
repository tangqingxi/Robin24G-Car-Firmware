#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include "FlightConfig.h"

typedef struct __attribute__((aligned(4))) {
	int corr[3];
#if (FLIGHT_KI_Q16 != 0)
	int integ[3];
#endif
#if (FLIGHT_KD_Q8 != 0)
	int prev_err[3];
	unsigned char have_prev;
#endif
} RateControl;

void RateControl_Init(RateControl *rc);
void RateControl_Update(RateControl *rc, const int target[3],
						const int gyro[3]);
void RateControl_ClearOutputs(RateControl *rc);

#endif /* RATE_CONTROL_H */
