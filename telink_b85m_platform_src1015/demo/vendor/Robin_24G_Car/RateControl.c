#include "RateControl.h"

static int rate_clamp(int v, int lim)
{
	if (v > lim) {
		return lim;
	}
	if (v < -lim) {
		return -lim;
	}
	return v;
}

void RateControl_Init(RateControl *rc)
{
	int i;

	for (i = 0; i < 3; i++) {
		rc->corr[i] = 0;
#if (FLIGHT_KI_Q16 != 0)
		rc->integ[i] = 0;
#endif
#if (FLIGHT_KD_Q8 != 0)
		rc->prev_err[i] = 0;
#endif
	}
#if (FLIGHT_KD_Q8 != 0)
	rc->have_prev = 0;
#endif
}

// 内环：误差=目标角速度−实测角速度；P=64/256，I/D=0 时对应状态不编译。
void RateControl_Update(RateControl *rc, const int target[3],
						const int gyro[3])
{
	int i;

	for (i = 0; i < 3; i++) {
		int err = target[i] - gyro[i];
		int corr = (err * FLIGHT_KP_Q8) >> 8;

#if (FLIGHT_KI_Q16 != 0)
		rc->integ[i] += err;
		rc->integ[i] = rate_clamp(rc->integ[i], FLIGHT_RATE_INT_MAX);
		corr += (rc->integ[i] * FLIGHT_KI_Q16) >> 16;
#endif
#if (FLIGHT_KD_Q8 != 0)
		{
			int d_err = rc->have_prev ? (err - rc->prev_err[i]) : 0;

			rc->prev_err[i] = err;
			corr += (d_err * FLIGHT_KD_Q8) >> 8;
		}
#endif
		rc->corr[i] = rate_clamp(corr, FLIGHT_CORR_LIMIT_Q10);
	}
#if (FLIGHT_KD_Q8 != 0)
	rc->have_prev = 1;
#endif
}

void RateControl_ClearOutputs(RateControl *rc)
{
	int i;

	for (i = 0; i < 3; i++) {
		rc->corr[i] = 0;
#if (FLIGHT_KI_Q16 != 0)
		rc->integ[i] = 0;
#endif
#if (FLIGHT_KD_Q8 != 0)
		rc->prev_err[i] = 0;
#endif
	}
#if (FLIGHT_KD_Q8 != 0)
	rc->have_prev = 0;
#endif
}
