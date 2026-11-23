#ifndef GROUND_DIAG_H
#define GROUND_DIAG_H

#include <types.h>

void GroundDiag_Init(void);
void GroundDiag_OnControl(int yaw_stick, const int tgt[3], const int gyro[3],
						  const int corr[3], const unsigned short pwm[4],
						  const short raw_acc[3], const int q[4],
						  unsigned char armed, unsigned char limited);
void GroundDiag_Process(void);

#endif
