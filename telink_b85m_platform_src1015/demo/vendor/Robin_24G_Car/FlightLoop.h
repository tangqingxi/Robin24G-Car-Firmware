#ifndef FLIGHT_LOOP_H
#define FLIGHT_LOOP_H

void FlightLoop_Init(void);
void FlightLoop_Process(void);
void FlightLoop_PreSampleSafety(void);
void FlightLoop_OnRcPacket(unsigned short thr, unsigned short rot,
						   unsigned short fb, unsigned short lr,
						   unsigned short key);
void FlightLoop_ResetDiagStats(void);
void FlightLoop_GetDiagStatus(unsigned char *armed, unsigned short pwm[4],
							  unsigned short *max_gap_ms,
							  unsigned short *missed_ticks);

#endif /* FLIGHT_LOOP_H */
