#ifndef FLIGHT_LOOP_H
#define FLIGHT_LOOP_H

void FlightLoop_Init(void);
void FlightLoop_Process(void);
void FlightLoop_PreSampleSafety(void);
void FlightLoop_OnRcPacket(unsigned short thr, unsigned short rot,
						   unsigned short fb, unsigned short lr,
						   unsigned short key);

#endif /* FLIGHT_LOOP_H */
