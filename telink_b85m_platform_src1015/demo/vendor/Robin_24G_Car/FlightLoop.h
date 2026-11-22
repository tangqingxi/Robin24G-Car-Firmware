#ifndef FLIGHT_LOOP_H
#define FLIGHT_LOOP_H

void FlightLoop_Init(void);
void FlightLoop_Process(void);
void FlightLoop_OnRcPacket(unsigned short thr, unsigned short rot,
						   unsigned short fb, unsigned short lr,
						   unsigned short key);
void FlightLoop_RecordRx(unsigned int ticks);
void FlightLoop_RecordRfLoop(unsigned int ticks);
unsigned char FlightLoop_DiagActive(void);

#endif /* FLIGHT_LOOP_H */
