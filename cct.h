#ifndef CCT_H_
#define CCT_H_


//esp8266 is a 80Mhz clock
#define  CCT_TICKS_PER_US 	80
#define  SYSTEM_CLOCK_HZ 	80000000

////esp32 is a 240Mhz clock
//
//#define CCT_TICKS_PER_US 	240
//#define SYSTEM_CLOCK_HZ		240000000

void 		cct_DelayUs(uint32_t us);
uint32_t  	cct_IntervalUs(uint32_t before, uint32_t after);
float  		cct_IntervalSecs(uint32_t before, uint32_t after);
void 		cct_SetMarker(void);
uint32_t 	cct_ElapsedTimeUs(void);

#endif