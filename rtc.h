#ifndef rtc_H
#define	rtc_H

#include <xc.h> // include processor files - each processor file is guarded.  


void gettime_hms(uint8_t*, uint8_t*, uint8_t*);
void settime_hms(uint8_t, uint8_t, uint8_t);
void settime_date(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t); // h,m,s,d,m,y1,y0,dow
void clocktick(void);

void gettime_hm(uint8_t*, uint8_t*);
void gettime_h(uint8_t*);
void gettime_m(uint8_t*);
void gettime_s(uint8_t*);
void gettime_date(uint8_t *dow, uint8_t *d,uint8_t *m,uint16_t *y);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

