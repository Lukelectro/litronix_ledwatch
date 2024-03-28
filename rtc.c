/*
 * File:   rtc.c
 * Author: lucas
 *
 * Created on May 15, 2023, 4:14 PM
 */


#include <xc.h>
#include "rtc.h"

uint8_t seconds=0, minutes=0, hours=0, dayofweek=0, days=1, months=1;
uint16_t years=1990;

void checkdays();

void clocktick(){
    seconds++;
    if(seconds >= 60){
        minutes++;
        seconds=0;
    }
    if(minutes>=60){
        hours++;
        minutes=0;
    }
    if(hours>=24){
        dayofweek++;
        days++;
        hours=0;
        checkdays(); 
    }       
}

/*
 *  now I get why normaly seconds sinds epoch is stored and only converted on display/readrequest... 
 *  it makes timekeeping more efficient as it is then just seconds++
 *  and only at conversion a bit of extra calculation is needed, but that wont happen every second
 * Oh well...
 * TODO: convert rtc to use seconds since epoch instead?
 */

void checkdays(){
if(dayofweek>=6){
        dayofweek=0;
    }
    switch(months){
        // 31 days fallthrough
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
            if(days>=32){
            days = 1;
            months++;
            }
            break;
            // 30 days falltrough:
        case 4:
        case 6:
        case 9:
        case 11:
            if(days>=31){
            days = 1;
            months++;
            }
            break;
        case 2: // february...
            if(years%4==0){
            //leap years, 29 days
                if(days>=30){
                    days = 1;
                    months++;
                }
            } 
            else 
            {
                if(days>=29){
                    days = 1;
                    months++;
                }
            }
            break;
        default:
            //should not happen
            days = 99;
            months = 99;
            dayofweek=99;
            break;
    }
    if(months>=13){
        months = 1;
    }
}

void gettime_hms(uint8_t *h, uint8_t *m, uint8_t *s){
    *h=hours; *m=minutes; *s=seconds;
};

void gettime_hm(uint8_t *h, uint8_t *m){
    *h=hours; *m=minutes;
};

void settime_hms(uint8_t h, uint8_t m, uint8_t s){
    hours=h; minutes=m; seconds=s;
};

void settime_date(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t M , uint8_t y1 , uint8_t y0, uint8_t dow) // h,m,s,d,m,y1,y0,dow
{
   hours=h; minutes=m; seconds=s; dayofweek=dow; days=d; months=M; years=y1*100+y0; 
}
void gettime_h(uint8_t *h){
    *h=hours;
};
void gettime_m(uint8_t *m){
   *m=minutes;
};
void gettime_s(uint8_t *s){
    *s=seconds;
};
void gettime_date(uint8_t *dow, uint8_t *d,uint8_t *m,uint16_t *y){
 *dow = dayofweek; *d=days; *m=months; *y=years;
}