#include <stdint.h>
#include <stdlib.h>
#include "timing_position.h"
#define TRUE 1
#define FALSE 0

uint32_t arr_time[DATAPOINTS];
uint8_t pos;
int32_t position = 0;
uint8_t previous_position = 0;
int total_count;

int motor_get_position(void)
{
  return position;
}

void timing_init(uint8_t pha, uint8_t phb) {
    pos = 0;
    total_count = 0;
    previous_position = (pha << 1) | phb;
    for (uint8_t i = 0; i < DATAPOINTS; i++) {
      arr_time[i] = 0;
    }
}

void insert_time(uint32_t time) {
  static uint32_t previous_time;
  if(previous_time <= time)
    arr_time[pos] = time - previous_time;
  else
    arr_time[pos] = (time+0x7fffffff) - (previous_time - 0x7fffffff);
  pos = (pos + 1) % DATAPOINTS;
  previous_time = time;
}

void update_position(uint8_t pha, uint8_t phb)
{
    uint8_t position_val = (pha << 1) | phb;
    switch(previous_position) {
      case (0):
        switch(position_val) {
          case (0): break;
          case (1): ++position; break;
          case (2): --position; break;
          case (3): break;
        }
        break;
      case (1):
        switch(position_val) {
          case (0): --position; break;
          case (1): break;
          case (2): break;
          case (3): ++position; break;
        }
        break;
      case (2):
        switch(position_val) {
          case (0): ++position; break;
          case (1): break;
          case (2): break;
          case (3): --position; break;
        }
        break;
      case (3):
        switch(position_val) {
          case (0): break;
          case (1): --position; break;
          case (2): ++position; break;
          case (3): break;
        }
    }
    previous_position = position_val;
}


int avg_ticks_per_interrupt() {
    int curr_point = pos;
    int32_t time_diff = 0;
    uint32_t sum_time_diff = 0;
    int count = 0;
    for ( int i = 0; i < DATAPOINTS - 1; i++ ) {
        sum_time_diff += arr_time[i];
    }

    return (sum_time_diff/DATAPOINTS) ;
}

int32_t motor_get_rpm(void)
{
  return (72000/avg_ticks_per_interrupt());
}
