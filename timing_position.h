#pragma once

#ifndef _TIMING_POSITION_H_INCLUDED_
#define _TIMING_POSITION_H_INCLUDED_

#define DATAPOINTS 21

#include <stdint.h>
#include <stdbool.h>


void timing_init(uint8_t pha, uint8_t phb);


uint32_t last_point();


void update_position(uint8_t pha, uint8_t phb);


void insert_time(uint32_t time);


int avg_ticks_per_interrupt();
#endif
