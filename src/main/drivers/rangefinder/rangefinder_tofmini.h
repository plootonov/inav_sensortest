/*
 * Test for TOF mini Waveshare
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

bool tofminiRangefinderDetect(void);
void tofminiRangefinderInit(void);
bool tofminiRangefinderUpdate(void);
int32_t tofminiRangefinderGetDistanceCm(void);
bool tofminiRangefinderIsHealthy(void);
