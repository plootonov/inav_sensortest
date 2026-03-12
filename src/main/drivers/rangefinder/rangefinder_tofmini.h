/*
 * Test for TOF mini Waveshare
 */

#pragma once

#include <stdbool.h>
#include "drivers/rangefinder/rangefinder.h"

#define RANGEFINDER_TOFMINI_TASK_PERIOD_MS 100

bool tofminiDetect(rangefinderDev_t *dev);
