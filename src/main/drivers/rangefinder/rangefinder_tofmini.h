/*
 * Test for TOF mini Waveshare
 */

#pragma once

#include <stdbool.h>
#include "drivers/rangefinder/rangefinder.h"

bool tofminiDetect(rangefinderDev_t *dev);
