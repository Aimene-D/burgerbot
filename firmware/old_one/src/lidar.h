#pragma once
#include <Arduino.h>

void initLidar();
bool lidarScanReady();
void lidarGetScan(float* ranges, float* intensities,
                  uint32_t* scan_start_ms, uint32_t* scan_duration_ms);
