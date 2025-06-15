#ifndef RADAR_ACQUISITION_H
#define RADAR_ACQUISITION_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "radar_processing.h"

// --- Acquisition Task Management ---
void radar_acquisition_task_create(void);
void radar_acquisition_task(void *pvParameters);

// --- IRQ Handling ---
void IRAM_ATTR gpio_radar_isr_handler(void *arg);

// --- Global Handles ---
extern SemaphoreHandle_t xFifoIRQSemaphore;

#endif /* RADAR_ACQUISITION_H */