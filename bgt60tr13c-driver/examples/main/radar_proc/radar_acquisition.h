#ifndef RADAR_ACQUISITION_H
#define RADAR_ACQUISITION_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "radar_processing.h"

/**
 * @brief Initializes and creates the radar acquisition task and required semaphores.
 */
void radar_acquisition_task_create(void);

/**
 * @brief Signals the radar acquisition task to start capturing a single frame.
 */
void radar_acquisition_trigger_frame(void);

/**
 * @brief Waits until the radar acquisition task is ready to accept a new frame trigger.
 * @return pdTRUE if the semaphore was taken, pdFALSE on timeout.
 */
BaseType_t radar_acquisition_wait_for_ready(TickType_t xTicksToWait);

/**
 * @brief ISR handler for the radar's FIFO interrupt.
 * @note The IRAM_ATTR is placed on the function definition in the .c file.
 */
void gpio_radar_isr_handler(void *arg);

#endif // RADAR_ACQUISITION_H
