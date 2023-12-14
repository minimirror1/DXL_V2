/*
 * mrs_task.cpp
 *
 *  Created on: Dec 14, 2023
 *      Author: minim
 */

#include "main.h"

/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mrs_task.h"
#include "cpp_tick.h"



void MrsTask(void *argument)
{
  /* USER CODE BEGIN MrsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MrsTask */
}
