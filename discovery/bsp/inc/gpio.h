/**
 ******************************************************************************
 * @file      gpio.h
 * @brief     C Source file of gpio.h.
 * @details   This file including all API functions's 
 *            implement of gpio.h.	
 *
 * @copyright
 ******************************************************************************
 */
#ifndef __GPIO_H__
#define __GPIO_H__

#include <types.h>

typedef enum
{
    IO_LED3 = 0,        /* LED3        */
    IO_LED4,            /* LED4        */
    IO_LED5,            /* LED5        */
    IO_LED6,            /* LED6        */
    IO_KEY0,            /* KEY1        */
} gpio_out_e;

extern status_t gpio_init(void);

#endif /* __GPIO_H__ */
