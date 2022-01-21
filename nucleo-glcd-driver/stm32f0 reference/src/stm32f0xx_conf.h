#ifndef _STM32F0XX_CONF_H
#define _STM32F0XX_CONF_H

#define assert_param(expr) ((expr) ? (int8_t)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
int8_t assert_failed(uint8_t* file, uint32_t line);

#endif
