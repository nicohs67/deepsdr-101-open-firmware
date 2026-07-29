#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "gd32f4xx.h"

/*
 * Debug por USART0: PA9 = TX (AF7), 115200 8N1 (o 57600 si a ti te
 * funciono a esa velocidad).
 */

void debug_uart_init(void);
void debug_print(const char *s);
void debug_print_hex32(const char *label, uint32_t val);
void debug_print_hex16(const char *label, uint16_t val);
void debug_print_dec(const char *label, uint32_t val);

#endif
