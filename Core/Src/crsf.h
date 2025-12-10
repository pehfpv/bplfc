#ifndef _CRSF_H_
#define _CRSF_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define CRSF_SYNC 0xC8

void crsf_init(void);
void crsf_parse(uint8_t *buf);
void crsf_on_byte(uint8_t b);

extern uint8_t rx_byte;
extern uint16_t crsf_raw[16];
extern uint16_t crsf_channels_us[16];
extern uint32_t crsf_last_packet_ms;
extern uint8_t crsf_packet_received;

#endif
