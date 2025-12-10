#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>

typedef struct CRSF_Data {
    uint16_t channels[16];   // raw 11-bit values
    uint32_t last_update_ms;
    uint8_t  failsafe;
} CRSF_Data;

extern CRSF_Data crsf;   // deklarasi global

void CRSF_ProcessByte(uint8_t b);
void CRSF_PrintChannels(void);

#endif // CRSF_H
