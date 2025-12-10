#include "crsf.h"

uint8_t rx_byte = 0;    // <--- WAJIB ADA
uint8_t crsf_buf[64];
uint8_t crsf_index = 0;
uint8_t crsf_frame_len = 0;

uint16_t crsf_raw[16];
uint16_t crsf_channels_us[16];
uint32_t crsf_last_packet_ms = 0;
uint8_t crsf_packet_received = 0;

static uint32_t millis(){ return HAL_GetTick(); }

static int crsf_raw_to_us(uint16_t v){
    return ((v - 172) * 1000 / (1811 - 172)) + 1000;
}

void crsf_init(void)
{
    crsf_index = 0;
}

void crsf_parse(uint8_t *buf)
{
    if(buf[2] != 0x16) return; // RC_CHANNELS_PACKED

    uint8_t* p = &buf[3];
    uint32_t bitbuf = 0;
    uint8_t bitcount = 0;

    for(int ch=0; ch<16; ch++) {

        while(bitcount < 11) {
            bitbuf |= ((uint32_t)(*p++)) << bitcount;
            bitcount += 8;
        }

        crsf_raw[ch] = bitbuf & 0x7FF;
        bitbuf >>= 11;
        bitcount -= 11;

        crsf_channels_us[ch] = crsf_raw_to_us(crsf_raw[ch]);
    }

    crsf_last_packet_ms = millis();
    crsf_packet_received = 1;
}

void crsf_on_byte(uint8_t b)
{
    if (crsf_index == 0)
    {
        if (b == CRSF_SYNC) {
            crsf_buf[crsf_index++] = b;
        }
    }
    else if (crsf_index == 1)
    {
        crsf_frame_len = b;
        crsf_buf[crsf_index++] = b;

        if (crsf_frame_len > 62)
            crsf_index = 0;
    }
    else
    {
        crsf_buf[crsf_index++] = b;

        if (crsf_index == crsf_frame_len + 2)
        {
            crsf_parse(crsf_buf);
            crsf_index = 0;
        }
    }
}
