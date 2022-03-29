#ifndef HDCHAL_H
#define HDCHAL_H

#include <stdint.h> //for uint
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
    bool hdcHALInit(uint8_t addr);
    uint8_t readReg(uint8_t reg);
    bool writeReg(uint8_t reg, uint8_t data);
#ifdef __cplusplus
}
#endif

#endif
