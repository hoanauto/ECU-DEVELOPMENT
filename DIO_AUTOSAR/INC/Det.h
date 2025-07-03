// Det.h
#ifndef DET_H
#define DET_H

#include <stdint.h>
#include <stdio.h>


#define DIO_WRITECHANNEL_ID           0x01
#define DIO_READCHANNEL_ID            0x02
#define DIO_WRITEPORT_ID              0x03
#define DIO_E_PARAM_INVALID_CHANNEL   0x0A
#define DIO_E_PARAM_INVALID_PORT      0x0B

static inline void Det_ReportError(uint16_t module_id, uint8_t instance_id,
                                   uint8_t api_id, uint8_t error_id)
{
    printf("[DET] Error in module %u, API %u: error code %u\n", module_id, api_id, error_id);
}

#endif
