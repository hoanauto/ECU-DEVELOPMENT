#ifndef DIO_CONFIG_H
#define DIO_CONFIG_H

#include "DIO.h"

// Số lượng kênh được cấu hình
#define DIO_CONFIGURED_CHANNELS 3

// Các index để dễ truy cập
#define LED_PC13_INDEX   0   // LED nhấp nháy tại PC13
#define BUTTON_A0_INDEX  1   // Nút nhấn tại PA0
#define LED_B0_INDEX     2   // Đèn điều khiển bằng nút tại PB0

extern const Dio_ChannelType Dio_ConfiguredChannels[DIO_CONFIGURED_CHANNELS];

#endif /* DIO_CONFIG_H */

