#include "Dio_Config.h"

// Cấu hình 3 kênh GPIO: PC13 (LED), PA0 (Nút), PB0 (Đèn điều khiển)
const Dio_ChannelType Dio_ConfiguredChannels[DIO_CONFIGURED_CHANNELS] = {
    DIO_CHANNEL_C13,  // PC13 → LED tự nhấp nháy
    DIO_CHANNEL_A0,   // PA0 → Nút nhấn
    DIO_CHANNEL_B0    // PB0 → LED điều khiển bằng nút
};
