/************************************************************
    file: Std_Types.h
 * @brief định nghĩa các kiểu dữ liệu chuẩn cho Autosar
 * @details file này cung cấp các kiểu dữ liệu
            nền tảng độc lập với phần cứng
            và trình phiên dịch cho các module
            phần mềm cơ bản trong Autosar
*************************************************************/

#ifndef STD_TYPES_H
#define STD_TYPES_H

#include <stdint.h>




#define DIO_VENDOR_ID  123U
#define DIO_MODULE_ID  456U
#define STD_TYPES_SW_MAJOR_VERSION  1U
#define STD_TYPES_SW_MINOR_VERSION  0U
#define STD_TYPES_SW_PATCH_VERSION  0U 

/* Boolean Data Types */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Standard Return Type */
typedef uint8_t Std_ReturnType;
// 2 giá trị trả về của Std_ReturnType//
#define E_OK        0x00U  /* Operation successful */
#define E_NOT_OK    0x01U  /* Operation not successful */

// logical state Definitions
/************************************************************
 *@brief định nghĩa các trạng thái logic cao hay thấp
 *@details được use cho các tín hiệu đầu vào, đầu ra
*************************************************************/
#define STD_HIGH 0x01U // trạng thái logic cao
#define STD_LOW  0x00U // trạng thái logic cao
/* Null Pointer Definition */
#define NULL_PTR    ((void*)0)

/* Standard Version Information Data Type */
typedef struct {
    uint16_t vendorID; // ID nhà cung cấp
    uint16_t moduleID; // ID module
    uint8_t  sw_major_version; //Phiên bản chính của phần mềm
    uint8_t  sw_minor_version; // Phiên bản phụ của phần mềm
    uint8_t  sw_patch_version; // Phiên bản sửa lỗi của phần mềm
} Std_VersionInfoType;
/* Định nghĩa kiểu dữ liệu cho các giá trị số nguyên không dấu */
typedef uint8_t    uint8;    /**< 8-bit unsigned integer */
typedef uint16_t   uint16;   /**< 16-bit unsigned integer */
typedef uint32_t   uint32;   /**< 32-bit unsigned integer */

/* Định nghĩa kiểu dữ liệu cho các giá trị số nguyên có dấu */
typedef int8_t     sint8;    /**< 8-bit signed integer */
typedef int16_t    sint16;   /**< 16-bit signed integer */
typedef int32_t    sint32;   /**< 32-bit signed integer */

/* Định nghĩa kiểu dữ liệu cho các giá trị thực */
typedef float      float32;  /**< 32-bit floating point */
typedef double     float64;
/* Định nghĩa kiểu dữ liệu boolean */
typedef uint8_t    boolean;  /**< Boolean type (0 or 1) */

#endif /* STD_TYPES_H */
